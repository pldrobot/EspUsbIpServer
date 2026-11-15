#include "UsbDevice.h"
#include "EspUsbIpLog.h"
#include <esp_heap_caps.h>

// Enumeration filter: runs inside the host library right after the device descriptor has
// been read, so seeing it proves reset, addressing and the first descriptor read worked.
// It only records the descriptor; the host task logs it (this callback must not block).
static volatile bool    g_enumSeen = false;
static usb_device_desc_t g_enumDesc;

static bool enumFilter(const usb_device_desc_t* desc, uint8_t* bConfigurationValue) {
    g_enumDesc = *desc;
    g_enumSeen = true;
    *bConfigurationValue = 1;
    return true;
}

UsbDevice::UsbDevice()
    : _state(DEVICE_DISCONNECTED), _maintenanceMode(false),
      _vendorId(0), _productId(0), _devAddr(0),
      _hostTask(nullptr), _clientHandle(nullptr), _deviceHandle(nullptr),
      _devMtx(nullptr),
      _gone(false), _closeReq(false), _closeStartMs(0), _relIdx(0), _pendingAddr(-1),
      _speed(USB_SPEED_FULL), _ifaceCount(0), _epCount(0),
      _devDescLen(0), _cfgDescLen(0) {}

// ── Lifecycle ────────────────────────────────────────────────────────────────

bool UsbDevice::begin() {
    if (_clientHandle) return true;

    _devMtx = xSemaphoreCreateMutex();
    if (!_devMtx) { _USBIP_LOGE("USB out of memory (mutex)"); return false; }

    usb_host_config_t hostCfg = { .skip_phy_setup = false,
                                  .intr_flags = ESP_INTR_FLAG_LEVEL1,
                                  .enum_filter_cb = enumFilter };
    if (usb_host_install(&hostCfg) != ESP_OK) {
        _USBIP_LOGE("USB Host install failed");
        return false;
    }

    usb_host_client_config_t clientCfg = {
        .is_synchronous = false,
        .max_num_event_msg = 15,
        .async = { .client_event_callback = _clientEventCallback,
                   .callback_arg = this }
    };
    if (usb_host_client_register(&clientCfg, &_clientHandle) != ESP_OK) {
        _USBIP_LOGE("USB Client register failed");
        usb_host_uninstall();
        return false;
    }

    xTaskCreatePinnedToCore(_hostTaskFn, "usb_host", USBIP_STACK_USB, this,
                            USBIP_PRIO_USB, &_hostTask, USBIP_CORE);
    _USBIP_LOGI("USB Host ready (core %d, free DMA heap %u B), waiting for device...", USBIP_CORE,
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    return true;
}

void UsbDevice::end() {
    if (!_clientHandle) return;
    // Let the client task finish closing a present device before it is stopped.
    if (_deviceHandle) {
        _gone = true; _state = DEVICE_DISCONNECTED;
        _closeStartMs = millis(); _relIdx = 0; _closeReq = true;
        for (int i = 0; i < 400 && _deviceHandle; i++) vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (_hostTask) { vTaskDelete(_hostTask); _hostTask = nullptr; }
    usb_host_client_deregister(_clientHandle);
    _clientHandle = nullptr;
    usb_host_uninstall();
    if (_devMtx) { vSemaphoreDelete(_devMtx); _devMtx = nullptr; }
    _USBIP_LOGI("USB Host uninstalled");
}

void UsbDevice::stop() {
    if (_hostTask) vTaskSuspend(_hostTask);
    _USBIP_LOGI("USB Host task suspended");
}

void UsbDevice::start() {
    if (_hostTask) vTaskResume(_hostTask);
    _USBIP_LOGI("USB Host task resumed");
}

// One task services both the host library (enumeration, hot-plug) and the client
// (device events and every transfer completion). The library is polled without
// blocking and the client waits only a few milliseconds, so a completion callback
// runs the moment its event arrives while library events are still serviced often.
void UsbDevice::_hostTaskFn(void* arg) {
    UsbDevice* self = static_cast<UsbDevice*>(arg);
    const uint32_t start = millis();
    uint32_t lastAdopt = 0;
    bool hinted = false;
    for (;;) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(0, &flags);
        if (g_enumSeen) {
            g_enumSeen = false;
            _USBIP_LOGD("USB enumeration: device descriptor read (VID:0x%04X PID:0x%04X class 0x%02X, "
                        "USB %x.%02x, ep0 mps %u)", g_enumDesc.idVendor, g_enumDesc.idProduct,
                        g_enumDesc.bDeviceClass, g_enumDesc.bcdUSB >> 8, g_enumDesc.bcdUSB & 0xFF,
                        g_enumDesc.bMaxPacketSize0);
        }
        if (flags) _USBIP_LOGD("USB host library event flags 0x%02X", (unsigned)flags);   // NO_CLIENTS / ALL_FREE

        // While closing, wake regularly: the host stack frees the device only after the
        // completion callbacks of its queued transfers have run on this very task.
        usb_host_client_handle_events(self->_clientHandle, pdMS_TO_TICKS(USBIP_USB_POLL_MS));
        if (self->_closeReq) self->_tryClose();

        // NEW_DEV only reaches clients registered when enumeration completes. A device that
        // was already plugged in at boot (or whose event was lost) would otherwise never be
        // picked up, so look for enumerated devices we do not hold, now and twice a second.
        if (!self->_deviceHandle && !self->_closeReq && millis() - lastAdopt >= 500) {
            lastAdopt = millis();
            self->_adoptExistingDevice();
        }

        if (!hinted && !self->_everSeen && millis() - start > 10000) {
            hinted = true;
            usb_host_lib_info_t li = {};
            usb_host_lib_info(&li);
            _USBIP_LOGW("USB no device detected after 10 s (host lib: %d device(s), %d client(s); "
                        "free DMA heap %u B, largest block %u B). Check the cable, that it is the OTG "
                        "port (not the UART port) and that the device gets VBUS power",
                        li.num_devices, li.num_clients,
                        (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
        }
    }
}

// ── Client events (usb_host task) ─────────────────────────────────────────────

void UsbDevice::_clientEventCallback(const usb_host_client_event_msg_t* msg, void* arg) {
    static_cast<UsbDevice*>(arg)->_handleClientEvent(msg);
}

void UsbDevice::_handleClientEvent(const usb_host_client_event_msg_t* msg) {
    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        _everSeen = true;
        _USBIP_LOGD("USB Device detected addr=%d", msg->new_dev.address);
        if (!_deviceHandle && !_closeReq) _openDevice(msg->new_dev.address);
        else                              _pendingAddr = msg->new_dev.address;   // retry after the close finishes
    } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        if (!_deviceHandle || msg->dev_gone.dev_hdl != _deviceHandle) return;   // not the device we hold
        _USBIP_LOGW("USB Device disconnected");
        _gone = true;
        _state = DEVICE_DISCONNECTED;
        _closeReq = true;
        _closeStartMs = millis();
        _relIdx = 0;
        if (_goneCb) _goneCb();
    }
}

String UsbDevice::_strDesc(const usb_str_desc_t* d) {
    if (!d || d->bLength < 2) return String();
    String s;
    int n = (d->bLength - 2) / 2;
    for (int i = 0; i < n; i++) {
        uint16_t c = d->wData[i];
        s += (c >= 0x20 && c < 0x7F) ? (char)c : '?';
    }
    return s;
}

void UsbDevice::_adoptExistingDevice() {
    uint8_t addrs[4];
    int n = 0;
    esp_err_t err = usb_host_device_addr_list_fill(sizeof(addrs), addrs, &n);
    static int shown = 0;                       // report the first few attempts: silence would hide a failure
    if (shown < 6 && (err != ESP_OK || n > 0 || shown == 0)) {
        shown++;
        _USBIP_LOGD("USB scan for already-enumerated devices: %s, %d found", esp_err_to_name(err), n);
    }
    if (err != ESP_OK || n <= 0) return;
    _everSeen = true;
    _USBIP_LOGI("USB Device already enumerated by the host (addr=%d): opening it", addrs[0]);
    _openDevice(addrs[0]);
}

bool UsbDevice::_openDevice(uint8_t devAddr) {
    usb_device_handle_t h = nullptr;
    if (usb_host_device_open(_clientHandle, devAddr, &h) != ESP_OK) {
        _USBIP_LOGE("USB Cannot open device");
        return false;
    }
    xSemaphoreTake(_devMtx, portMAX_DELAY);
    _deviceHandle = h;
    xSemaphoreGive(_devMtx);
    _gone    = false;
    _devAddr = devAddr;
    _state   = DEVICE_CONNECTED;

    const usb_device_desc_t* devDesc;
    if (usb_host_get_device_descriptor(_deviceHandle, &devDesc) == ESP_OK) {
        _vendorId   = devDesc->idVendor;
        _productId  = devDesc->idProduct;
        _devDescLen = sizeof(usb_device_desc_t) < USBIP_DEV_DESC_SIZE ? sizeof(usb_device_desc_t)
                                                                      : USBIP_DEV_DESC_SIZE;
        memcpy(_devDescBuf, devDesc, _devDescLen);
        _USBIP_LOGD("USB VID:0x%04X PID:0x%04X", _vendorId, _productId);
    }

    const usb_config_desc_t* cfgDesc;
    if (usb_host_get_active_config_descriptor(_deviceHandle, &cfgDesc) == ESP_OK) {
        if (cfgDesc->wTotalLength > USBIP_CFG_DESC_SIZE)
            _USBIP_LOGW("USB configuration descriptor is %u bytes, stored copy truncated to %d",
                        cfgDesc->wTotalLength, USBIP_CFG_DESC_SIZE);
        _cfgDescLen = min((uint16_t)USBIP_CFG_DESC_SIZE, cfgDesc->wTotalLength);
        memcpy(_cfgDescBuf, cfgDesc, _cfgDescLen);
    }

    usb_device_info_t info;
    if (usb_host_device_info(_deviceHandle, &info) == ESP_OK) {
        _speed        = info.speed;
        _manufacturer = _strDesc(info.str_desc_manufacturer);
        _product      = _strDesc(info.str_desc_product);
        _serial       = _strDesc(info.str_desc_serial_num);
    } else {
        _speed = USB_SPEED_FULL;
    }
    if (_manufacturer.isEmpty()) _manufacturer = String("VID:0x") + String(_vendorId, HEX);
    if (_product.isEmpty())      _product      = String("PID:0x") + String(_productId, HEX);
    _USBIP_LOGI("USB %s / %s", _manufacturer.c_str(), _product.c_str());

    if (!_enumerateInterfaces()) {
        _USBIP_LOGE("USB No usable interface found or cannot claim");
        _gone = true; _state = DEVICE_DISCONNECTED;
        _closeReq = true; _closeStartMs = millis(); _relIdx = 0;
        return false;
    }

    _state = DEVICE_READY;
    _USBIP_LOGI("USB Device ready (speed=%s, %d interface(s), %d endpoint(s))",
                _speed == USB_SPEED_LOW ? "low" : "full", _ifaceCount, _epCount);
    return true;
}

// Walk every interface in the active configuration (alternate setting 0 only),
// claiming each one and recording its class triple plus every bulk/interrupt
// endpoint it exposes. Isochronous endpoints and non-default alternate settings
// are skipped (out of scope).
bool UsbDevice::_enumerateInterfaces() {
    const usb_config_desc_t* cfgDesc;
    if (usb_host_get_active_config_descriptor(_deviceHandle, &cfgDesc) != ESP_OK)
        return false;

    int offset = 0;
    const uint8_t* p = (const uint8_t*)cfgDesc;
    uint16_t total   = cfgDesc->wTotalLength;

    while (offset < total) {
        const usb_standard_desc_t* desc = (const usb_standard_desc_t*)(p + offset);
        if (desc->bLength == 0) break;

        if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            const usb_intf_desc_t* iface = (const usb_intf_desc_t*)desc;

            if (iface->bAlternateSetting == 0 && _ifaceCount < USBIP_MAX_IFACES) {
                esp_err_t err = usb_host_interface_claim(_clientHandle, _deviceHandle,
                                                          iface->bInterfaceNumber,
                                                          iface->bAlternateSetting);
                if (err == ESP_OK) {
                    _ifaces[_ifaceCount++] = { iface->bInterfaceNumber, iface->bInterfaceClass,
                                                iface->bInterfaceSubClass, iface->bInterfaceProtocol };
                    _USBIP_LOGD("USB Interface #%d class=0x%02X/0x%02X/0x%02X claimed",
                                iface->bInterfaceNumber, iface->bInterfaceClass,
                                iface->bInterfaceSubClass, iface->bInterfaceProtocol);

                    // Class-specific descriptors (HID, CDC functional, ...) sit between the
                    // interface and its endpoints, so scan until the next interface and count
                    // only real endpoint descriptors.
                    int epOff  = offset + desc->bLength;
                    int epSeen = 0;
                    while (epOff < total && epSeen < iface->bNumEndpoints) {
                        const usb_standard_desc_t* raw = (const usb_standard_desc_t*)(p + epOff);
                        if (raw->bLength == 0) break;
                        if (raw->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) break;
                        if (raw->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
                            epSeen++;
                            const usb_ep_desc_t* ep = (const usb_ep_desc_t*)raw;
                            usb_transfer_type_t xtype = USB_EP_DESC_GET_XFERTYPE(ep);
                            if (xtype == USB_TRANSFER_TYPE_ISOCHRONOUS)
                                _USBIP_LOGW("USB iso ep=0x%02X skipped (isochronous not supported)",
                                            ep->bEndpointAddress);
                            if ((xtype == USB_TRANSFER_TYPE_BULK || xtype == USB_TRANSFER_TYPE_INTR) &&
                                _epCount < USBIP_MAX_EPS) {
                                _eps[_epCount++] = { ep->bEndpointAddress, (uint8_t)xtype,
                                                      ep->wMaxPacketSize, ep->bInterval,
                                                      iface->bInterfaceClass };
                                _USBIP_LOGD("USB %s ep=0x%02X mps=%d",
                                            xtype == USB_TRANSFER_TYPE_BULK ? "Bulk" : "Intr",
                                            ep->bEndpointAddress, ep->wMaxPacketSize);
                            }
                        }
                        epOff += raw->bLength;
                    }
                } else {
                    _USBIP_LOGW("USB Interface #%d claim failed: %s",
                                iface->bInterfaceNumber, esp_err_to_name(err));
                }
            }
        }
        offset += desc->bLength;
    }

    return (_ifaceCount > 0 && _epCount > 0);
}

bool UsbDevice::getIfaceDesc(int i, uint8_t out[4]) const {
    if (i < 0 || i >= _ifaceCount) return false;
    out[0] = _ifaces[i].ifaceClass;
    out[1] = _ifaces[i].ifaceSubClass;
    out[2] = _ifaces[i].ifaceProtocol;
    out[3] = 0;
    return true;
}

const UsbIpEndpoint* UsbDevice::findEndpoint(uint8_t address) const {
    for (int i = 0; i < _epCount; i++)
        if (_eps[i].address == address) return &_eps[i];
    return nullptr;
}

// ── Closing (usb_host task, between event batches) ────────────────────────────

void UsbDevice::_resetDeviceFields() {
    _ifaceCount = 0;
    _epCount    = 0;
    _devDescLen = _cfgDescLen = 0;
    _manufacturer = _product = _serial = "";
    _vendorId = _productId = 0;
    _devAddr = 0;
}

// Releases interfaces and closes the device without ever blocking. While transfers
// are still queued the stack answers ESP_ERR_INVALID_STATE; the caller simply tries
// again after the next batch of events, which delivers their completions.
bool UsbDevice::_tryClose() {
    if (_deviceHandle) {
        while (_relIdx < _ifaceCount) {
            esp_err_t err = usb_host_interface_release(_clientHandle, _deviceHandle,
                                                       _ifaces[_relIdx].ifaceNum);
            if (err == ESP_ERR_INVALID_STATE) {
                if (millis() - _closeStartMs > USBIP_CLOSE_RETRY_MS) break;   // give up below
                return false;
            }
            _relIdx++;
        }

        bool closed = false;
        xSemaphoreTake(_devMtx, portMAX_DELAY);
        esp_err_t err = usb_host_device_close(_clientHandle, _deviceHandle);
        if (err != ESP_ERR_INVALID_STATE) { _deviceHandle = nullptr; closed = true; }
        xSemaphoreGive(_devMtx);

        if (!closed) {
            if (millis() - _closeStartMs <= USBIP_CLOSE_RETRY_MS) return false;
            _USBIP_LOGE("USB device could not be closed in time; abandoning handle");
            xSemaphoreTake(_devMtx, portMAX_DELAY);
            _deviceHandle = nullptr;
            xSemaphoreGive(_devMtx);
        } else if (err != ESP_OK) {
            _USBIP_LOGW("USB device close: %s", esp_err_to_name(err));
        }
    }

    _resetDeviceFields();
    _state           = DEVICE_DISCONNECTED;
    _maintenanceMode = false;
    _closeReq        = false;
    _USBIP_LOGW("USB Device closed");

    if (_pendingAddr >= 0) {                       // a device was plugged in while we were closing
        int addr = _pendingAddr; _pendingAddr = -1;
        _openDevice((uint8_t)addr);
    }
    return true;
}

// ── Transfers ────────────────────────────────────────────────────────────────

esp_err_t UsbDevice::submitTransfer(usb_transfer_t* xfer) {
    esp_err_t e = ESP_ERR_INVALID_STATE;
    xSemaphoreTake(_devMtx, portMAX_DELAY);
    if (_deviceHandle && !_gone) {
        xfer->device_handle = _deviceHandle;
        e = usb_host_transfer_submit(xfer);
    }
    xSemaphoreGive(_devMtx);
    return e;
}

esp_err_t UsbDevice::submitControl(usb_transfer_t* xfer) {
    esp_err_t e = ESP_ERR_INVALID_STATE;
    xSemaphoreTake(_devMtx, portMAX_DELAY);
    if (_deviceHandle && !_gone) {
        xfer->device_handle = _deviceHandle;
        e = usb_host_transfer_submit_control(_clientHandle, xfer);
    }
    xSemaphoreGive(_devMtx);
    return e;
}

void UsbDevice::cancelEndpoint(uint8_t epAddr) {
    xSemaphoreTake(_devMtx, portMAX_DELAY);
    if (_deviceHandle && !_gone) {
        usb_host_endpoint_halt(_deviceHandle, epAddr);
        usb_host_endpoint_flush(_deviceHandle, epAddr);
    }
    xSemaphoreGive(_devMtx);
}

void UsbDevice::clearEndpoint(uint8_t epAddr) {
    xSemaphoreTake(_devMtx, portMAX_DELAY);
    if (_deviceHandle && !_gone) usb_host_endpoint_clear(_deviceHandle, epAddr);
    xSemaphoreGive(_devMtx);
}

void UsbDevice::setMaintenanceMode(bool active) {
    _maintenanceMode = active;
    UsbDeviceState from = active ? DEVICE_READY : DEVICE_MAINTENANCE;
    UsbDeviceState to   = active ? DEVICE_MAINTENANCE : DEVICE_READY;
    _state.compare_exchange_strong(from, to);
    _USBIP_LOGD("USB Maintenance lock: %s", active ? "ON" : "OFF");
}
