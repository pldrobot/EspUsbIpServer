#pragma once
#include <Arduino.h>
#include <atomic>
#include <functional>
#include "usb/usb_host.h"
#include "freertos/semphr.h"
#include "UsbIpConfig.h"

// USB standard request codes handled by the USB/IP layer instead of being forwarded
#define USB_REQ_SET_CONFIGURATION 0x09
#define USB_REQ_SET_INTERFACE     0x0B

enum UsbDeviceState {
    DEVICE_DISCONNECTED,
    DEVICE_CONNECTED,
    DEVICE_READY,
    DEVICE_MAINTENANCE    // a USB/IP client holds the device
};

struct UsbIpIface {
    uint8_t ifaceNum;
    uint8_t ifaceClass;
    uint8_t ifaceSubClass;
    uint8_t ifaceProtocol;
};

struct UsbIpEndpoint {
    uint8_t  address;        // bEndpointAddress, direction bit included
    uint8_t  type;           // usb_transfer_type_t: BULK or INTR
    uint16_t maxPacketSize;
    uint8_t  interval;
    uint8_t  ifaceClass;     // class of the interface that owns the endpoint
};

// USB host side: owns the host library, the client, and the one attached device.
//
// Threads: one `usb_host` task pumps the host library and the client. Transfer
// completion callbacks and device events all run on it. A device that went
// away is closed from that same task, in small steps between event batches, because
// the host stack only releases it once every queued transfer has been completed
// (by callbacks on this very task).
//
// Generic by design: any single full/low-speed device is exported. Every interface
// (alternate setting 0) is claimed; every bulk and interrupt endpoint is forwarded.
// Isochronous endpoints, alternate settings > 0, hubs and high speed are out of scope.
class UsbDevice {
public:
    using GoneCallback = std::function<void()>;

    UsbDevice();

    // Install the USB host and start its tasks. Safe to call once.
    bool begin();
    // Close the device, stop the tasks and uninstall the host.
    void end();

    // Suspend / resume the host tasks.
    void stop();
    void start();

    // Exclusive lock held by the USB/IP server while a client is attached.
    void setMaintenanceMode(bool active);
    bool isInMaintenanceMode() const { return _maintenanceMode; }

    // Called from the usb_host task the moment the device is unplugged.
    void onDeviceGone(GoneCallback cb) { _goneCb = cb; }

    // ── Transfers (used by UsbIpServer) ──────────────────────────────────────
    // `xfer` is owned by the caller; its callback runs on the usb_host task. Both fail
    // with ESP_ERR_INVALID_STATE when no device is present.
    esp_err_t submitTransfer(usb_transfer_t* xfer);   // bulk / interrupt
    esp_err_t submitControl(usb_transfer_t* xfer);    // control (setup packet in data_buffer)
    void cancelEndpoint(uint8_t epAddr);              // halt + flush: queued transfers complete as CANCELED
    void clearEndpoint(uint8_t epAddr);               // un-halt once every transfer has completed

    // ── State ────────────────────────────────────────────────────────────────
    UsbDeviceState getState() const { return _state; }
    bool isConnected() const { return _state >= DEVICE_CONNECTED; }
    bool isReady()     const { return _state == DEVICE_READY; }

    // ── Identity ─────────────────────────────────────────────────────────────
    String   getManufacturer() const { return _manufacturer; }
    String   getProduct()      const { return _product; }
    String   getSerial()       const { return _serial; }
    uint16_t getVendorId()     const { return _vendorId; }
    uint16_t getProductId()    const { return _productId; }
    uint8_t  getDevAddr()      const { return _devAddr; }
    usb_speed_t getSpeed()     const { return _speed; }

    int  getIfaceCount() const { return _ifaceCount; }
    bool getIfaceDesc(int i, uint8_t out[4]) const;

    int  getEndpointCount() const { return _epCount; }
    const UsbIpEndpoint* findEndpoint(uint8_t address) const;
    bool getEndpointInfo(int i, uint8_t& addr, uint8_t& type, uint16_t& mps) const {
        if (i < 0 || i >= _epCount) return false;
        addr = _eps[i].address; type = _eps[i].type; mps = _eps[i].maxPacketSize & 0x7FF;
        return true;
    }

    // Raw descriptor bytes forwarded to the USB/IP client during import
    const uint8_t* getDevDescRaw() const { return _devDescBuf; }
    size_t         getDevDescLen() const { return _devDescLen; }
    const uint8_t* getCfgDescRaw() const { return _cfgDescBuf; }
    size_t         getCfgDescLen() const { return _cfgDescLen; }

private:
    std::atomic<UsbDeviceState> _state;
    volatile bool _maintenanceMode;

    String   _manufacturer, _product, _serial;
    uint16_t _vendorId, _productId;
    uint8_t  _devAddr;

    TaskHandle_t             _hostTask;
    usb_host_client_handle_t _clientHandle;
    usb_device_handle_t      _deviceHandle;     // guarded by _devMtx
    SemaphoreHandle_t        _devMtx;

    // Device-close state machine (runs on usb_host).
    volatile bool _everSeen = false;   // a NEW_DEV event has arrived since begin()
    volatile bool _gone;           // device unplugged or unusable: no more submits
    volatile bool _closeReq;
    uint32_t      _closeStartMs;
    int           _relIdx;         // next interface to release
    int           _pendingAddr;    // NEW_DEV that arrived while still closing, or -1

    usb_speed_t _speed;

    UsbIpIface    _ifaces[USBIP_MAX_IFACES];
    int           _ifaceCount;
    UsbIpEndpoint _eps[USBIP_MAX_EPS];
    int           _epCount;

    uint8_t _devDescBuf[USBIP_DEV_DESC_SIZE];
    size_t  _devDescLen;
    uint8_t _cfgDescBuf[USBIP_CFG_DESC_SIZE];
    size_t  _cfgDescLen;

    GoneCallback _goneCb;

    static void _hostTaskFn(void* arg);
    static void _clientEventCallback(const usb_host_client_event_msg_t*, void*);
    void _handleClientEvent(const usb_host_client_event_msg_t*);

    bool _openDevice(uint8_t devAddr);
    void _adoptExistingDevice();   // open a device the host enumerated before we could see its NEW_DEV
    bool _enumerateInterfaces();
    bool _tryClose();              // true once the device is fully closed
    void _resetDeviceFields();
    static String _strDesc(const usb_str_desc_t* d);
};
