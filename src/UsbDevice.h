#pragma once
#include <Arduino.h>
#include "usb/usb_host.h"

#define USB_CLASS_PRINTER       0x07
#define USB_SUBCLASS_PRINTER    0x01
#define USB_PROTOCOL_UNIDIR     0x01
#define USB_PROTOCOL_BIDIR      0x02
#define USB_PROTOCOL_1284       0x03

#define USB_PRINTER_GET_DEVICE_ID   0x00
#define USB_PRINTER_GET_PORT_STATUS 0x01
#define USB_PRINTER_SOFT_RESET      0x02

// USB standard request codes (intercepted by USB/IP forwarding layer)
#define USB_REQ_SET_ADDRESS       0x05
#define USB_REQ_SET_CONFIGURATION 0x09
#define USB_REQ_SET_INTERFACE     0x0B

enum UsbDeviceState {
    DEVICE_DISCONNECTED,
    DEVICE_CONNECTED,
    DEVICE_READY,
    DEVICE_BUSY,
    DEVICE_MAINTENANCE,   // Locked for USB/IP exclusive access
    DEVICE_ERROR
};

#define DEV_DESC_BUF_SIZE   18
#define CFG_DESC_BUF_SIZE   512

// Generic enumeration tables — this library claims and forwards *any* bulk or
// interrupt endpoint on *any* interface (alternate setting 0 only), not just
// a USB-printer interface. Isochronous endpoints and non-default alternate
// settings are out of scope (see README).
#define USBIP_MAX_IFACES  8
#define USBIP_MAX_EPS     16

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
    uint8_t  interval;       // bInterval (informational, interrupt endpoints)
};

class UsbDevice {
public:
    UsbDevice();

    // Install USB Host driver and spawn the event task on the opposite core.
    bool begin();

    // Suspend / resume the USB Host event task.
    void stop();
    void start();

    // Normal data path — rejected while USB/IP holds the maintenance lock.
    // Uses the device's first bulk OUT/IN endpoint pair (set during
    // enumeration); for multi-endpoint devices, use submitTransfer() instead.
    int sendData(const uint8_t* data, size_t length);
    int readData(uint8_t* buffer, size_t maxLength, uint32_t timeoutMs = 2000);

    // Maintenance/diagnostic path
    int  sendMaintCommand(const uint8_t* cmd, size_t cmdLen,
                          uint8_t* response = nullptr,
                          size_t   responseMaxLen = 0,
                          uint32_t responseTimeoutMs = 2000);
    // USB-printer SOFT_RESET — no-op (returns false) unless a printer-class
    // interface (0x07/0x01) is actually present among the claimed interfaces.
    bool softReset();

    // Exclusive lock held by the USB/IP server while a client is attached
    void setMaintenanceMode(bool active);
    bool isInMaintenanceMode() const { return _maintenanceMode; }

    // Generic transfers used by UsbIpServer to forward URBs
    int submitControlTransfer(usb_setup_packet_t* setup,
                              uint8_t* data, size_t dataLen,
                              uint32_t timeoutMs = 3000);
    int submitBulkOut(const uint8_t* data, size_t len, uint32_t timeoutMs = 5000);
    int submitBulkIn(uint8_t*  buf,  size_t maxLen,    uint32_t timeoutMs = 3000);
    // Generic bulk/interrupt transfer to any claimed endpoint, addressed by
    // its real bEndpointAddress (direction bit included) — this is what lets
    // UsbIpServer forward a URB to the correct pipe on a multi-endpoint
    // device instead of assuming a single remembered pair.
    int submitTransfer(uint8_t epAddr, uint8_t* buf, size_t len, uint32_t timeoutMs = 5000);

    // State
    UsbDeviceState getState()   const { return _state; }
    bool isConnected() const { return _state >= DEVICE_CONNECTED; }
    bool isReady()     const { return _state == DEVICE_READY; }
    bool isBusy()      const { return _state == DEVICE_BUSY; }

    // Identity
    String   getManufacturer() const { return _manufacturer; }
    String   getProduct()      const { return _product; }
    String   getDeviceId()     const { return _deviceId; }
    uint16_t getVendorId()     const { return _vendorId; }
    uint16_t getProductId()    const { return _productId; }
    uint8_t  getEpOut()        const { return _epOut; }
    uint8_t  getEpIn()         const { return _epIn; }
    uint8_t  getIfaceNum()     const { return _ifaceCount ? _ifaces[0].ifaceNum : 0; }
    usb_speed_t getSpeed()     const { return _speed; }

    // Every interface actually claimed on the current device (not just one) —
    // used by UsbIpServer to report real descriptors instead of a fixed tuple.
    int  getIfaceCount() const { return _ifaceCount; }
    bool getIfaceDesc(int i, uint8_t out[4]) const;

    // Raw descriptor bytes forwarded to the USB/IP client during import
    const uint8_t* getDevDescRaw() const { return _devDescBuf; }
    size_t         getDevDescLen() const { return _devDescLen; }
    const uint8_t* getCfgDescRaw() const { return _cfgDescBuf; }
    size_t         getCfgDescLen() const { return _cfgDescLen; }

private:
    UsbDeviceState _state;
    bool _maintenanceMode;

    String   _manufacturer, _product, _deviceId;
    uint16_t _vendorId, _productId;

    TaskHandle_t             _taskHandle;
    usb_host_client_handle_t _clientHandle;
    usb_device_handle_t      _deviceHandle;

    // "Primary" bulk pair — the first bulk IN/OUT endpoints found during
    // enumeration — kept for sendData()/readData()/sendMaintCommand(), which
    // predate (and don't need) the full generic endpoint table below.
    uint8_t  _epOut, _epIn;
    uint16_t _epOutMPS, _epInMPS;

    usb_speed_t _speed;

    UsbIpIface    _ifaces[USBIP_MAX_IFACES];
    int           _ifaceCount;
    UsbIpEndpoint _eps[USBIP_MAX_EPS];
    int           _epCount;

    uint8_t _devDescBuf[DEV_DESC_BUF_SIZE];
    size_t  _devDescLen;
    uint8_t _cfgDescBuf[CFG_DESC_BUF_SIZE];
    size_t  _cfgDescLen;

    static void _deviceTask(void* arg);
    void        _poll();

    static void _clientEventCallback(const usb_host_client_event_msg_t*, void*);
    void _handleClientEvent(const usb_host_client_event_msg_t*);
    bool _openDevice(uint8_t devAddr);
    bool _enumerateInterfaces();
    void _closeDevice();
    bool _getDeviceId();

    const UsbIpEndpoint* _findEndpoint(uint8_t address) const;
    int  _printerIfaceNum() const;

    int _bulkTransfer(uint8_t endpoint, uint8_t* buffer, size_t length, uint32_t timeoutMs);
};
