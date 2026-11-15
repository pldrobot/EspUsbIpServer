#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <functional>
#include "usb/usb_host.h"

class UsbDevice;

using UsbIpConnectCallback    = std::function<void(const char* clientIP)>;
using UsbIpDisconnectCallback = std::function<void(const char* clientIP, uint32_t durationSec)>;

#define USBIP_PORT        3240
#define USBIP_VERSION     0x0111
#define OP_REQ_DEVLIST    0x8005
#define OP_REP_DEVLIST    0x0005
#define OP_REQ_IMPORT     0x8003
#define OP_REP_IMPORT     0x0003
#define USBIP_CMD_SUBMIT  0x00000001
#define USBIP_CMD_UNLINK  0x00000002
#define USBIP_RET_SUBMIT  0x00000003
#define USBIP_RET_UNLINK  0x00000004
#define USBIP_DIR_OUT     0
#define USBIP_DIR_IN      1
#define USBIP_SPEED_LOW   1
#define USBIP_SPEED_FULL  2
#define USBIP_SPEED_HIGH  3
#define USBIP_BUSNUM      1
#define USBIP_DEVNUM      2
#define USBIP_BUSID       "1-1"

struct UsbIpSessionRecord {
    bool     valid       = false;
    char     ip[16]      = {};
    uint32_t endedAt     = 0;   // uptime seconds at disconnect
    uint32_t durationSec = 0;   // seconds the client was attached
};

class UsbIpServer {
public:
    // Initialise and start the server task pinned to the caller's core.
    // Call after WiFi is connected. device must outlive this object.
    void begin(UsbDevice* device);

    // Suspend/resume the server task without destroying it.
    void stop();
    void start();

    // True while a USB/IP client has successfully imported the device.
    bool isClientAttached() const { return _attached; }

    // Register callbacks fired when a client attaches or detaches.
    void onClientConnect(UsbIpConnectCallback cb)       { _onConnectCb    = cb; }
    void onClientDisconnect(UsbIpDisconnectCallback cb) { _onDisconnectCb = cb; }

    // Force-disconnect the current client. Unconditional: works even when the
    // peer's socket is already gone. Safe to call from any task (the drop is
    // performed by the server task within ~1 ms).
    void kickClient() { _kickReq = true; }

    // JSON status blob: attached state, current client, session history.
    String statusJson() const;

private:
    WiFiServer   _server;
    WiFiClient   _client;
    UsbDevice*   _device     = nullptr;
    bool         _attached   = false;
    volatile bool _kickReq   = false;
    TaskHandle_t _taskHandle = nullptr;

    UsbIpConnectCallback    _onConnectCb;
    UsbIpDisconnectCallback _onDisconnectCb;

    char          _curIP[16]   = {};
    unsigned long _connectedMs = 0;
    unsigned long _attachedMs  = 0;

    static const int MAX_HISTORY = 6;
    UsbIpSessionRecord _history[MAX_HISTORY];
    int                _histNext = 0;

    // IN URBs (bulk/interrupt reads) are submitted asynchronously and answered
    // when the device completes them, so a read that waits for data (an idle
    // serial port, an interrupt endpoint) never blocks later URBs - such as the
    // OUT write that makes the device produce that data.
    struct PendingUrb {
        bool            used      = false;
        volatile bool   done      = false;
        bool            cancelled = false;
        uint32_t        seqnum    = 0;
        uint32_t        order     = 0;     // submission order (replies must keep it)
        uint8_t         epAddr    = 0;
        int32_t         bufLen    = 0;
        usb_transfer_t* xfer      = nullptr;
        // Read-ahead URBs (bulk IN) are not backed by a USB transfer: they are filled
        // from the endpoint's receive ring in _rxService().
        bool            ring      = false;
        uint8_t*        buf       = nullptr;
        int32_t         got       = 0;
    };
    static const int MAX_PENDING = 32;
    PendingUrb _pending[MAX_PENDING];
    uint32_t   _pendingSeq = 0;

    // Bulk IN read-ahead. The ESP keeps each bulk IN endpoint armed with small
    // single-packet transfers at all times and stores what arrives in a ring, instead of
    // reading only while the client has a URB outstanding. A real host controller polls
    // continuously; reading only on demand (with a network round trip between reads)
    // lets a streaming device overflow its tiny FIFO - a CH340 at 115200 baud loses data
    // in gaps of a few milliseconds. Packet boundaries are kept so client URBs complete
    // exactly as on real hardware: on a full buffer or on a short/zero-length packet.
    static const int RX_PKT_MAX  = 64;      // largest full-speed bulk packet
    static const int RX_RING     = 128;     // packets buffered per endpoint
    static const int RX_ARMS     = 12;      // transfers kept queued on the device
    static const int RX_CARRY    = 4096;    // bytes handed back by an unlinked, part-filled URB
    static const int RX_URB_MAX  = 4096;    // largest read buffer per client URB
    static const int MAX_RX_EPS  = 2;
    struct RxPkt { uint8_t len; uint8_t off; uint8_t data[RX_PKT_MAX]; };
    struct RxArm { usb_transfer_t* xfer = nullptr; volatile bool done = false; };
    struct RxEp {
        bool      active  = false;
        uint8_t   addr    = 0;
        uint16_t  mps     = 0;
        RxPkt*    ring    = nullptr;
        int       head    = 0, count = 0;
        uint8_t*  carry   = nullptr;
        int       carryLen = 0;
        RxArm     arm[RX_ARMS];
        int       armed   = 0;
        unsigned long retryAt = 0;
    };
    RxEp _rx[MAX_RX_EPS];

    static void _rxDone(usb_transfer_t* t);
    RxEp* _rxFind(uint8_t addr);
    void  _rxStart();
    void  _rxStop();
    void  _rxArm(RxEp& r);
    void  _rxPump();
    void  _rxService();

    static void _urbDone(usb_transfer_t* t);
    void _drainPending();
    void _cancelAllPending();
    void _handleUnlink(uint32_t seqnum, uint32_t victim);

    static void _serverTask(void* arg);
    void        _poll();

    // Session teardown that does not depend on connected(): records history,
    // resets state and closes the socket. Safe to call with no session.
    void _dropSession();
    // True when the peer closed (FIN/RST) or the socket is unusable.
    bool _peerGone();
    bool _sessionAlive();
    void _enableKeepAlive();

    void _onConnect(const String& ip);
    void _onAttach();
    void _onDisconnect();

    bool _handleDevList();
    bool _handleImport();
    void _handleUrbs();
    bool _processSubmit(uint32_t seqnum, uint32_t direction, uint32_t ep,
                        const uint8_t setup[8], int32_t bufLen);

    bool _readExact(uint8_t* buf, size_t len, uint32_t timeoutMs = 5000);
    bool _writeAll(const uint8_t* buf, size_t len);
    void _fillDevInfo(uint8_t out[312]);

    static void     _w32(uint8_t* p, uint32_t v);
    static void     _w16(uint8_t* p, uint16_t v);
    static uint32_t _r32(const uint8_t* p);
    static uint16_t _r16(const uint8_t* p);
};
