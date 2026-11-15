#pragma once
#include <Arduino.h>
#include <functional>
#include "usb/usb_host.h"
#include "freertos/semphr.h"
#include "UsbIpConfig.h"
#include "UsbIpProto.h"

class UsbDevice;

using UsbIpConnectCallback    = std::function<void(const char* clientIP)>;
using UsbIpDisconnectCallback = std::function<void(const char* clientIP, uint32_t durationSec)>;

#define USBIP_PORT        3240
#define USBIP_BUSNUM      1
#define USBIP_BUSID       "1-1"
#define USBIP_SYSFS_PATH  "/sys/devices/esp32/usb/1-1"

struct UsbIpSessionRecord {
    bool     valid       = false;
    char     ip[16]      = {};
    uint32_t endedAt     = 0;   // uptime seconds at disconnect
    uint32_t durationSec = 0;   // seconds the client was attached
};

// USB/IP server for one device at a time.
//
// Tasks (all on USBIP_CORE):
//   usbip_acc   accepts TCP connections; extra clients are closed on the spot.
//   usbip_sess  runs the session: handshake, then reads CMD_SUBMIT / CMD_UNLINK and
//               submits transfers. Never waits for a transfer to finish, except an
//               oversized bulk OUT, which is sent in pieces.
//   usbip_tx    drains the response queue to the socket and keeps read-ahead armed.
// Transfer completions arrive on the USB host's usb_host task and only queue a response.
//
// Memory: transfers come from a pool preallocated in begin(). An OUT payload is read
// from the socket straight into the transfer buffer and an IN result is written from
// it, so data is not copied on its way through.
class UsbIpServer {
public:
    // Call after WiFi is up (or at any time; the listener binds to all interfaces).
    // `device` must outlive this object.
    void begin(UsbDevice* device);

    // Refuse new clients and drop the current one / allow clients again.
    void stop();
    void start();

    // True while a USB/IP client has successfully imported the device.
    bool isClientAttached() const { return _attached; }

    void onClientConnect(UsbIpConnectCallback cb)       { _onConnectCb    = cb; }
    void onClientDisconnect(UsbIpDisconnectCallback cb) { _onDisconnectCb = cb; }

    // Force-disconnect the current client. Safe from any task.
    void kickClient();

    // JSON status blob: attached state, current client, session history.
    String statusJson() const;

private:
    // ── URB pool ─────────────────────────────────────────────────────────────
    enum UrbState : uint8_t {
        URB_FREE,
        URB_RESERVED,   // taken by the session task, not yet submitted
        URB_PENDING,    // submitted to the USB host
        URB_RING,       // bulk IN served from the read-ahead ring
        URB_RESUBMIT,   // cancelled by a flush meant for another URB; to be submitted again
        URB_QUEUED      // finished; response waits in the sender queue
    };
    struct Urb {
        usb_transfer_t* xfer     = nullptr;
        UsbIpServer*    owner    = nullptr;
        volatile uint8_t state   = URB_FREE;
        uint32_t        gen      = 0;       // session this URB belongs to
        uint32_t        order    = 0;       // submission order
        uint32_t        seqnum   = 0;
        int32_t         bufLen   = 0;       // bytes the client asked for (IN) / sent (OUT)
        int32_t         got      = 0;       // ring URBs: bytes collected so far
        uint8_t         epAddr   = 0;
        bool            isCtrl   = false;
        bool            isIn     = false;
        bool            unlinked = false;
        bool            chunked  = false;   // piece of an oversized OUT (receiver waits on _chunkSem)
        int16_t         clearEp  = -1;      // CLEAR_HALT: un-halt this endpoint when it succeeds
        int32_t         chunkStatus = 0;
        int32_t         chunkActual = 0;
        volatile bool   chunkDone   = false;
    };
    Urb      _urbs[USBIP_MAX_URBS];
    int      _urbCount  = 0;
    uint32_t _orderSeq  = 0;

    // ── Response queue (single FIFO: wire order == enqueue order) ────────────
    struct Resp {
        usbipproto::UrbHeader hdr;
        Urb*             urb;      // released after the send
        const uint8_t*   data;     // payload inside the URB's transfer buffer
        uint32_t         len;
        bool             unlink;
    };
    Resp _q[USBIP_RESP_QUEUE];
    int  _qHead = 0, _qCount = 0, _qUnlinks = 0;

    // ── Bulk IN read-ahead ───────────────────────────────────────────────────
    struct RxPkt { uint8_t len; uint8_t off; uint8_t data[USBIP_RX_PKT_MAX]; };
    struct RxArm { usb_transfer_t* xfer = nullptr; UsbIpServer* owner = nullptr; int8_t ep = 0; bool inUsb = false; };
    struct RxEp {
        bool      active = false;
        bool      dead   = false;      // a transfer could not be cancelled: slot never reused
        bool      needClear = false;
        uint8_t   addr   = 0;
        uint16_t  mps    = 0;
        RxPkt*    ring   = nullptr;
        int       head = 0, count = 0;
        uint8_t*  carry  = nullptr;
        int       carryLen = 0;
        RxArm     arm[USBIP_RX_ARMS];
        int       armed  = 0;
        uint32_t  retryAt = 0;
    };
    RxEp _rx[USBIP_MAX_RX_EPS];

    // ── State ────────────────────────────────────────────────────────────────
    UsbDevice*        _device   = nullptr;
    SemaphoreHandle_t _mtx      = nullptr;   // URB table, queue, read-ahead, socket fd
    SemaphoreHandle_t _chunkSem = nullptr;
    TaskHandle_t      _accTask = nullptr, _sessTask = nullptr, _txTask = nullptr;
    int               _listenFd = -1;
    volatile int      _fd       = -1;        // current client socket (closed only by the session task)
    char              _curIP[16] = {};
    volatile bool     _sessionActive = false;
    volatile bool     _attached = false;
    volatile bool     _stopping = false;     // session is ending: completions are discarded
    volatile bool     _kickReq  = false;
    volatile bool     _enabled  = true;
    volatile bool     _txBusy   = false;
    uint32_t          _gen      = 0;
    // Rate counters, reported every few seconds while attached
    volatile uint32_t _nSubmit = 0, _nRet = 0, _nUnlink = 0;
    volatile int      _qMax    = 0;

    UsbIpConnectCallback    _onConnectCb;
    UsbIpDisconnectCallback _onDisconnectCb;
    unsigned long _connectedMs = 0;
    unsigned long _attachedMs  = 0;

    static const int MAX_HISTORY = 6;
    UsbIpSessionRecord _history[MAX_HISTORY];
    int                _histNext = 0;

    // ── Tasks ────────────────────────────────────────────────────────────────
    static void _accTaskFn(void* arg);
    static void _sessTaskFn(void* arg);
    static void _txTaskFn(void* arg);
    void _acceptLoop();
    void _replyBusy(int fd, const char* ip);
    void _sessionLoop();
    void _senderLoop();

    // ── Session ──────────────────────────────────────────────────────────────
    void _runSession();
    bool _handshake();
    bool _replyDevList();
    bool _replyImport(const char* busid);
    void _fillDevRecord(usbipproto::DeviceRecord& d) const;
    void _receiveLoop();
    void _teardown();
    void _onAttach();
    void _onDisconnect();
    void _onDeviceGone();
    void _shutdownFd();
    bool _abort() const;

    // ── Socket helpers (session task) ────────────────────────────────────────
    int  _recvAll(void* buf, size_t len, uint32_t timeoutMs);   // 0 ok, <0 closed/abort/timeout
    bool _discard(size_t n);
    bool _sendAll(const void* buf, size_t len, bool more = false);
    void _setupSocket(int fd);

    // ── URB handling (session task) ──────────────────────────────────────────
    bool _handleSubmit(const usbipproto::UrbHeader& h);
    bool _submitControl(uint32_t seq, bool in, int32_t len, const uint8_t setup[8]);
    bool _submitData(uint32_t seq, bool in, uint8_t epn, uint32_t flags, int32_t len);
    bool _chunkedOut(uint32_t seq, uint8_t epAddr, uint32_t flags, int32_t len);
    void _launch(Urb* u, uint32_t seq, uint8_t epAddr, bool ctrl, bool in, int32_t bufLen, int16_t clearEp);
    void _handleUnlink(const usbipproto::UrbHeader& h);
    void _resubmitEndpoint(uint8_t epAddr);
    int  _pendingOn(uint8_t epAddr);
    Urb* _acquire(int need);
    void _freeLocked(Urb* u);
    void _cancelAllUrbs();
    void _respondSimple(uint32_t seq, int32_t status, uint32_t actual);
    void _respondUnlink(uint32_t seq, int32_t status);
    bool _pushLocked(const usbipproto::UrbHeader& h, Urb* u, const uint8_t* data, uint32_t len, bool unlink);
    void _wakeSender() { if (_txTask) xTaskNotifyGive(_txTask); }
    int  _maxData() const;

    static void _urbDone(usb_transfer_t* t);
    void _onUrbDone(Urb* u);
    static int32_t _mapStatus(usb_transfer_status_t s);

    // ── Read-ahead ───────────────────────────────────────────────────────────
    RxEp* _rxFind(uint8_t addr);
    void  _rxStart();
    void  _rxStop();
    void  _rxArmLocked(RxEp& r);
    void  _rxServiceLocked(RxEp& r);
    void  _rxMaintain();
    static void _rxDone(usb_transfer_t* t);
    void  _onRxDone(RxArm& a);
};
