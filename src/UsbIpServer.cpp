#include "UsbIpServer.h"
#include "UsbDevice.h"
#include "EspUsbIpLog.h"
#include <esp_heap_caps.h>
#include <errno.h>
#include <lwip/sockets.h>
#include <lwip/tcp.h>

using namespace usbipproto;

#ifndef MSG_MORE
#define MSG_MORE 0
#endif

#define LOCK()   xSemaphoreTake(_mtx, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(_mtx)

static inline bool timeReached(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

static bool recvFd(int fd, void* buf, size_t len) {
    uint8_t* b = (uint8_t*)buf;
    size_t got = 0;
    while (got < len) {
        int n = recv(fd, b + got, len - got, 0);
        if (n <= 0) return false;                  // closed, error, or SO_RCVTIMEO expired
        got += n;
    }
    return true;
}

static bool sendFd(int fd, const void* buf, size_t len, bool more) {
    const uint8_t* b = (const uint8_t*)buf;
    size_t sent = 0;
    while (sent < len) {
        int n = send(fd, b + sent, len - sent, more ? MSG_MORE : 0);
        if (n > 0) { sent += n; continue; }
        if (n < 0 && errno == EINTR) continue;
        return false;                      // error, or SO_SNDTIMEO expired
    }
    return true;
}

// ── Public ───────────────────────────────────────────────────────────────────

void UsbIpServer::begin(UsbDevice* device) {
    if (_txTask) return;
    _device   = device;
    _mtx      = xSemaphoreCreateMutex();
    _chunkSem = xSemaphoreCreateCounting(4, 0);

    // Transfer pool, smallest slots first (so a URB takes the smallest that fits).
    struct Pool { uint16_t size; uint8_t count; };
    const Pool pools[] = { { USBIP_POOL0_SIZE, USBIP_POOL0_COUNT },
                           { USBIP_POOL1_SIZE, USBIP_POOL1_COUNT },
                           { USBIP_POOL2_SIZE, USBIP_POOL2_COUNT } };
    for (const Pool& p : pools) {
        for (int i = 0; i < p.count && _urbCount < USBIP_MAX_URBS; i++) {
            usb_transfer_t* x = nullptr;
            if (usb_host_transfer_alloc(p.size, 0, &x) != ESP_OK || !x) {
                _USBIP_LOGW("USB transfer pool: only %d of %d x %u B slots (out of DMA memory)",
                            i, p.count, p.size);
                break;
            }
            Urb& u = _urbs[_urbCount++];
            u.xfer = x; u.owner = this; u.state = URB_FREE;
        }
    }
    _USBIP_LOGI("USB transfer pool: %d slots, largest %d B (free DMA heap now %u B)", _urbCount,
                _maxData(), (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));

    _device->onDeviceGone([this]() { _onDeviceGone(); });

    _listenFd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (_listenFd < 0) { _USBIP_LOGE("socket() failed"); return; }
    int on = 1;
    setsockopt(_listenFd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    struct sockaddr_in a = {};
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port        = htons(USBIP_PORT);
    if (bind(_listenFd, (struct sockaddr*)&a, sizeof(a)) != 0 || listen(_listenFd, 2) != 0) {
        _USBIP_LOGE("bind/listen on port %d failed (errno %d)", USBIP_PORT, errno);
        close(_listenFd); _listenFd = -1;
        return;
    }

    xTaskCreatePinnedToCore(_txTaskFn,   "usbip_tx",   USBIP_STACK_SENDER,  this, USBIP_PRIO_NET, &_txTask,   USBIP_CORE);
    xTaskCreatePinnedToCore(_sessTaskFn, "usbip_sess", USBIP_STACK_SESSION, this, USBIP_PRIO_NET, &_sessTask, USBIP_CORE);
    xTaskCreatePinnedToCore(_accTaskFn,  "usbip_acc",  USBIP_STACK_ACCEPT,  this, USBIP_PRIO_NET, &_accTask,  USBIP_CORE);
    _USBIP_LOGI("Server on port %d (core %d) - one client at a time", USBIP_PORT, USBIP_CORE);
}

void UsbIpServer::stop() {
    _enabled = false;
    kickClient();
    for (int i = 0; i < 300 && _sessionActive; i++) vTaskDelay(pdMS_TO_TICKS(10));
    _USBIP_LOGI("Server stopped");
}

void UsbIpServer::start() {
    _enabled = true;
    _USBIP_LOGI("Server started");
}

void UsbIpServer::kickClient() {
    if (!_sessionActive) return;
    _kickReq = true;
    _shutdownFd();
}

// ── Tasks ────────────────────────────────────────────────────────────────────

void UsbIpServer::_accTaskFn(void* arg)  { static_cast<UsbIpServer*>(arg)->_acceptLoop(); }
void UsbIpServer::_sessTaskFn(void* arg) { static_cast<UsbIpServer*>(arg)->_sessionLoop(); }
void UsbIpServer::_txTaskFn(void* arg)   { static_cast<UsbIpServer*>(arg)->_senderLoop(); }

void UsbIpServer::_acceptLoop() {
    for (;;) {
        struct sockaddr_in ra = {};
        socklen_t rl = sizeof(ra);
        int fd = accept(_listenFd, (struct sockaddr*)&ra, &rl);
        if (fd < 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

        char ip[16];
        uint32_t ipv = ntohl(ra.sin_addr.s_addr);
        snprintf(ip, sizeof(ip), "%u.%u.%u.%u",
                 (unsigned)(ipv >> 24), (unsigned)((ipv >> 16) & 255),
                 (unsigned)((ipv >> 8) & 255), (unsigned)(ipv & 255));

        // A new connection must not be refused just because an earlier one has not attached the
        // device yet (a device-list request still being torn down, or one that never imports).
        // Drop that idle session and take over; only an attached device makes us busy.
        LOCK();
        const bool preempt = _sessionActive && !_attached && _enabled;
        UNLOCK();
        if (preempt) {
            _USBIP_LOGD("Dropping unattached session from %s for a new connection", _curIP);
            _kickReq = true;
            _shutdownFd();
            for (int i = 0; i < 200 && _sessionActive; i++) vTaskDelay(pdMS_TO_TICKS(5));   // up to 1 s
        }

        LOCK();
        bool busy = _sessionActive || !_enabled;
        if (!busy) {
            _sessionActive = true;
            _fd = fd;
            strncpy(_curIP, ip, sizeof(_curIP) - 1);
            _curIP[sizeof(_curIP) - 1] = 0;
        }
        UNLOCK();

        if (busy) {
            if (_enabled) _replyBusy(fd, ip);       // answer, don't just hang up
            close(fd);
            continue;
        }
        xTaskNotifyGive(_sessTask);
    }
}

// A second connection while the device is held. Like usbipdcpp (and the kernel's usbipd),
// answer instead of hanging up: DEVLIST lists no device (it is in use), IMPORT gets status
// NA ("not available"). Short timeouts: this runs on the accept task.
void UsbIpServer::_replyBusy(int fd, const char* ip) {
    struct timeval tv = { 2, 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    OpHeader op;
    if (!recvFd(fd, &op, sizeof(op))) return;
    const uint16_t code = be16(op.code);

    if (code == OP_REQ_DEVLIST) {
        struct __attribute__((packed)) { OpHeader op; uint32_t count; } rep;
        memset(&rep, 0, sizeof(rep));
        rep.op.version = be16(VERSION);
        rep.op.code    = be16(OP_REP_DEVLIST);
        rep.op.status  = be32(OP_STATUS_OK);
        rep.count      = be32(0);
        sendFd(fd, &rep, sizeof(rep), false);
    } else if (code == OP_REQ_IMPORT) {
        char busid[32];
        if (!recvFd(fd, busid, sizeof(busid))) return;
        OpHeader rep;
        memset(&rep, 0, sizeof(rep));
        rep.version = be16(VERSION);
        rep.code    = be16(OP_REP_IMPORT);
        rep.status  = be32(OP_STATUS_NA);
        sendFd(fd, &rep, sizeof(rep), false);
        _USBIP_LOGW("IMPORT from %s refused: device held by %s", ip, _curIP);
    }
}

void UsbIpServer::_sessionLoop() {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        _runSession();
    }
}

// Drains the response queue to the socket, and keeps read-ahead armed.
void UsbIpServer::_senderLoop() {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        _rxMaintain();

#if USBIP_STATS
        // While a client is attached, report the URB rate every 5 s (only when something moved).
        static uint32_t lastStat = 0, pSub = 0, pRet = 0, pUnl = 0;
        if (_attached && millis() - lastStat >= 5000) {
            lastStat = millis();
            const uint32_t s = _nSubmit, r = _nRet, u = _nUnlink;
            if (s != pSub || r != pRet || u != pUnl) {
                _USBIP_LOGI("USB/IP rate: %u submit, %u ret, %u unlink in 5 s; response queue max %d; free heap %u",
                            (unsigned)(s - pSub), (unsigned)(r - pRet), (unsigned)(u - pUnl), _qMax,
                            (unsigned)ESP.getFreeHeap());
                pSub = s; pRet = r; pUnl = u; _qMax = 0;
            }
        }
#endif

        for (;;) {
            Resp r;
            int  fd;
            LOCK();
            if (_qCount == 0 || _stopping) { UNLOCK(); break; }   // teardown owns the queue once stopping
            r = _q[_qHead];
            _qHead = (_qHead + 1) % USBIP_RESP_QUEUE;
            _qCount--;
            if (r.unlink) _qUnlinks--;
            fd = _fd;
            _txBusy = true;
            UNLOCK();

            bool ok = (fd >= 0) && sendFd(fd, &r.hdr, sizeof(r.hdr), r.len > 0);
            if (ok && r.len > 0) ok = sendFd(fd, r.data, r.len, false);

            LOCK();
            if (r.urb) _freeLocked(r.urb);
            _txBusy = false;
            UNLOCK();

            if (ok) _nRet = _nRet + 1;
            if (!ok) { _shutdownFd(); break; }   // the session task sees the closed socket and ends
        }
    }
}

// ── Session ──────────────────────────────────────────────────────────────────

void UsbIpServer::_setupSocket(int fd) {
    int on = 1, idle = 20, intvl = 5, cnt = 3;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
    // Detect vanished peers (power loss, network drop) that never send FIN/RST: ~35 s.
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
    // recv wakes regularly so the session can notice kicks, unplugs and shutdowns.
    struct timeval rt = { USBIP_SOCK_TICK_MS / 1000, (USBIP_SOCK_TICK_MS % 1000) * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rt, sizeof(rt));
    struct timeval st = { USBIP_SEND_TIMEOUT_MS / 1000, (USBIP_SEND_TIMEOUT_MS % 1000) * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &st, sizeof(st));
}

void UsbIpServer::_runSession() {
    _setupSocket(_fd);
    LOCK();
    _gen++;
    _stopping    = false;
    _kickReq     = false;
    _attached    = false;
    _connectedMs = millis();
    _attachedMs  = 0;
    UNLOCK();
    _USBIP_LOGD("Connection from %s", _curIP);

    if (_handshake()) {
        _onAttach();
        _receiveLoop();
    }
    _teardown();
}

bool UsbIpServer::_abort() const {
    return _stopping || _kickReq || !_enabled || (_attached && !_device->isConnected());
}

void UsbIpServer::_shutdownFd() {
    LOCK();
    int fd = _fd;
    if (fd >= 0) shutdown(fd, SHUT_RDWR);
    UNLOCK();
}

void UsbIpServer::_onDeviceGone() {
    _shutdownFd();       // runs on usb_host: wakes the session so it can tear down
}

void UsbIpServer::_onAttach() {
    _attachedMs = millis();
    _attached   = true;
    _device->setMaintenanceMode(true);
    _rxStart();
    _USBIP_LOGI("Attached: %s", _curIP);
    if (_onConnectCb) _onConnectCb(_curIP);
}

void UsbIpServer::_onDisconnect() {
    if (!_curIP[0]) return;

    // Only a session that attached the device is a session worth reporting. A connection that
    // just asked for the device list (every scan of a client does) must not add a history entry
    // or fire the disconnect callback: applications typically save state in it.
    if (_attached) {
        UsbIpSessionRecord& r = _history[_histNext];
        r.valid       = true;
        strncpy(r.ip, _curIP, sizeof(r.ip) - 1);
        r.endedAt     = (uint32_t)(millis() / 1000);
        r.durationSec = _attachedMs ? (uint32_t)((millis() - _attachedMs) / 1000) : 0;
        _histNext = (_histNext + 1) % MAX_HISTORY;

        _USBIP_LOGI("Disconnected: %s (attached %us)", _curIP, r.durationSec);
        _device->setMaintenanceMode(false);
        _attached = false;
        if (_onDisconnectCb) _onDisconnectCb(r.ip, r.durationSec);
    } else {
        _USBIP_LOGD("Connection from %s closed without attaching", _curIP);
    }
    memset(_curIP, 0, sizeof(_curIP));
    _connectedMs = 0;
    _attachedMs  = 0;
}

// Ends a session in the order that keeps every buffer valid: stop accepting
// completions, wake everything blocked on the socket, cancel what the USB host still
// owns and wait for it, and only then recycle the pool and close the socket.
void UsbIpServer::_teardown() {
    LOCK(); _stopping = true; UNLOCK();
    _shutdownFd();
    _wakeSender();
    _cancelAllUrbs();
    _rxStop();
    for (int i = 0; i < 500 && _txBusy; i++) vTaskDelay(pdMS_TO_TICKS(1));   // sender gives up once the socket is shut down

    LOCK();
    while (_qCount > 0) {                            // responses nobody will read any more
        Resp& r = _q[_qHead];
        if (r.urb) _freeLocked(r.urb);
        _qHead = (_qHead + 1) % USBIP_RESP_QUEUE;
        _qCount--;
    }
    _qUnlinks = 0;
    for (int i = 0; i < _urbCount; i++)
        if (_urbs[i].state != URB_PENDING) _freeLocked(&_urbs[i]);   // PENDING ones were never confirmed done: their callback frees them
    int fd = _fd;
    _fd = -1;
    UNLOCK();
    if (fd >= 0) close(fd);

    _onDisconnect();
    LOCK(); _sessionActive = false; UNLOCK();
}

// ── Socket helpers (session task) ────────────────────────────────────────────

// Reads exactly `len` bytes. `idleMs` bounds the wait for the first byte (0 = forever);
// once data has started to arrive the rest must follow within USBIP_PAYLOAD_TIMEOUT_MS.
// Returns 0 on success, negative when the peer closed, the session is aborting, or a
// timeout expired.
int UsbIpServer::_recvAll(void* buf, size_t len, uint32_t idleMs) {
    uint8_t* b = (uint8_t*)buf;
    size_t got = 0;
    uint32_t last = millis();
    while (got < len) {
        if (_abort()) return -2;
        int n = recv(_fd, b + got, len - got, 0);
        if (n > 0) { got += n; last = millis(); continue; }
        if (n == 0) return -1;                                  // orderly shutdown
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            uint32_t limit = got ? USBIP_PAYLOAD_TIMEOUT_MS : idleMs;
            if (limit && millis() - last > limit) return -3;
            continue;
        }
        return -1;
    }
    return 0;
}

bool UsbIpServer::_discard(size_t n) {
    uint8_t tmp[128];
    while (n > 0) {
        size_t c = n < sizeof(tmp) ? n : sizeof(tmp);
        if (_recvAll(tmp, c, USBIP_PAYLOAD_TIMEOUT_MS) != 0) return false;
        n -= c;
    }
    return true;
}

bool UsbIpServer::_sendAll(const void* buf, size_t len, bool more) {
    return sendFd(_fd, buf, len, more);
}

// ── Handshake ────────────────────────────────────────────────────────────────

bool UsbIpServer::_handshake() {
    for (;;) {
        OpHeader op;
        if (_recvAll(&op, sizeof(op), USBIP_HANDSHAKE_TIMEOUT_MS) != 0) return false;
        uint16_t ver  = be16(op.version);
        uint16_t code = be16(op.code);
        if (ver != VERSION && ver != 0) {
            _USBIP_LOGW("Unsupported USB/IP version 0x%04X", ver);
            return false;
        }
        if (code == OP_REQ_DEVLIST) {
            if (!_replyDevList()) return false;
            continue;                                   // the client normally closes next
        }
        if (code == OP_REQ_IMPORT) {
            char busid[32];
            if (_recvAll(busid, sizeof(busid), USBIP_HANDSHAKE_TIMEOUT_MS) != 0) return false;
            busid[sizeof(busid) - 1] = 0;               // the field need not be NUL-terminated
            return _replyImport(busid);
        }
        _USBIP_LOGW("Unexpected handshake opcode 0x%04X", code);
        return false;
    }
}

void UsbIpServer::_fillDevRecord(DeviceRecord& d) const {
    memset(&d, 0, sizeof(d));
    strncpy(d.path,  USBIP_SYSFS_PATH, sizeof(d.path)  - 1);
    strncpy(d.busid, USBIP_BUSID,      sizeof(d.busid) - 1);
    d.busnum = be32(USBIP_BUSNUM);
    d.devnum = be32(_device->getDevAddr() ? _device->getDevAddr() : 1);
    // The ESP32-S3's USB-OTG has no High-Speed PHY: report the negotiated Low or Full speed.
    d.speed     = be32(_device->getSpeed() == USB_SPEED_LOW ? SPEED_LOW : SPEED_FULL);
    d.idVendor  = be16(_device->getVendorId());
    d.idProduct = be16(_device->getProductId());

    // bcdDevice, class triple and bNumConfigurations come from the real device
    // descriptor, bConfigurationValue from the active configuration descriptor.
    const uint8_t* dd = _device->getDevDescRaw();
    const uint8_t* cd = _device->getCfgDescRaw();
    const bool haveDev = _device->getDevDescLen() >= 18;
    const bool haveCfg = _device->getCfgDescLen() >= 6;
    d.bcdDevice           = be16(haveDev ? (uint16_t)(dd[12] | (dd[13] << 8)) : 0x0100);
    d.bDeviceClass        = haveDev ? dd[4]  : 0;
    d.bDeviceSubClass     = haveDev ? dd[5]  : 0;
    d.bDeviceProtocol     = haveDev ? dd[6]  : 0;
    d.bConfigurationValue = haveCfg ? cd[5]  : 1;
    d.bNumConfigurations  = haveDev ? dd[17] : 1;
    d.bNumInterfaces      = (uint8_t)_device->getIfaceCount();
}

bool UsbIpServer::_replyDevList() {
    struct __attribute__((packed)) Reply {
        OpHeader        op;
        uint32_t        count;
        DeviceRecord    dev;
        InterfaceRecord ifaces[USBIP_MAX_IFACES];
    } rep;

    const bool has = _device->isConnected();
    memset(&rep, 0, sizeof(rep));
    rep.op.version = be16(VERSION);
    rep.op.code    = be16(OP_REP_DEVLIST);
    rep.op.status  = be32(OP_STATUS_OK);
    rep.count      = be32(has ? 1 : 0);

    size_t len = offsetof(Reply, dev);
    if (has) {
        _fillDevRecord(rep.dev);
        len = offsetof(Reply, ifaces);
        int n = _device->getIfaceCount();
        for (int i = 0; i < n && i < USBIP_MAX_IFACES; i++) {
            uint8_t d[4];
            if (!_device->getIfaceDesc(i, d)) break;
            rep.ifaces[i].bInterfaceClass    = d[0];
            rep.ifaces[i].bInterfaceSubClass = d[1];
            rep.ifaces[i].bInterfaceProtocol = d[2];
            len += sizeof(InterfaceRecord);
        }
    }
    return _sendAll(&rep, len);
}

bool UsbIpServer::_replyImport(const char* busid) {
    struct __attribute__((packed)) Reply { OpHeader op; DeviceRecord dev; } rep;
    memset(&rep, 0, sizeof(rep));
    rep.op.version = be16(VERSION);
    rep.op.code    = be16(OP_REP_IMPORT);

    if (!_device->isConnected() || strcmp(busid, USBIP_BUSID) != 0) {
        rep.op.status = be32(OP_STATUS_NO_DEV);
        _sendAll(&rep.op, sizeof(rep.op));
        _USBIP_LOGW("IMPORT rejected: %s", _device->isConnected() ? "unknown busid" : "device not connected");
        return false;
    }
    rep.op.status = be32(OP_STATUS_OK);
    _fillDevRecord(rep.dev);
    return _sendAll(&rep, sizeof(rep));
}

// ── Receive loop ─────────────────────────────────────────────────────────────

void UsbIpServer::_receiveLoop() {
    for (;;) {
        UrbHeader h;
        if (_recvAll(&h, sizeof(h), 0) != 0) break;       // wait as long as the client likes
        const uint32_t cmd = be32(h.command);
        if (cmd == CMD_SUBMIT) {
            if (!_handleSubmit(h)) break;
        } else if (cmd == CMD_UNLINK) {
            _handleUnlink(h);
        } else {
            // The stream cannot be resynchronised after an unknown message.
            _USBIP_LOGW("Unknown command 0x%08X - dropping session", (unsigned)cmd);
            break;
        }
    }
}

// ── URB pool ─────────────────────────────────────────────────────────────────

int UsbIpServer::_maxData() const {
    int m = 0;
    for (int i = 0; i < _urbCount; i++)
        if ((int)_urbs[i].xfer->data_buffer_size > m) m = (int)_urbs[i].xfer->data_buffer_size;
    return m;
}

UsbIpServer::Urb* UsbIpServer::_acquire(int need) {
    const uint32_t start = millis();
    for (;;) {
        LOCK();
        for (int i = 0; i < _urbCount; i++) {            // pools ascend by size: first fit is the smallest
            Urb& u = _urbs[i];
            if (u.state == URB_FREE && (int)u.xfer->data_buffer_size >= need) {
                u.state = URB_RESERVED;
                u.gen   = _gen;
                u.order = ++_orderSeq;
                UNLOCK();
                return &u;
            }
        }
        UNLOCK();
        if (_abort() || millis() - start > USBIP_SLOT_WAIT_MS) return nullptr;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void UsbIpServer::_freeLocked(Urb* u) {
    u->state    = URB_FREE;
    u->unlinked = false;
    u->chunked  = false;
    u->isCtrl   = false;
    u->clearEp  = -1;
    u->got      = 0;
}

int32_t UsbIpServer::_mapStatus(usb_transfer_status_t s) {
    switch (s) {
        case USB_TRANSFER_STATUS_COMPLETED: return ST_OK;
        case USB_TRANSFER_STATUS_CANCELED:  return ST_ECONNRESET;
        case USB_TRANSFER_STATUS_NO_DEVICE: return ST_ENODEV;
        case USB_TRANSFER_STATUS_TIMED_OUT: return ST_ETIMEDOUT;
        case USB_TRANSFER_STATUS_OVERFLOW:  return -75;           // EOVERFLOW
        default:                            return ST_EPIPE;      // stall / error
    }
}

// ── Response queue ───────────────────────────────────────────────────────────

// `unlink` marks entries from the small reserved share (RET_UNLINK and locally
// answered requests). URB responses always fit: there is room for one per pool slot.
bool UsbIpServer::_pushLocked(const UrbHeader& h, Urb* u, const uint8_t* data, uint32_t len, bool unlink) {
    if (unlink && _qUnlinks >= USBIP_UNLINK_SLOTS) return false;
    if (_qCount >= USBIP_RESP_QUEUE) return false;
    Resp& r = _q[(_qHead + _qCount) % USBIP_RESP_QUEUE];
    r.hdr = h; r.urb = u; r.data = data; r.len = len; r.unlink = unlink;
    _qCount++;
    if (_qCount > _qMax) _qMax = _qCount;
    if (unlink) _qUnlinks++;
    return true;
}

void UsbIpServer::_respondSimple(uint32_t seq, int32_t status, uint32_t actual) {
    UrbHeader h;
    makeRetSubmit(h, seq, status, actual);
    for (;;) {
        LOCK();
        bool ok = _pushLocked(h, nullptr, nullptr, 0, true);
        UNLOCK();
        if (ok) { _wakeSender(); return; }
        if (_abort()) return;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void UsbIpServer::_respondUnlink(uint32_t seq, int32_t status) {
    UrbHeader h;
    makeRetUnlink(h, seq, status);
    for (;;) {
        LOCK();
        bool ok = _pushLocked(h, nullptr, nullptr, 0, true);
        UNLOCK();
        if (ok) { _wakeSender(); return; }
        if (_abort()) return;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ── CMD_SUBMIT ───────────────────────────────────────────────────────────────

// Returns false when the stream is broken and the session must end.
bool UsbIpServer::_handleSubmit(const UrbHeader& h) {
    _nSubmit = _nSubmit + 1;
    const uint32_t seq   = be32(h.seqnum);
    const bool     in    = be32(h.direction) == DIR_IN;
    const uint32_t epn   = be32(h.ep);
    const uint32_t flags = be32(h.u.submit.transfer_flags);
    const int32_t  len   = (int32_t)be32(h.u.submit.transfer_buffer_length);
    const uint32_t npk   = be32(h.u.submit.number_of_packets);
    uint8_t setup[8];
    memcpy(setup, h.u.submit.setup, sizeof(setup));

    if (epn > 15 || len < 0 || len > USBIP_MAX_URB_LEN) {
        _USBIP_LOGW("Bad URB (ep %u, len %d) - dropping session", (unsigned)epn, (int)len);
        return false;
    }

    // Isochronous is not supported: swallow the payload and descriptors, fail the URB.
    if (npk != 0 && npk != NOT_ISO) {
        if (npk > MAX_ISO_PACKETS) return false;
        if (!_discard((in ? 0 : (size_t)len) + (size_t)npk * ISO_DESC_SIZE)) return false;
        _respondSimple(seq, ST_EPIPE, 0);
        return true;
    }

    if (epn == 0) return _submitControl(seq, in, len, setup);
    return _submitData(seq, in, (uint8_t)epn, flags, len);
}

// Fills the fields every submitted URB needs, then hands it to the USB host.
// A refused submit is answered right away.
void UsbIpServer::_launch(Urb* u, uint32_t seq, uint8_t epAddr, bool ctrl, bool in,
                          int32_t bufLen, int16_t clearEp) {
    usb_transfer_t* x = u->xfer;
    x->bEndpointAddress = epAddr;
    x->timeout_ms       = 0;
    x->callback         = _urbDone;
    x->context          = u;

    LOCK();
    u->seqnum   = seq;
    u->epAddr   = epAddr;
    u->isCtrl   = ctrl;
    u->isIn     = in;
    u->bufLen   = bufLen;
    u->unlinked = false;
    u->chunked  = false;
    u->clearEp  = clearEp;
    u->state    = URB_PENDING;       // set before submit: the completion may run at once
    esp_err_t err = ctrl ? _device->submitControl(x) : _device->submitTransfer(x);
    if (err != ESP_OK) _freeLocked(u);
    UNLOCK();

    if (err != ESP_OK)
        _respondSimple(seq, err == ESP_ERR_INVALID_STATE ? ST_ENODEV : ST_EPIPE, 0);
}

bool UsbIpServer::_submitControl(uint32_t seq, bool in, int32_t len, const uint8_t setup[8]) {
    const uint8_t  rt     = setup[0], rq = setup[1];
    const uint16_t wValue = setup[2] | (setup[3] << 8);
    const uint16_t wIndex = setup[4] | (setup[5] << 8);
    const uint16_t wLen   = setup[6] | (setup[7] << 8);
    const size_t   outBytes = in ? 0 : (size_t)len;

    // Requests the ESP answers itself: the device is already configured, only
    // alternate setting 0 is exported, and a port reset must not drop the device.
    bool local = false; int32_t lst = ST_OK;
    if (rt == 0x00 && rq == USB_REQ_SET_CONFIGURATION)        { local = true; }
    else if (rt == 0x01 && rq == USB_REQ_SET_INTERFACE)       { local = true; lst = (wValue == 0) ? ST_OK : ST_EPIPE; }
    else if (rt == 0x23 && rq == 0x03 && wValue == 4)         { local = true; }   // SET_FEATURE(PORT_RESET)
    if (local) {
        if (!_discard(outBytes)) return false;
        _respondSimple(seq, lst, 0);
        return true;
    }

    // The host stack requires num_bytes == 8 + wLength exactly.
    const int need = 8 + (int)wLen;
    if ((uint32_t)len != wLen || ((rt & 0x80) != 0) != in || need > _maxData()) {
        if (!_discard(outBytes)) return false;
        _respondSimple(seq, ST_EPIPE, 0);
        return true;
    }

    Urb* u = _acquire(need);
    if (!u) {
        if (!_discard(outBytes)) return false;
        _respondSimple(seq, ST_ENOMEM, 0);
        return true;
    }

    usb_transfer_t* x = u->xfer;
    memcpy(x->data_buffer, setup, 8);
    if (!in && len > 0 && _recvAll(x->data_buffer + 8, len, USBIP_PAYLOAD_TIMEOUT_MS) != 0) {
        LOCK(); _freeLocked(u); UNLOCK();
        return false;
    }
    x->num_bytes = need;
    x->flags     = 0;

    // CLEAR_FEATURE(ENDPOINT_HALT) goes to the device; the host stack's own pipe
    // must be un-halted too, which the completion handler does.
    const int16_t clearEp = (rt == 0x02 && rq == 0x01 && wValue == 0) ? (int16_t)(wIndex & 0xFF) : -1;
    _launch(u, seq, 0, true, in, len, clearEp);
    return true;
}

bool UsbIpServer::_submitData(uint32_t seq, bool in, uint8_t epn, uint32_t flags, int32_t len) {
    const uint8_t addr = epn | (in ? 0x80 : 0x00);
    const UsbIpEndpoint* ep = _device->findEndpoint(addr);
    const size_t outBytes = in ? 0 : (size_t)len;

    if (!ep) {                                       // not a claimed endpoint (e.g. isochronous)
        if (!_discard(outBytes)) return false;
        _respondSimple(seq, ST_EPIPE, 0);
        return true;
    }
    if (len == 0) {                                  // nothing to transfer: complete successfully
        _respondSimple(seq, ST_OK, 0);
        return true;
    }

    uint16_t mps = ep->maxPacketSize & 0x7FF;
    if (mps == 0) mps = 64;
    const int maxd = _maxData();

    if (!in) {
        if (len > maxd) return _chunkedOut(seq, addr, flags, len);
        Urb* u = _acquire(len);
        if (!u) {
            if (!_discard(outBytes)) return false;
            _respondSimple(seq, ST_ENOMEM, 0);
            return true;
        }
        usb_transfer_t* x = u->xfer;
        if (_recvAll(x->data_buffer, len, USBIP_PAYLOAD_TIMEOUT_MS) != 0) {
            LOCK(); _freeLocked(u); UNLOCK();
            return false;
        }
        x->num_bytes = len;
        x->flags     = (flags & URB_ZERO_PACKET) ? USB_TRANSFER_FLAG_ZERO_PACK : 0;
        _launch(u, seq, addr, false, false, len, -1);
        return true;
    }

    // IN. Bulk endpoints with read-ahead are answered from the ring.
    const bool intr = (ep->type == USB_TRANSFER_TYPE_INTR);
    RxEp* r = intr ? nullptr : _rxFind(addr);
    if (r) {
        const int n = len > maxd ? maxd : len;       // a short read is always legal
        Urb* u = _acquire(n);
        if (!u) { _respondSimple(seq, ST_ENOMEM, 0); return true; }
        LOCK();
        u->seqnum = seq; u->epAddr = addr; u->isCtrl = false; u->isIn = true;
        u->bufLen = n; u->got = 0; u->unlinked = false;
        u->state  = URB_RING;
        _rxServiceLocked(*r);                        // data may already be waiting
        UNLOCK();
        _wakeSender();
        return true;
    }

    // Asynchronous read. The host stack wants a multiple of MPS; round DOWN (at least one
    // packet) so the device can never send more than the client asked for. An interrupt
    // transfer is one packet: a longer request would only complete on a short packet, and
    // an exactly-MPS report (a keyboard) would sit unreported until the buffer filled.
    int n;
    if (intr) n = mps;
    else { n = len > maxd ? maxd : len; n = (n / mps) * mps; if (n == 0) n = mps; }
    Urb* u = _acquire(n);
    if (!u) { _respondSimple(seq, ST_ENOMEM, 0); return true; }
    u->xfer->num_bytes = n;
    u->xfer->flags     = 0;
    _launch(u, seq, addr, false, true, len < n ? len : n, -1);
    return true;
}

// A bulk OUT larger than one pool slot is sent slot by slot. Two slots are used as a
// double buffer: while one piece is on the wire the next one is read from the socket
// straight into the other (DMA) buffer, so network and USB overlap. Pieces are queued on
// the endpoint in order. The URB is answered once, when every piece has finished.
// (usbipdcpp leaves chunking disabled and instead allocates one transfer for the whole
// URB; the S3's ~300 KB of internal DMA RAM cannot do that for large writes.)
bool UsbIpServer::_chunkedOut(uint32_t seq, uint8_t addr, uint32_t flags, int32_t len) {
    const int maxd = _maxData();
    Urb* u[2] = { _acquire(maxd), nullptr };
    if (u[0]) u[1] = _acquire(maxd);                 // best effort: one buffer still works, just serially
    if (!u[0]) {
        if (!_discard((size_t)len)) return false;
        _respondSimple(seq, ST_ENOMEM, 0);
        return true;
    }
    const int nbuf = u[1] ? 2 : 1;
    while (xSemaphoreTake(_chunkSem, 0) == pdTRUE) {}   // drop stale wake-ups

    bool inflight[2] = { false, false };
    int32_t remaining = len, total = 0, status = ST_OK;

    // Waits for piece `i` and accounts for it. false = unrecoverable (session must end).
    auto finish = [&](int i) -> bool {
        Urb* c = u[i];
        const uint32_t start = millis();
        while (!c->chunkDone) {
            xSemaphoreTake(_chunkSem, pdMS_TO_TICKS(50));
            if (c->chunkDone) break;
            if (_abort()) return false;              // teardown cancels and recycles the URBs
            if (millis() - start > USBIP_CHUNK_TIMEOUT_MS) {
                _device->cancelEndpoint(addr);       // stuck: flush the endpoint
                const uint32_t t0 = millis();
                while (!c->chunkDone && millis() - t0 < USBIP_CANCEL_WAIT_MS * 3)
                    xSemaphoreTake(_chunkSem, pdMS_TO_TICKS(20));
                if (!c->chunkDone) return false;     // still owned by the host: end the session
                _device->clearEndpoint(addr);
                if (status == ST_OK) status = ST_ETIMEDOUT;
                break;
            }
        }
        LOCK();
        const usb_transfer_status_t st = (usb_transfer_status_t)c->chunkStatus;
        const int actual = c->chunkActual;
        UNLOCK();
        if (st == USB_TRANSFER_STATUS_COMPLETED) total += actual;
        else if (status == ST_OK) status = _mapStatus(st);
        inflight[i] = false;
        return true;
    };

    int cur = 0;
    while (remaining > 0) {
        Urb* c = u[cur];
        if (inflight[cur] && !finish(cur)) return false;       // the buffer must be idle before it is refilled

        const int n = remaining < maxd ? remaining : maxd;
        if (_recvAll(c->xfer->data_buffer, n, USBIP_PAYLOAD_TIMEOUT_MS) != 0) return false;
        remaining -= n;

        if (status == ST_OK) {                       // after an error, only drain the stream
            usb_transfer_t* x = c->xfer;
            x->num_bytes        = n;
            x->flags            = (remaining == 0 && (flags & URB_ZERO_PACKET)) ? USB_TRANSFER_FLAG_ZERO_PACK : 0;
            x->bEndpointAddress = addr;
            x->timeout_ms       = 0;
            x->callback         = _urbDone;
            x->context          = c;
            LOCK();
            c->seqnum = seq; c->epAddr = addr; c->isCtrl = false; c->isIn = false;
            c->bufLen = n; c->unlinked = false; c->chunked = true; c->chunkDone = false;
            c->state  = URB_PENDING;
            const esp_err_t err = _device->submitTransfer(x);
            if (err != ESP_OK) { c->state = URB_RESERVED; c->chunked = false; }
            UNLOCK();
            if (err != ESP_OK) status = (err == ESP_ERR_INVALID_STATE) ? ST_ENODEV : ST_EPIPE;
            else inflight[cur] = true;
        }
        cur = (cur + 1) % nbuf;
    }
    for (int i = 0; i < nbuf; i++)
        if (inflight[i] && !finish(i)) return false;

    UrbHeader h;
    makeRetSubmit(h, seq, status, (uint32_t)total);
    LOCK();
    for (int i = 0; i < nbuf; i++) u[i]->chunked = false;
    if (u[1]) _freeLocked(u[1]);
    u[0]->state = URB_QUEUED;
    if (!_pushLocked(h, u[0], nullptr, 0, false)) _freeLocked(u[0]);
    UNLOCK();
    _wakeSender();
    return true;
}

// ── Completion (usb_host task) ────────────────────────────────────────────────

void UsbIpServer::_urbDone(usb_transfer_t* t) {
    Urb* u = static_cast<Urb*>(t->context);
    u->owner->_onUrbDone(u);
}

void UsbIpServer::_onUrbDone(Urb* u) {
    LOCK();
    usb_transfer_t* x = u->xfer;
    if (u->state != URB_PENDING) { UNLOCK(); return; }
    if (u->gen != _gen || _stopping) { _freeLocked(u); UNLOCK(); return; }   // its session is over
    if (u->chunked) {                                   // a piece of a pipelined OUT
        u->chunkStatus = x->status;
        u->chunkActual = x->actual_num_bytes;
        u->chunkDone   = true;                          // the flag is the truth; the semaphore only wakes
        UNLOCK();
        xSemaphoreGive(_chunkSem);
        return;
    }
    if (u->unlinked) { _freeLocked(u); UNLOCK(); return; }                   // already answered via RET_UNLINK

    // Cancelled by an endpoint flush meant for another URB: the session task resubmits it.
    if (x->status == USB_TRANSFER_STATUS_CANCELED) { u->state = URB_RESUBMIT; UNLOCK(); return; }

    const int32_t status = _mapStatus(x->status);
    uint32_t actual = 0;
    const uint8_t* data = nullptr;
    if (x->status == USB_TRANSFER_STATUS_COMPLETED) {
        int n = x->actual_num_bytes;
        if (u->isCtrl) { n -= 8; if (n < 0) n = 0; }
        if (n > u->bufLen) n = u->bufLen;            // only when the client asked for less than one packet
        actual = (uint32_t)n;
        if (u->isIn && n > 0) data = x->data_buffer + (u->isCtrl ? 8 : 0);

        // CLEAR_HALT reached the device: un-halt the host pipe too. Skip it while other
        // transfers are queued on that endpoint (the pipe is then not halted, and clearing
        // an active pipe would disturb them) or while read-ahead keeps it armed.
        if (u->clearEp >= 0) {
            const uint8_t cep = (uint8_t)u->clearEp;
            bool busy = _rxFind(cep) != nullptr;
            for (int i = 0; i < _urbCount && !busy; i++)
                if (&_urbs[i] != u && _urbs[i].state == URB_PENDING && !_urbs[i].isCtrl &&
                    _urbs[i].epAddr == cep) busy = true;
            if (!busy) _device->clearEndpoint(cep);
        }
    }

    UrbHeader h;
    makeRetSubmit(h, u->seqnum, status, actual);
    u->state = URB_QUEUED;
    if (!_pushLocked(h, u, data, data ? actual : 0, false)) _freeLocked(u);
    UNLOCK();
    _wakeSender();
}

// ── CMD_UNLINK ───────────────────────────────────────────────────────────────
//
// The kernel needs RET_SUBMIT before RET_UNLINK, and the queue is one FIFO, so:
//  - URB still pending: cancel it, answer RET_UNLINK(-ECONNRESET), never RET_SUBMIT.
//  - URB already completed (its RET_SUBMIT is queued or sent) or unknown: RET_UNLINK(0).

int UsbIpServer::_pendingOn(uint8_t epAddr) {
    int n = 0;
    LOCK();
    for (int i = 0; i < _urbCount; i++)
        if (_urbs[i].state == URB_PENDING && !_urbs[i].isCtrl && _urbs[i].epAddr == epAddr) n++;
    UNLOCK();
    return n;
}

void UsbIpServer::_handleUnlink(const UrbHeader& h) {
    _nUnlink = _nUnlink + 1;
    const uint32_t seq    = be32(h.seqnum);
    const uint32_t victim = be32(h.u.unlink.unlink_seqnum);
    int32_t status = ST_OK;
    bool    cancel = false;
    uint8_t ep     = 0;

    LOCK();
    Urb* v = nullptr;
    for (int i = 0; i < _urbCount; i++) {
        Urb& u = _urbs[i];
        if (u.seqnum == victim && !u.unlinked &&
            (u.state == URB_PENDING || u.state == URB_RING || u.state == URB_RESUBMIT)) { v = &u; break; }
    }
    if (v) {
        status = ST_ECONNRESET;
        if (v->state == URB_RING) {
            // Part-filled data goes back to the front of the endpoint's stream, so
            // cancelling a read never loses bytes.
            RxEp* r = _rxFind(v->epAddr);
            if (r && v->got > 0 && r->carryLen + v->got <= USBIP_RX_CARRY) {
                memmove(r->carry + v->got, r->carry, r->carryLen);
                memcpy(r->carry, v->xfer->data_buffer, v->got);
                r->carryLen += v->got;
            }
            _freeLocked(v);
        } else if (v->state == URB_RESUBMIT) {
            _freeLocked(v);
        } else {
            v->unlinked = true;
            if (!v->isCtrl) { cancel = true; ep = v->epAddr; }   // EP0 cannot be cancelled on its own
        }
    }
    UNLOCK();

    if (cancel) {
        _device->cancelEndpoint(ep);                 // flushes every URB queued on this endpoint
        const uint32_t start = millis();
        while (_pendingOn(ep) > 0 && millis() - start < USBIP_CANCEL_WAIT_MS) vTaskDelay(1);
        if (_pendingOn(ep) == 0) {
            // Un-halt now: the client usually resubmits a read the instant it gets RET_UNLINK,
            // and a halted pipe would reject it. Innocent URBs caught by the flush go back in.
            _device->clearEndpoint(ep);
            _resubmitEndpoint(ep);
        } else {
            _USBIP_LOGW("USB ep 0x%02X did not flush in time", ep);
        }
    }
    _respondUnlink(seq, status);
}

void UsbIpServer::_resubmitEndpoint(uint8_t epAddr) {
    LOCK();
    for (;;) {
        Urb* best = nullptr;                         // oldest first, to keep submission order
        for (int i = 0; i < _urbCount; i++) {
            Urb& u = _urbs[i];
            if (u.state == URB_RESUBMIT && u.epAddr == epAddr &&
                (!best || (int32_t)(u.order - best->order) < 0)) best = &u;
        }
        if (!best) break;
        best->state = URB_PENDING;
        if (_device->submitTransfer(best->xfer) != ESP_OK) {
            UrbHeader h;
            makeRetSubmit(h, best->seqnum, ST_EPIPE, 0);
            best->state = URB_QUEUED;
            if (!_pushLocked(h, best, nullptr, 0, false)) _freeLocked(best);
        }
    }
    UNLOCK();
    _wakeSender();
}

// Cancels everything the USB host still owns and waits (bounded) for the completions.
// A URB whose completion never arrives stays PENDING, so its slot is not reused until
// its callback finally runs (it then sees a stale session and frees itself).
void UsbIpServer::_cancelAllUrbs() {
    uint8_t eps[USBIP_MAX_URBS];
    int ne = 0;
    LOCK();
    for (int i = 0; i < _urbCount; i++) {
        const Urb& u = _urbs[i];
        if (u.state != URB_PENDING || u.isCtrl) continue;
        bool dup = false;
        for (int k = 0; k < ne; k++) if (eps[k] == u.epAddr) dup = true;
        if (!dup) eps[ne++] = u.epAddr;
    }
    UNLOCK();

    for (int k = 0; k < ne; k++) _device->cancelEndpoint(eps[k]);   // no-op if the device is gone

    const uint32_t start = millis();
    for (;;) {
        int pending = 0;
        LOCK();
        for (int i = 0; i < _urbCount; i++) if (_urbs[i].state == URB_PENDING) pending++;
        UNLOCK();
        if (pending == 0) break;
        if (millis() - start > USBIP_TEARDOWN_WAIT_MS) {
            _USBIP_LOGE("USB %d transfer(s) not completed after cancel; slots stay reserved until they finish", pending);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    for (int k = 0; k < ne; k++) _device->clearEndpoint(eps[k]);
}

// ── Bulk IN read-ahead ───────────────────────────────────────────────────────

void UsbIpServer::_rxDone(usb_transfer_t* t) {
    RxArm* a = static_cast<RxArm*>(t->context);
    a->owner->_onRxDone(*a);
}

UsbIpServer::RxEp* UsbIpServer::_rxFind(uint8_t addr) {
    for (int i = 0; i < USBIP_MAX_RX_EPS; i++)
        if (_rx[i].active && _rx[i].addr == addr) return &_rx[i];
    return nullptr;
}

void UsbIpServer::_rxStart() {
    const int n = _device->getEndpointCount();
    for (int i = 0; i < n; i++) {
        uint8_t addr, type; uint16_t mps;
        if (!_device->getEndpointInfo(i, addr, type, mps)) continue;
        if (!(addr & 0x80) || type != USB_TRANSFER_TYPE_BULK || mps == 0 || mps > USBIP_RX_PKT_MAX) continue;
        if (_rxFind(addr)) continue;
        if (!USBIP_READAHEAD_ALL) {
            const UsbIpEndpoint* e = _device->findEndpoint(addr);
            const uint8_t cls = e ? e->ifaceClass : 0xFF;
            if (cls != 0xFF && cls != 0x02 && cls != 0x0A) continue;   // not a streaming endpoint
        }

        int k = -1;
        for (int j = 0; j < USBIP_MAX_RX_EPS; j++) if (!_rx[j].active && !_rx[j].dead) { k = j; break; }
        if (k < 0) break;
        RxEp& r = _rx[k];

        RxPkt*   ring  = (RxPkt*)malloc(sizeof(RxPkt) * USBIP_RX_RING);
        uint8_t* carry = (uint8_t*)malloc(USBIP_RX_CARRY);
        int made = 0;
        usb_transfer_t* xs[USBIP_RX_ARMS];
        if (ring && carry)
            for (; made < USBIP_RX_ARMS; made++)
                if (usb_host_transfer_alloc(mps, 0, &xs[made]) != ESP_OK) break;
        if (!ring || !carry || made == 0) {
            for (int m = 0; m < made; m++) usb_host_transfer_free(xs[m]);
            free(ring); free(carry);
            _USBIP_LOGW("USB read-ahead for ep 0x%02X: out of memory, reading on demand", addr);
            continue;
        }

        LOCK();
        r.addr = addr; r.mps = mps; r.ring = ring; r.carry = carry;
        r.head = r.count = 0; r.carryLen = 0; r.armed = 0; r.retryAt = 0; r.needClear = false;
        for (int a = 0; a < USBIP_RX_ARMS; a++) {
            RxArm& arm = r.arm[a];
            arm.xfer = (a < made) ? xs[a] : nullptr;
            arm.owner = this; arm.ep = (int8_t)k; arm.inUsb = false;
            if (arm.xfer) {
                arm.xfer->bEndpointAddress = addr;
                arm.xfer->callback  = _rxDone;
                arm.xfer->context   = &arm;
                arm.xfer->flags     = 0;
                arm.xfer->timeout_ms = 0;
            }
        }
        r.active = true;
        _rxArmLocked(r);
        UNLOCK();
        _USBIP_LOGD("USB read-ahead on ep 0x%02X (%d armed)", addr, r.armed);
    }
}

// Keeps the device polled: re-queues idle arms while the ring has room.
void UsbIpServer::_rxArmLocked(RxEp& r) {
    if (!r.active || _stopping || r.needClear) return;
    if (!timeReached(millis(), r.retryAt)) return;
    for (int i = 0; i < USBIP_RX_ARMS; i++) {
        RxArm& a = r.arm[i];
        if (!a.xfer || a.inUsb) continue;
        if (r.count + r.armed >= USBIP_RX_RING) return;          // ring full: let the device NAK
        a.xfer->num_bytes = r.mps;
        if (_device->submitTransfer(a.xfer) != ESP_OK) { r.retryAt = millis() + 200; return; }
        a.inUsb = true;
        r.armed++;
    }
}

void UsbIpServer::_onRxDone(RxArm& a) {
    LOCK();
    RxEp& r = _rx[a.ep];
    a.inUsb = false;
    if (r.armed > 0) r.armed--;
    if (!r.active || _stopping) { UNLOCK(); return; }

    usb_transfer_t* x = a.xfer;
    if (x->status == USB_TRANSFER_STATUS_COMPLETED) {
        if (r.count < USBIP_RX_RING) {
            int n = x->actual_num_bytes;
            if (n > r.mps) n = r.mps;
            RxPkt& p = r.ring[(r.head + r.count) % USBIP_RX_RING];
            p.len = (uint8_t)n; p.off = 0;
            memcpy(p.data, x->data_buffer, n);
            r.count++;
        }
    } else if (x->status != USB_TRANSFER_STATUS_CANCELED &&
               x->status != USB_TRANSFER_STATUS_NO_DEVICE) {
        r.needClear = true;                                       // stall or error: _rxMaintain un-halts it
        r.retryAt   = millis() + 50;
    }
    _rxServiceLocked(r);
    UNLOCK();
    _wakeSender();
}

// Completes client read URBs from the ring, oldest first, with real USB semantics: a
// URB ends when its buffer is full or a short (or zero-length) packet ends the transfer.
void UsbIpServer::_rxServiceLocked(RxEp& r) {
    for (;;) {
        Urb* s = nullptr;
        for (int i = 0; i < _urbCount; i++) {
            Urb& c = _urbs[i];
            if (c.state == URB_RING && c.epAddr == r.addr &&
                (!s || (int32_t)(c.order - s->order) < 0)) s = &c;
        }
        if (!s) break;

        uint8_t* buf = s->xfer->data_buffer;
        bool term = false;
        if (r.carryLen > 0 && s->got < s->bufLen) {
            int n = min(r.carryLen, (int)(s->bufLen - s->got));
            memcpy(buf + s->got, r.carry, n);
            s->got += n;
            memmove(r.carry, r.carry + n, r.carryLen - n);
            r.carryLen -= n;
        }
        while (s->got < s->bufLen && r.carryLen == 0 && r.count > 0) {
            RxPkt& p = r.ring[r.head];
            int n = min((int)(p.len - p.off), (int)(s->bufLen - s->got));
            memcpy(buf + s->got, p.data + p.off, n);
            p.off += n; s->got += n;
            if (p.off >= p.len) {
                const bool shortPkt = p.len < r.mps;
                r.head = (r.head + 1) % USBIP_RX_RING; r.count--;
                if (shortPkt) { term = true; break; }
            }
        }
        if (!(s->got >= s->bufLen || term)) break;                // waiting for more data

        UrbHeader h;
        makeRetSubmit(h, s->seqnum, ST_OK, (uint32_t)s->got);
        s->state = URB_QUEUED;
        if (!_pushLocked(h, s, buf, s->got, false)) _freeLocked(s);
    }
    _rxArmLocked(r);                                              // ring space freed: keep the device polled
}

// Runs on the sender task every tick: un-halts a stalled endpoint and re-arms idle slots.
void UsbIpServer::_rxMaintain() {
    LOCK();
    if (!_stopping) {
        for (int e = 0; e < USBIP_MAX_RX_EPS; e++) {
            RxEp& r = _rx[e];
            if (!r.active) continue;
            if (r.needClear && r.armed == 0 && timeReached(millis(), r.retryAt)) {
                _device->clearEndpoint(r.addr);
                r.needClear = false;
            }
            _rxArmLocked(r);
        }
    }
    UNLOCK();
}

void UsbIpServer::_rxStop() {
    for (int e = 0; e < USBIP_MAX_RX_EPS; e++) {
        RxEp& r = _rx[e];
        if (!r.active) continue;

        LOCK(); r.active = false; bool any = r.armed > 0; UNLOCK();   // completions stop servicing
        if (any) _device->cancelEndpoint(r.addr);

        const uint32_t start = millis();
        for (;;) {
            LOCK(); int armed = r.armed; UNLOCK();
            if (armed == 0 || millis() - start > 1000) break;
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        LOCK();
        bool stuck = false;
        for (int i = 0; i < USBIP_RX_ARMS; i++) {
            RxArm& a = r.arm[i];
            if (!a.xfer) continue;
            if (a.inUsb) { stuck = true; continue; }
            usb_host_transfer_free(a.xfer);
            a.xfer = nullptr;
        }
        if (stuck) {
            // The host still owns a transfer: keep the ring and the slot, never reuse them.
            r.dead = true;
            _USBIP_LOGE("USB read-ahead transfer on ep 0x%02X could not be cancelled; slot retired", r.addr);
        } else {
            free(r.ring); free(r.carry);
            r.ring = nullptr; r.carry = nullptr;
        }
        r.armed = 0; r.count = 0; r.carryLen = 0; r.needClear = false;
        UNLOCK();
        _device->clearEndpoint(r.addr);
    }
}

// ── Status JSON ──────────────────────────────────────────────────────────────

String UsbIpServer::statusJson() const {
    String j = "{";
    j += "\"attached\":" + String(_attached ? "true" : "false") + ",";

    if (_curIP[0]) {
        uint32_t connSec = _connectedMs ? (uint32_t)((millis() - _connectedMs) / 1000) : 0;
        uint32_t attSec  = _attachedMs  ? (uint32_t)((millis() - _attachedMs)  / 1000) : 0;
        j += "\"client\":{\"ip\":\"" + String(_curIP) + "\""
           + ",\"connectedSec\":"    + String(connSec)
           + ",\"attachedSec\":"     + String(attSec) + "},";
    } else {
        j += "\"client\":null,";
    }

    uint32_t nowSec = (uint32_t)(millis() / 1000);
    j += "\"history\":[";
    bool first = true;
    for (int i = 1; i <= MAX_HISTORY; i++) {
        int idx = (_histNext - i + MAX_HISTORY) % MAX_HISTORY;
        const UsbIpSessionRecord& r = _history[idx];
        if (!r.valid) continue;
        if (!first) j += ",";
        first = false;
        uint32_t agoSec = (r.endedAt <= nowSec) ? (nowSec - r.endedAt) : 0;
        j += "{\"ip\":\"" + String(r.ip) + "\""
           + ",\"durationSec\":" + String(r.durationSec)
           + ",\"agoSec\":"      + String(agoSec) + "}";
    }
    j += "]}";
    return j;
}
