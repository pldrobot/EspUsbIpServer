#include "UsbIpServer.h"
#include "UsbDevice.h"
#include "EspUsbIpLog.h"
#include <errno.h>
#include <lwip/sockets.h>
#include <lwip/tcp.h>

// ── Big-endian helpers ────────────────────────────────────────────────────────

void     UsbIpServer::_w32(uint8_t* p, uint32_t v) { p[0]=v>>24; p[1]=v>>16; p[2]=v>>8; p[3]=v; }
void     UsbIpServer::_w16(uint8_t* p, uint16_t v) { p[0]=v>>8;  p[1]=v; }
uint32_t UsbIpServer::_r32(const uint8_t* p)       { return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]; }
uint16_t UsbIpServer::_r16(const uint8_t* p)       { return ((uint16_t)p[0]<<8)|p[1]; }

// ── I/O ───────────────────────────────────────────────────────────────────────

bool UsbIpServer::_readExact(uint8_t* buf, size_t len, uint32_t timeoutMs) {
    size_t got = 0;
    unsigned long deadline = millis() + timeoutMs;
    while (got < len) {
        if (!_client.connected()) return false;
        int avail = _client.available();
        if (avail > 0) {
            int n = _client.read(buf + got, min((int)(len - got), avail));
            if (n > 0) got += n;
        } else if (millis() > deadline) {
            return false;
        } else {
            delay(1);
        }
    }
    return true;
}

bool UsbIpServer::_writeAll(const uint8_t* buf, size_t len) {
    size_t wrote = 0;
    while (wrote < len) {
        int n = _client.write(buf + wrote, len - wrote);
        if (n <= 0) return false;
        wrote += n;
    }
    return true;
}

// ── Device info (312 bytes) ───────────────────────────────────────────────────

void UsbIpServer::_fillDevInfo(uint8_t out[312]) {
    memset(out, 0, 312);
    strncpy((char*)out,         "/sys/devices/esp32/usb/1-1", 255);
    strncpy((char*)(out + 256), USBIP_BUSID, 31);
    _w32(out + 288, USBIP_BUSNUM);
    _w32(out + 292, USBIP_DEVNUM);
    // The ESP32-S3's USB-OTG peripheral has no High-Speed PHY — report the
    // device's real negotiated speed (Low or Full), not a hardcoded value.
    uint32_t speed = (_device->getSpeed() == USB_SPEED_LOW) ? USBIP_SPEED_LOW : USBIP_SPEED_FULL;
    _w32(out + 296, speed);
    _w16(out + 300, _device->getVendorId());
    _w16(out + 302, _device->getProductId());
    // bcdDevice, bDeviceClass/SubClass/Protocol, bNumConfigurations come from the
    // real device descriptor; bConfigurationValue from the active configuration.
    const uint8_t* dd = _device->getDevDescRaw();
    const uint8_t* cd = _device->getCfgDescRaw();
    bool haveDev = _device->getDevDescLen() >= 18;
    bool haveCfg = _device->getCfgDescLen() >= 6;
    _w16(out + 304, haveDev ? (uint16_t)(dd[12] | (dd[13] << 8)) : 0x0100);
    out[306] = haveDev ? dd[4]  : 0;
    out[307] = haveDev ? dd[5]  : 0;
    out[308] = haveDev ? dd[6]  : 0;
    out[309] = haveCfg ? cd[5]  : 1;
    out[310] = haveDev ? dd[17] : 1;
    out[311] = (uint8_t)_device->getIfaceCount();
}

// ── Session tracking ──────────────────────────────────────────────────────────

void UsbIpServer::_onConnect(const String& ip) {
    strncpy(_curIP, ip.c_str(), sizeof(_curIP) - 1);
    _connectedMs = millis();
    _attachedMs  = 0;
    _attached    = false;   // a new TCP connection can never belong to an attached session
    _USBIP_LOGI("Connected: %s", _curIP);
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

    UsbIpSessionRecord& r = _history[_histNext];
    r.valid       = true;
    strncpy(r.ip, _curIP, sizeof(r.ip) - 1);
    r.endedAt     = (uint32_t)(millis() / 1000);
    r.durationSec = _attachedMs ? (uint32_t)((millis() - _attachedMs) / 1000) : 0;
    _histNext = (_histNext + 1) % MAX_HISTORY;

    _USBIP_LOGI("Disconnected: %s (attached %us)", _curIP, r.durationSec);
    if (_attached) _device->setMaintenanceMode(false);
    if (_onDisconnectCb) _onDisconnectCb(r.ip, r.durationSec);
    memset(_curIP, 0, sizeof(_curIP));
    _connectedMs = 0;
    _attachedMs  = 0;
    _attached    = false;
}

// ── Task ──────────────────────────────────────────────────────────────────────

void UsbIpServer::_serverTask(void* arg) {
    UsbIpServer* self = static_cast<UsbIpServer*>(arg);
    for (;;) {
        self->_poll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// NB: WiFiClient is NetworkClient, whose operator bool() calls connected(), so
// `_client && !_client.connected()` can never be true. Session state is tracked
// with _curIP instead ("we believe a session exists").

bool UsbIpServer::_peerGone() {
    int fd = _client.fd();
    if (fd < 0) return true;
    uint8_t b;
    int r = recv(fd, &b, 1, MSG_PEEK | MSG_DONTWAIT);
    if (r == 0) return true;                                   // orderly shutdown (FIN)
    if (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN && errno != EINTR) return true;
    return false;
}

bool UsbIpServer::_sessionAlive() {
    return _client.connected() && !_peerGone();
}

void UsbIpServer::_dropSession() {
    _cancelAllPending();
    _rxStop();
    if (_curIP[0]) _onDisconnect();   // history, callbacks, maintenance mode, resets state
    _client.stop();                   // safe on a closed / never-opened client
    _attached = false;
}

// Detect vanished peers (power loss, network drop) that never send FIN/RST: ~35 s.
void UsbIpServer::_enableKeepAlive() {
    int fd = _client.fd();
    if (fd < 0) return;
    int on = 1, idle = 20, intvl = 5, cnt = 3;
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
}

void UsbIpServer::_poll() {
    if (_kickReq) {
        _kickReq = false;
        _dropSession();
        _USBIP_LOGI("Client kicked by caller");
    }

    if (_server.hasClient()) {
        WiFiClient incoming = _server.accept();
        if (_curIP[0] && _sessionAlive()) {
            _USBIP_LOGW("Rejected %s — device held by %s",
                        incoming.remoteIP().toString().c_str(), _curIP);
            incoming.stop();
        } else {
            _dropSession();           // clear any stale session before taking the new one
            _client = incoming;
            _enableKeepAlive();
            _onConnect(_client.remoteIP().toString());
        }
    }

    // Drop detection
    if (_curIP[0] && !_sessionAlive()) _dropSession();
    if (_curIP[0] && _attached && !_device->isConnected()) {
        _USBIP_LOGW("USB device removed while attached - closing session");
        _dropSession();
    }
    if (!_curIP[0]) return;

    if (!_attached) {
        if (_client.available() >= 8) {
            uint8_t hdr[8];
            if (!_readExact(hdr, 8)) { _client.stop(); return; }
            uint16_t cmd = _r16(hdr + 2);
            if      (cmd == OP_REQ_DEVLIST) _handleDevList();
            else if (cmd == OP_REQ_IMPORT)  _handleImport();
        }
    } else {
        _drainPending();
        _rxPump();
        _rxService();
        if (_curIP[0] && _client.available() >= 48) _handleUrbs();
    }
}

// ── Public ────────────────────────────────────────────────────────────────────

void UsbIpServer::begin(UsbDevice* device) {
    _device = device;
    _server.begin(USBIP_PORT);
    _server.setNoDelay(true);
    BaseType_t core = (xPortGetCoreID() ^ 1);
    xTaskCreatePinnedToCore(_serverTask, "usbip_srv", 8192, this, 4, &_taskHandle, core);
    _USBIP_LOGI("Server on port %d (core %d) — concurrent connections rejected", USBIP_PORT, (int)core);
}

void UsbIpServer::stop() {
    if (_taskHandle) {
        vTaskSuspend(_taskHandle);
        _kickReq = false;
        _dropSession();
        _USBIP_LOGI("Server stopped");
    }
}

void UsbIpServer::start() {
    if (_taskHandle) {
        vTaskResume(_taskHandle);
        _USBIP_LOGI("Server started");
    }
}

// ── Handshake ─────────────────────────────────────────────────────────────────

bool UsbIpServer::_handleDevList() {
    bool has = _device->isConnected();
    uint8_t rep[12];
    _w16(rep + 0, USBIP_VERSION);
    _w16(rep + 2, OP_REP_DEVLIST);
    _w32(rep + 4, 0);
    _w32(rep + 8, has ? 1 : 0);
    if (!_writeAll(rep, 12)) return false;
    if (has) {
        uint8_t devinfo[312]; _fillDevInfo(devinfo);
        if (!_writeAll(devinfo, 312)) return false;
        // One interface-descriptor entry per claimed interface, sourced from
        // the real device (printer, CDC-ACM composite, vendor-specific, ...)
        // instead of a fixed printer-class tuple.
        int n = _device->getIfaceCount();
        for (int i = 0; i < n; i++) {
            uint8_t iface[4];
            if (!_device->getIfaceDesc(i, iface)) break;
            if (!_writeAll(iface, 4)) return false;
        }
    }
    return true;
}

bool UsbIpServer::_handleImport() {
    uint8_t busid[32];
    if (!_readExact(busid, 32)) return false;

    uint8_t rep[8];
    _w16(rep + 0, USBIP_VERSION);
    _w16(rep + 2, OP_REP_IMPORT);

    if (!_device->isConnected()) {
        _w32(rep + 4, 1);
        _writeAll(rep, 8);
        _USBIP_LOGE("IMPORT rejected: device not connected");
        return false;
    }

    _w32(rep + 4, 0);
    if (!_writeAll(rep, 8)) return false;
    uint8_t devinfo[312]; _fillDevInfo(devinfo);
    if (!_writeAll(devinfo, 312)) return false;
    _onAttach();
    return true;
}

// ── URB forwarding ────────────────────────────────────────────────────────────

void UsbIpServer::_handleUrbs() {
    uint8_t hdr[48];
    if (!_readExact(hdr, 48)) { _client.stop(); return; }

    uint32_t command   = _r32(hdr +  0);
    uint32_t seqnum    = _r32(hdr +  4);
    uint32_t direction = _r32(hdr + 12);
    uint32_t ep        = _r32(hdr + 16);
    int32_t  bufLen    = (int32_t)_r32(hdr + 24);
    uint8_t  setup[8]; memcpy(setup, hdr + 40, 8);

    if (command == USBIP_CMD_SUBMIT) {
        _processSubmit(seqnum, direction, ep, setup, bufLen);
    } else if (command == USBIP_CMD_UNLINK) {
        _handleUnlink(seqnum, _r32(hdr + 20));
    }
}

bool UsbIpServer::_processSubmit(uint32_t seqnum, uint32_t direction, uint32_t ep,
                                   const uint8_t setup[8], int32_t bufLen) {
    uint8_t* buf = nullptr;
    if (bufLen > 0) {
        buf = (uint8_t*)malloc(bufLen);
        if (!buf) { _client.stop(); return false; }
    }

    int actual = 0; int32_t status = 0;

    if (ep == 0) {
        if (direction == USBIP_DIR_OUT && bufLen > 0)
            if (!_readExact(buf, bufLen)) { free(buf); return false; }
        usb_setup_packet_t pkt; memcpy(&pkt, setup, 8);
        actual = _device->submitControlTransfer(&pkt, buf, bufLen, 3000);
        if (actual < 0) { status = -32; actual = 0; }
        // CLEAR_FEATURE(ENDPOINT_HALT) was forwarded to the device; the host stack's own
        // pipe also needs clearing. Skip it while reads are queued on that endpoint: then
        // the pipe is not halted, and clearing an active pipe would disturb them.
        if (status == 0 && setup[0] == 0x02 && setup[1] == 0x01 && setup[2] == 0 && setup[3] == 0) {
            bool busy = false;
            for (int i = 0; i < MAX_PENDING; i++)
                if (_pending[i].used && !_pending[i].done && _pending[i].epAddr == setup[4]) busy = true;
            if (_rxFind(setup[4])) busy = true;        // read-ahead keeps it armed
            if (!busy) _device->clearEndpoint(setup[4]);
        }
        _USBIP_TRACE("CTRL setup", 0, 8, setup, status);
        if (direction == USBIP_DIR_IN) _USBIP_TRACE("CTRL in ", 0, actual, buf, status);
    } else {
        // Build the real endpoint address from the URB's endpoint number +
        // direction, and forward to it directly — works for any claimed
        // bulk/interrupt endpoint on any interface, not just a single
        // remembered pair.
        uint8_t epAddr = (uint8_t)ep | (direction == USBIP_DIR_IN ? 0x80 : 0x00);
        if (direction == USBIP_DIR_OUT) {
            if (bufLen > 0 && !_readExact(buf, bufLen)) { free(buf); return false; }
            if (bufLen > 0) {
                actual = _device->submitTransfer(epAddr, buf, bufLen, 8000);
                if (actual < 0) { status = -32; actual = 0; }
                _USBIP_TRACE("OUT", epAddr, bufLen, buf, status);
            }   // zero-length OUT: nothing to send, complete it successfully
        } else if (bufLen > 0) {
            free(buf); buf = nullptr;
            RxEp* rx = _rxFind(epAddr);
            if (bufLen > (rx ? RX_URB_MAX : 1024)) bufLen = rx ? RX_URB_MAX : 1024;   // short reads are legal; bounds RAM
            PendingUrb* slot = nullptr;
            for (int i = 0; i < MAX_PENDING; i++) if (!_pending[i].used) { slot = &_pending[i]; break; }
            if (slot && rx) {
                uint8_t* b = (uint8_t*)malloc(bufLen);
                if (b) {
                    slot->used = true; slot->done = false; slot->cancelled = false;
                    slot->seqnum = seqnum; slot->order = ++_pendingSeq; slot->epAddr = epAddr;
                    slot->bufLen = bufLen; slot->xfer = nullptr;
                    slot->ring = true; slot->buf = b; slot->got = 0;
                    return true;                     // answered later by _rxService()
                }
                slot = nullptr;
            } else if (slot) {
                slot->ring = false; slot->buf = nullptr; slot->got = 0;
                slot->used = true; slot->done = false; slot->cancelled = false;
                slot->seqnum = seqnum; slot->order = ++_pendingSeq; slot->epAddr = epAddr; slot->bufLen = bufLen; slot->xfer = nullptr;
                slot->xfer = _device->beginInTransfer(epAddr, bufLen, _urbDone, slot);
                if (slot->xfer) return true;         // answered later by _drainPending()
                slot->used = false;
            }
            if (!slot) _USBIP_LOGW("USB too many pending IN URBs (max %d), rejecting ep 0x%02X",
                                   MAX_PENDING, epAddr);
            status = slot ? -32 : -12;               // EPIPE / ENOMEM
        }
    }

    uint8_t rep[48] = {};
    _w32(rep +  0, USBIP_RET_SUBMIT);
    _w32(rep +  4, seqnum);
    _w32(rep + 20, (uint32_t)status);
    _w32(rep + 24, (uint32_t)actual);
    if (!_writeAll(rep, 48)) { free(buf); return false; }
    if (direction == USBIP_DIR_IN && actual > 0 && buf)
        if (!_writeAll(buf, actual)) { free(buf); return false; }

    free(buf);
    return true;
}

// ── Async IN URBs ─────────────────────────────────────────────────────────────

void UsbIpServer::_urbDone(usb_transfer_t* t) {
    static_cast<PendingUrb*>(t->context)->done = true;   // runs in the USB host task
}

void UsbIpServer::_drainPending() {
    for (;;) {
        // Reply in submission order. Reads on one endpoint complete in the order they
        // were queued, and a streaming device (a serial port) would be corrupted if
        // those replies went out in slot order instead.
        PendingUrb* best = nullptr;
        for (int i = 0; i < MAX_PENDING; i++) {
            PendingUrb& c = _pending[i];
            if (c.used && c.done && (!best || (int32_t)(c.order - best->order) < 0)) best = &c;
        }
        if (!best) return;
        PendingUrb& p = *best;

        usb_transfer_t* x = p.xfer;
        if (p.cancelled) {                           // already answered via RET_UNLINK
            _device->clearEndpoint(p.epAddr);
            _device->freeTransfer(x);
            p.used = false;
            continue;
        }

        int32_t status = 0; int actual = 0;
        switch (x->status) {
            case USB_TRANSFER_STATUS_COMPLETED:
                actual = x->actual_num_bytes;
                if (actual > p.bufLen) {
                    // Only possible when the client asks for fewer bytes than one packet.
                    _USBIP_LOGW("USB ep 0x%02X: %d bytes dropped (client read buffer %d < packet size)",
                                p.epAddr, actual - p.bufLen, p.bufLen);
                    actual = p.bufLen;
                }
                break;
            case USB_TRANSFER_STATUS_CANCELED:  status = -104; break;   // ECONNRESET
            case USB_TRANSFER_STATUS_NO_DEVICE: status = -19;  break;   // ENODEV
            case USB_TRANSFER_STATUS_TIMED_OUT: status = -110; break;   // ETIMEDOUT
            default:                            status = -32;  break;   // EPIPE
        }

        uint8_t rep[48] = {};
        _w32(rep +  0, USBIP_RET_SUBMIT);
        _w32(rep +  4, p.seqnum);
        _w32(rep + 20, (uint32_t)status);
        _w32(rep + 24, (uint32_t)actual);
        _USBIP_TRACE("IN ", p.epAddr, actual, x->data_buffer, status);
        bool ok = _writeAll(rep, 48) && (actual == 0 || _writeAll(x->data_buffer, actual));
        _device->freeTransfer(x);
        p.used = false;
        if (!ok) { _client.stop(); return; }
    }
}

void UsbIpServer::_handleUnlink(uint32_t seqnum, uint32_t victim) {
    int32_t status = 0;                              // 0: URB had already completed
    for (int i = 0; i < MAX_PENDING; i++) {
        PendingUrb& p = _pending[i];
        if (p.used && !p.cancelled && p.seqnum == victim) {
            if (p.ring) {
                // Part-filled data goes back to the front of the endpoint's stream, so
                // cancelling a read never loses bytes.
                RxEp* r = _rxFind(p.epAddr);
                if (r && p.got > 0 && r->carryLen + p.got <= RX_CARRY) {
                    memmove(r->carry + p.got, r->carry, r->carryLen);
                    memcpy(r->carry, p.buf, p.got);
                    r->carryLen += p.got;
                }
                free(p.buf); p.buf = nullptr; p.used = false;
                status = -104;
            } else if (p.done) {
                _drainPending();                     // finished but unsent: deliver it first
            } else {
                _device->cancelEndpoint(p.epAddr);
                status = -104;                       // ECONNRESET: unlinked before completion
                unsigned long deadline = millis() + 300;
                while (!p.done && millis() < deadline) delay(1);
                if (p.done) {
                    // Flushed. Un-halt now: the client usually resubmits a read on this
                    // endpoint the instant it gets RET_UNLINK, and a halted pipe would
                    // reject it (the client would then see a stall).
                    _device->clearEndpoint(p.epAddr);
                    _device->freeTransfer(p.xfer);
                    p.used = false;
                } else {
                    p.cancelled = true;              // cleaned up later in _drainPending()
                }
            }
            break;
        }
    }
    _USBIP_TRACE("UNLINK", 0, 0, nullptr, status);
    uint8_t rep[48] = {};
    _w32(rep +  0, USBIP_RET_UNLINK);
    _w32(rep +  4, seqnum);
    _w32(rep + 20, (uint32_t)status);
    _writeAll(rep, 48);
}

// Cancel everything still queued on the device and release it, so no
// transfer outlives its client.
void UsbIpServer::_cancelAllPending() {
    for (int i = 0; i < MAX_PENDING; i++) {
        PendingUrb& p = _pending[i];
        if (p.used && p.ring) { free(p.buf); p.buf = nullptr; p.used = false; continue; }
        if (p.used && !p.done) _device->cancelEndpoint(p.epAddr);
    }
    unsigned long deadline = millis() + 1500;
    for (;;) {
        bool waiting = false;
        for (int i = 0; i < MAX_PENDING; i++)
            if (_pending[i].used && !_pending[i].done) waiting = true;
        if (!waiting || millis() > deadline) break;
        delay(2);
    }
    for (int i = 0; i < MAX_PENDING; i++) {
        PendingUrb& p = _pending[i];
        if (!p.used) continue;
        if (p.done) { _device->clearEndpoint(p.epAddr); _device->freeTransfer(p.xfer); }
        else _USBIP_LOGE("USB transfer on ep 0x%02X could not be cancelled; leaking it", p.epAddr);
        p.used = false;   // an unfinished transfer is leaked rather than freed under the host stack
    }
}

// ── Bulk IN read-ahead ────────────────────────────────────────────────────────

void UsbIpServer::_rxDone(usb_transfer_t* t) {
    static_cast<RxArm*>(t->context)->done = true;        // runs in the USB host task
}

UsbIpServer::RxEp* UsbIpServer::_rxFind(uint8_t addr) {
    for (int i = 0; i < MAX_RX_EPS; i++)
        if (_rx[i].active && _rx[i].addr == addr) return &_rx[i];
    return nullptr;
}

void UsbIpServer::_rxStart() {
    int n = _device->getEndpointCount();
    for (int i = 0; i < n; i++) {
        uint8_t addr, type; uint16_t mps;
        if (!_device->getEndpointInfo(i, addr, type, mps)) continue;
        if (!(addr & 0x80) || type != USB_TRANSFER_TYPE_BULK || mps == 0 || mps > RX_PKT_MAX) continue;
        if (_rxFind(addr)) continue;
        RxEp* r = nullptr;
        for (int k = 0; k < MAX_RX_EPS; k++) if (!_rx[k].active) { r = &_rx[k]; break; }
        if (!r) break;
        r->ring  = (RxPkt*)malloc(sizeof(RxPkt) * RX_RING);
        r->carry = (uint8_t*)malloc(RX_CARRY);
        if (!r->ring || !r->carry) {
            free(r->ring); free(r->carry); r->ring = nullptr; r->carry = nullptr;
            _USBIP_LOGW("USB read-ahead for ep 0x%02X: out of memory, reading on demand", addr);
            continue;
        }
        r->addr = addr; r->mps = mps; r->head = r->count = 0; r->carryLen = 0;
        r->armed = 0; r->retryAt = 0;
        for (int a = 0; a < RX_ARMS; a++) { r->arm[a].xfer = nullptr; r->arm[a].done = false; }
        r->active = true;
        _rxArm(*r);
        _USBIP_LOGI("USB read-ahead on ep 0x%02X (%d armed)", addr, r->armed);
    }
}

void UsbIpServer::_rxArm(RxEp& r) {
    if (millis() < r.retryAt) return;
    for (int i = 0; i < RX_ARMS; i++) {
        RxArm& a = r.arm[i];
        if (a.xfer) continue;
        if (r.count + r.armed >= RX_RING) return;        // ring full: let the device NAK
        a.done = false;
        a.xfer = _device->beginInTransfer(r.addr, r.mps, _rxDone, &a);
        if (!a.xfer) { r.retryAt = millis() + 200; return; }
        r.armed++;
    }
}

void UsbIpServer::_rxPump() {
    for (int e = 0; e < MAX_RX_EPS; e++) {
        RxEp& r = _rx[e];
        if (!r.active) continue;
        for (int i = 0; i < RX_ARMS; i++) {
            RxArm& a = r.arm[i];
            if (!a.xfer || !a.done) continue;
            usb_transfer_t* x = a.xfer;
            if (x->status == USB_TRANSFER_STATUS_COMPLETED) {
                if (r.count < RX_RING) {
                    int n = x->actual_num_bytes;
                    if (n > r.mps) n = r.mps;
                    RxPkt& p = r.ring[(r.head + r.count) % RX_RING];
                    p.len = (uint8_t)n; p.off = 0;
                    memcpy(p.data, x->data_buffer, n);
                    r.count++;
                }
            } else if (x->status != USB_TRANSFER_STATUS_CANCELED &&
                       x->status != USB_TRANSFER_STATUS_NO_DEVICE) {
                _device->clearEndpoint(r.addr);          // stall or error: un-halt and try again
                r.retryAt = millis() + 50;
            }
            _device->freeTransfer(x);
            a.xfer = nullptr; a.done = false; r.armed--;
        }
        _rxArm(r);
    }
}

// Complete client read URBs from the ring, oldest first, with real USB semantics: a URB
// ends when its buffer is full or a short (or zero-length) packet ends the transfer.
void UsbIpServer::_rxService() {
    for (int e = 0; e < MAX_RX_EPS; e++) {
        RxEp& r = _rx[e];
        if (!r.active) continue;
        for (;;) {
            PendingUrb* s = nullptr;
            for (int i = 0; i < MAX_PENDING; i++) {
                PendingUrb& c = _pending[i];
                if (c.used && c.ring && c.epAddr == r.addr &&
                    (!s || (int32_t)(c.order - s->order) < 0)) s = &c;
            }
            if (!s) break;

            bool term = false;
            if (r.carryLen > 0 && s->got < s->bufLen) {
                int n = min(r.carryLen, (int)(s->bufLen - s->got));
                memcpy(s->buf + s->got, r.carry, n);
                s->got += n;
                memmove(r.carry, r.carry + n, r.carryLen - n);
                r.carryLen -= n;
            }
            while (s->got < s->bufLen && r.carryLen == 0 && r.count > 0) {
                RxPkt& p = r.ring[r.head];
                int n = min((int)(p.len - p.off), (int)(s->bufLen - s->got));
                memcpy(s->buf + s->got, p.data + p.off, n);
                p.off += n; s->got += n;
                if (p.off >= p.len) {
                    bool shortPkt = p.len < r.mps;
                    r.head = (r.head + 1) % RX_RING; r.count--;
                    if (shortPkt) { term = true; break; }
                }
            }
            if (!(s->got >= s->bufLen || term)) break;   // waiting for more data

            uint8_t rep[48] = {};
            _w32(rep +  0, USBIP_RET_SUBMIT);
            _w32(rep +  4, s->seqnum);
            _w32(rep + 24, (uint32_t)s->got);
            _USBIP_TRACE("IN ", s->epAddr, s->got, s->buf, 0);
            bool ok = _writeAll(rep, 48) && (s->got == 0 || _writeAll(s->buf, s->got));
            free(s->buf); s->buf = nullptr; s->used = false;
            if (!ok) { _client.stop(); return; }
        }
        _rxArm(r);                                       // ring space freed: keep the device polled
    }
}

void UsbIpServer::_rxStop() {
    for (int e = 0; e < MAX_RX_EPS; e++) {
        RxEp& r = _rx[e];
        if (!r.active) continue;
        bool any = false;
        for (int i = 0; i < RX_ARMS; i++) if (r.arm[i].xfer && !r.arm[i].done) any = true;
        if (any) _device->cancelEndpoint(r.addr);
        unsigned long deadline = millis() + 1000;
        for (;;) {
            bool waiting = false;
            for (int i = 0; i < RX_ARMS; i++) if (r.arm[i].xfer && !r.arm[i].done) waiting = true;
            if (!waiting || millis() > deadline) break;
            delay(2);
        }
        for (int i = 0; i < RX_ARMS; i++) {
            RxArm& a = r.arm[i];
            if (!a.xfer) continue;
            if (a.done) _device->freeTransfer(a.xfer);
            else _USBIP_LOGE("USB read-ahead transfer on ep 0x%02X could not be cancelled; leaking it", r.addr);
            a.xfer = nullptr; a.done = false;
        }
        _device->clearEndpoint(r.addr);
        free(r.ring); free(r.carry);
        r.ring = nullptr; r.carry = nullptr; r.armed = 0; r.count = 0; r.carryLen = 0;
        r.active = false;
    }
}

// ── Status JSON ───────────────────────────────────────────────────────────────

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
