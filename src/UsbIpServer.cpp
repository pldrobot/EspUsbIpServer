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
    _w32(out + 296, USBIP_SPEED_HIGH);
    _w16(out + 300, _device->getVendorId());
    _w16(out + 302, _device->getProductId());
    _w16(out + 304, 0x0100);
    out[306] = 0; out[307] = 0; out[308] = 0;
    out[309] = 1; out[310] = 1; out[311] = 1;
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
        if (_client.available() >= 48) _handleUrbs();
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
        // Interface descriptor: class=0x07 (printer), subclass=0x01, protocol=0x02, reserved=0x00
        uint8_t iface[4] = { 0x07, 0x01, 0x02, 0x00 };
        _writeAll(iface, 4);
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
        uint8_t rep[48] = {};
        _w32(rep +  0, USBIP_RET_UNLINK);
        _w32(rep +  4, seqnum);
        _w32(rep + 20, (uint32_t)(-104));
        _writeAll(rep, 48);
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
    } else if (direction == USBIP_DIR_OUT) {
        if (bufLen > 0 && !_readExact(buf, bufLen)) { free(buf); return false; }
        actual = _device->submitBulkOut(buf, bufLen, 8000);
        if (actual < 0) { status = -32; actual = 0; }
    } else {
        if (bufLen > 0) {
            actual = _device->submitBulkIn(buf, bufLen, 5000);
            if (actual < 0) { status = -32; actual = 0; }
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
