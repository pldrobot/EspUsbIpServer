#pragma once
// Internal logging macros — not part of the public API.
// Set USBIP_LOG_LEVEL before including esp_usbip_server.h to control verbosity:
//   0 = silent, 1 = errors only, 2 = errors + warnings, 3 = lifecycle info (default),
//   4 = debug (enumeration detail, endpoints, host events)
//
// Optional: the application may define
//     extern "C" void espusbip_log_hook(char level, const char* line);
// to receive every log line (e.g. to keep a persistent log). It is a weak
// symbol, so nothing happens if the application does not define it.

#include <Arduino.h>
#include <stdarg.h>

extern "C" void espusbip_log_hook(char level, const char* line) __attribute__((weak));

inline void _usbipLog(char level, const char* fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.printf("[USBIP %c] %s\n", level, buf);
    if (espusbip_log_hook) espusbip_log_hook(level, buf);
}

#ifndef USBIP_LOG_LEVEL
#define USBIP_LOG_LEVEL 3
#endif

#if USBIP_LOG_LEVEL >= 3
#define _USBIP_LOGI(fmt, ...) _usbipLog('I', fmt, ##__VA_ARGS__)
#else
#define _USBIP_LOGI(...) ((void)0)
#endif

#if USBIP_LOG_LEVEL >= 4
#define _USBIP_LOGD(fmt, ...) _usbipLog('D', fmt, ##__VA_ARGS__)
#else
#define _USBIP_LOGD(...) ((void)0)
#endif

#if USBIP_LOG_LEVEL >= 2
#define _USBIP_LOGW(fmt, ...) _usbipLog('W', fmt, ##__VA_ARGS__)
#else
#define _USBIP_LOGW(...) ((void)0)
#endif

#if USBIP_LOG_LEVEL >= 1
#define _USBIP_LOGE(fmt, ...) _usbipLog('E', fmt, ##__VA_ARGS__)
#else
#define _USBIP_LOGE(...) ((void)0)
#endif

// Optional URB tracing for diagnosing a device that will not talk: define
// USBIP_TRACE=1 (e.g. -DUSBIP_TRACE=1). Logs the first USBIP_TRACE_MAX
// forwarded URBs - control setup packets and the leading bytes of every
// transfer - then goes quiet so it cannot flood the log.
#ifndef USBIP_TRACE
#define USBIP_TRACE 0
#endif
#ifndef USBIP_TRACE_MAX
#define USBIP_TRACE_MAX 300
#endif
#if USBIP_TRACE
inline void _usbipTrace(const char* what, uint8_t ep, int len, const uint8_t* d, int status) {
    static int n = 0;
    if (n >= USBIP_TRACE_MAX) return;
    if (++n == USBIP_TRACE_MAX) { _usbipLog('W', "trace limit reached, tracing stopped"); return; }
    char hex[40] = {0};
    int show = (d && len > 0) ? (len < 12 ? len : 12) : 0;
    for (int i = 0; i < show; i++) snprintf(hex + i * 3, sizeof(hex) - i * 3, "%02X ", d[i]);
    _usbipLog('I', "T %s ep=%02X len=%d st=%d %s", what, ep, len, status, hex);
}
#define _USBIP_TRACE(...) _usbipTrace(__VA_ARGS__)
#else
#define _USBIP_TRACE(...) ((void)0)
#endif
