#pragma once
// Internal logging macros — not part of the public API.
// Set USBIP_LOG_LEVEL before including esp_usbip_server.h to control verbosity:
//   0 = silent, 1 = errors only, 2 = errors + warnings, 3 = all (default)
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
