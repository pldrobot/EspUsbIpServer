#pragma once
// Internal logging macros — not part of the public API.
// Set USBIP_LOG_LEVEL before including esp_usbip_server.h to control verbosity:
//   0 = silent, 1 = errors only, 2 = errors + warnings, 3 = all (default)

#ifndef USBIP_LOG_LEVEL
#define USBIP_LOG_LEVEL 3
#endif

#if USBIP_LOG_LEVEL >= 3
#define _USBIP_LOGI(fmt, ...) Serial.printf("[USBIP I] " fmt "\n", ##__VA_ARGS__)
#else
#define _USBIP_LOGI(...) ((void)0)
#endif

#if USBIP_LOG_LEVEL >= 2
#define _USBIP_LOGW(fmt, ...) Serial.printf("[USBIP W] " fmt "\n", ##__VA_ARGS__)
#else
#define _USBIP_LOGW(...) ((void)0)
#endif

#if USBIP_LOG_LEVEL >= 1
#define _USBIP_LOGE(fmt, ...) Serial.printf("[USBIP E] " fmt "\n", ##__VA_ARGS__)
#else
#define _USBIP_LOGE(...) ((void)0)
#endif
