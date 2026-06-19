#pragma once
// Main include for the esp_usbip_server library.
//
// Required build flags (platformio.ini):
//   -DARDUINO_USB_MODE=0          ; enables USB Host mode on ESP32-S3
//   -DARDUINO_USB_CDC_ON_BOOT=0   ; frees the USB peripheral for Host use
//
// Optional — set before including this header to control log verbosity:
//   #define USBIP_LOG_LEVEL 0   // silent
//   #define USBIP_LOG_LEVEL 1   // errors only
//   #define USBIP_LOG_LEVEL 2   // errors + warnings
//   #define USBIP_LOG_LEVEL 3   // all (default)

#include "UsbDevice.h"
#include "UsbIpServer.h"
