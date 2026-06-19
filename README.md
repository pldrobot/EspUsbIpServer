# esp_usbip_server

Arduino library for ESP32-S3 that exposes a connected USB device over the network using the [USB/IP protocol](https://www.kernel.org/doc/html/latest/usb/usbip_protocol.html) (TCP port 3240).

Handles USB Host enumeration, bulk and control transfer forwarding, and the full USB/IP handshake. `begin()` spawns an internal FreeRTOS task — no manual polling in `loop()` required.

---

## Hardware requirements

- **ESP32-S3** with an exposed USB D+/D− pair wired for Host mode (not the UART/JTAG USB)
- A USB device plugged into the Host port (currently auto-enumerates USB printers, class `0x07`)

---

## PlatformIO setup

### 1. `platformio.ini` build flags

USB Host mode requires disabling the default CDC-on-boot configuration:

```ini
build_flags =
    -DBOARD_HAS_PSRAM
    -DARDUINO_USB_MODE=0          ; enable USB Host mode
    -DARDUINO_USB_CDC_ON_BOOT=0   ; free the USB peripheral

build_unflags =
    -DARDUINO_USB_MODE
    -DARDUINO_USB_CDC_ON_BOOT
```

### 2. Include

```cpp
#include <EspUsbIpServer.h>
```

---

## Quick start

```cpp
#include <WiFi.h>
#include <EspUsbIpServer.h>

UsbDevice   usbDevice;
UsbIpServer usbipServer;

// USB Host must run on Core 1
void usbHostTask(void*) {
    usbDevice.begin();
    for (;;) {
        usbDevice.task();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);

    xTaskCreatePinnedToCore(usbHostTask, "usb_host", 4096, nullptr, 5, nullptr, 1);

    WiFi.begin("SSID", "password");
    while (WiFi.status() != WL_CONNECTED) delay(500);

    // Spawns the server task pinned to this core (same as loop())
    usbipServer.begin(&usbDevice);

    usbipServer.onClientConnect([](const char* ip) {
        Serial.printf("Client attached: %s\n", ip);
    });
    usbipServer.onClientDisconnect([](const char* ip, uint32_t duration) {
        Serial.printf("Client detached: %s (%us)\n", ip, duration);
    });
}

void loop() {
    delay(1000);  // server runs in its own task
}
```

Attach from a Linux host:
```bash
usbip attach -r <ESP32_IP> -b 1-1
```

Windows clients use [usbipd-win](https://github.com/dorssel/usbipd-win).

---

## API

### `UsbDevice`

| Method | Description |
|--------|-------------|
| `begin()` | Install USB Host driver and register client. Call once from the Core 1 task before looping. |
| `task()` | Drive USB Host events. Call every ~10 ms from the Core 1 task. |
| `isConnected()` | `true` once a device has been enumerated. |
| `isReady()` | `true` when the device is enumerated and not locked. |
| `sendData(data, len)` | Bulk OUT — rejected while USB/IP holds the lock. |
| `readData(buf, maxLen, timeout)` | Bulk IN. |
| `sendMaintCommand(cmd, len, resp, respLen, timeout)` | Send a command and optionally read back a response. |
| `softReset()` | Send USB printer soft-reset control request. |
| `setMaintenanceMode(bool)` | Acquire/release the exclusive USB/IP lock (managed automatically by `UsbIpServer`). |
| `getVendorId()` / `getProductId()` | USB VID/PID of the connected device. |
| `getDevDescRaw()` / `getCfgDescRaw()` | Raw descriptor bytes forwarded to the USB/IP client. |
| `submitControlTransfer(setup, data, len, timeout)` | Generic control transfer (used by `UsbIpServer`). |
| `submitBulkOut(data, len, timeout)` | Generic bulk OUT (used by `UsbIpServer`). |
| `submitBulkIn(buf, maxLen, timeout)` | Generic bulk IN (used by `UsbIpServer`). |

### `UsbIpServer`

| Method | Description |
|--------|-------------|
| `begin(UsbDevice*)` | Start TCP server on port 3240 and spawn the server task pinned to the caller's core. |
| `stop()` | Suspend the server task and kick any active client. |
| `start()` | Resume the server task after `stop()`. |
| `onClientConnect(cb)` | Register `void(const char* clientIP)` — fired when a client imports the device. |
| `onClientDisconnect(cb)` | Register `void(const char* clientIP, uint32_t durationSec)` — fired on disconnect. |
| `isClientAttached()` | `true` while a USB/IP client has imported the device. |
| `kickClient()` | Force-disconnect the current client. |
| `statusJson()` | JSON blob: attached state, current client IP/durations, session history (last 6). |

---

## Logging

Log output goes to `Serial` at the verbosity level set by `USBIP_LOG_LEVEL`:

| Value | Output |
|-------|--------|
| `0` | Silent |
| `1` | Errors only |
| `2` | Errors + warnings |
| `3` | All (default) |

Set before including the header:

```cpp
#define USBIP_LOG_LEVEL 1
#include <EspUsbIpServer.h>
```

---

## Architecture notes

- **Dual-core**: `UsbDevice::task()` runs on **Core 1** in a pinned FreeRTOS task. `UsbIpServer` spawns its own task on whichever core `begin()` is called from (typically Core 1 alongside `loop()`).
- **Single session**: a new incoming connection automatically kicks the existing client.
- **Maintenance lock**: `UsbIpServer` acquires `setMaintenanceMode(true)` on attach and releases it on disconnect, blocking normal `sendData()` calls while a remote client is active.
- **SET_CONFIGURATION / SET_INTERFACE** control requests are intercepted and acknowledged locally — the device is already configured by the ESP-IDF USB Host driver.

---

## License

MIT — see [LICENSE](LICENSE).
