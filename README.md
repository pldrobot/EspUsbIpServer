# EspUsbIpServer

Arduino library for ESP32-S3 that exposes one connected USB device over the network using the [USB/IP protocol](https://www.kernel.org/doc/html/latest/usb/usbip_protocol.html) (TCP port 3240).

It handles USB Host enumeration, control/bulk/interrupt transfer forwarding, and the full USB/IP handshake. `begin()` starts internal FreeRTOS tasks, so nothing needs polling in `loop()`.

---

## Hardware requirements

- **ESP32-S3** with an exposed USB D+/D− pair wired for Host mode (not the UART/JTAG USB)
- Any Low-/Full-Speed (12 Mbps) USB device plugged into the Host port: printers, CDC-ACM serial devices, CH340/FTDI-style serial adapters, HID devices, and so on. Every interface (alternate setting 0) is claimed and every bulk or interrupt endpoint on it is forwarded.
- One device and one client at a time. Hubs, isochronous endpoints (audio, webcams), alternate settings above 0 and High Speed are out of scope. The ESP32-S3 USB-OTG has no High-Speed PHY anyway.

---

## PlatformIO setup

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

Tested with Arduino core 3.x (platform `espressif32@8.0.0`).

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

void setup() {
    Serial.begin(115200);

    usbDevice.begin();                       // starts the USB host tasks

    WiFi.begin("SSID", "password");
    while (WiFi.status() != WL_CONNECTED) delay(500);

    usbipServer.begin(&usbDevice);           // starts the USB/IP tasks, port 3240

    usbipServer.onClientConnect([](const char* ip) {
        Serial.printf("Client attached: %s\n", ip);
    });
    usbipServer.onClientDisconnect([](const char* ip, uint32_t duration) {
        Serial.printf("Client detached: %s (%us)\n", ip, duration);
    });
}

void loop() {
    delay(1000);  // everything runs in its own tasks
}
```

---

## Connecting a client

### USBIP-Client (recommended)

[**USBIP-Client**](https://github.com/pldrobot/USBIP-Client) is a small cross-platform system-tray app
(Rust) built for this server. Right-click the tray icon to see the servers found on the LAN, check one to
attach and uncheck it to detach, with **Refresh** and **Exit** at the bottom.

- **Discovery:** it scans the local /24 for port 3240 (plus hosts listed in `servers.txt`), then asks each
  server for its device list.
- **Attach:** on Windows it drives `usbip.exe` from a bundled usbip-win2 install; on Linux it uses the kernel
  `vhci_hcd` sysfs interface directly.
- **While a device is attached,** its Refresh and scan open extra connections to the server. This library
  answers them instead of dropping them: `DEVLIST` returns an empty list (the device is in use) and `IMPORT`
  is refused with status `NA`, so the client's list stays clean.

### Other USB/IP clients

Any standard USB/IP client works; the tray app just automates discovery and attach.

- **Linux:** the kernel client from `usbip` / `linux-tools`:
  ```bash
  usbip attach -r <ESP32_IP> -b 1-1
  ```
- **Windows:** [usbip-win2](https://github.com/vadimgrn/usbip-win2) (`usbip.exe attach -r <ESP32_IP> -b 1-1`).

---

## API

### `UsbDevice`

| Method | Description |
|--------|-------------|
| `begin()` | Install the USB host and start its task (`usb_host`). |
| `end()` | Close the device, stop the tasks, uninstall the host. |
| `stop()` / `start()` | Suspend / resume the host tasks. |
| `isConnected()` / `isReady()` | A device is enumerated / enumerated and not held by a client. |
| `getManufacturer()` / `getProduct()` / `getSerial()` | The device's real string descriptors (`VID:0x..` / `PID:0x..` if it has none). |
| `getVendorId()` / `getProductId()` / `getSpeed()` | VID, PID and negotiated speed (`USB_SPEED_LOW` / `USB_SPEED_FULL`). |
| `getIfaceCount()` / `getIfaceDesc(i, out[4])` | Every claimed interface with its real class triple. |
| `getEndpointCount()` / `findEndpoint(addr)` | The claimed bulk and interrupt endpoints. |
| `getDevDescRaw()` / `getCfgDescRaw()` | Raw descriptors forwarded to the client. |
| `setMaintenanceMode(bool)` | Exclusive client lock (managed by `UsbIpServer`). |
| `onDeviceGone(cb)` | Called on the `usb_host` task the moment the device is unplugged. |
| `submitTransfer(xfer)` / `submitControl(xfer)` | Submit a transfer (used by `UsbIpServer`). |
| `cancelEndpoint(ep)` / `clearEndpoint(ep)` | Halt+flush / un-halt an endpoint. |

### `UsbIpServer`

| Method | Description |
|--------|-------------|
| `begin(UsbDevice*)` | Preallocate the transfer pool, listen on port 3240, start the tasks. |
| `stop()` / `start()` | Refuse new clients and drop the current one / accept clients again. |
| `onClientConnect(cb)` | `void(const char* clientIP)`, fired when a client imports the device. |
| `onClientDisconnect(cb)` | `void(const char* clientIP, uint32_t durationSec)`, fired on disconnect. |
| `isClientAttached()` | `true` while a client has imported the device. |
| `kickClient()` | Force-disconnect the current client. |
| `statusJson()` | JSON: attached state, current client, last 6 sessions. |

Earlier prototypes also had printer helpers (`sendData`, `readData`, `sendMaintCommand`, `softReset`, `getDeviceId`), blocking `submitControlTransfer` / `submitBulk*` calls and `UsbDevice::task()`. They are not part of the 1.0 API.

---

## Configuration

All tunables are in [`src/UsbIpConfig.h`](src/UsbIpConfig.h) and can be overridden with `-D` flags: task core, priorities and stack sizes, the transfer pool (`USBIP_POOL*_SIZE` / `_COUNT`), the read-ahead ring, and every timeout.

Log verbosity is `USBIP_LOG_LEVEL` (`0` silent, `1` errors, `2` plus warnings, `3` lifecycle info, default; `4` adds enumeration/endpoint detail). `USBIP_STATS=1` also logs the URB rate every 5 s while a client is attached (off by default; it would fill the small crash log). Define `USBIP_TRACE=1` to trace the first URBs.

---

## Architecture

```
usb_host    pumps the USB host library and the client: enumeration, device events,
            and every transfer completion callback
usbip_acc   accepts TCP connections; a second client is closed immediately
usbip_sess  handshake, then reads CMD_SUBMIT / CMD_UNLINK and submits transfers
usbip_tx    writes queued responses to the socket, keeps read-ahead armed
```

- **Asynchronous.** `usbip_sess` submits a URB and moves on; the completion callback (on `usb_host`) only queues the response. Nothing waits on one slow URB while others are ready. The one exception is a bulk OUT larger than a pool slot, which is sent in slot-sized pieces.
- **Zero copy, no malloc per URB.** Transfers come from a pool allocated in `begin()` (the ESP32-S3 can only DMA from internal RAM). An OUT payload is read from the socket straight into the transfer buffer; an IN result is written from it.
- **Ordering.** One response FIFO carries `RET_SUBMIT` and `RET_UNLINK`, so wire order equals completion order. `CMD_UNLINK` of a pending URB cancels its endpoint, answers `RET_UNLINK(-ECONNRESET)` and never sends a `RET_SUBMIT`; other URBs caught by the flush are resubmitted in order.
- **Zero copy, as in usbipdcpp.** An OUT payload is read from the socket straight into the DMA transfer buffer and an IN result is sent straight from it. Bulk and interrupt IN read into the client URB's buffer (up to 8 KB per transfer) with one callback per URB. A bulk OUT larger than a pool slot is sent in pieces through two alternating buffers, so the next piece is received while the previous one is on the wire.
- **Bulk IN read-ahead (streaming endpoints only).** Vendor-specific, CDC-control and CDC-data bulk IN endpoints (packet size 64 or less) are kept armed and buffered, so a serial adapter (a CH340 at 115200 baud) does not overflow its FIFO while the network round trip for the next read is in flight. This path copies each packet twice; every other class uses the direct path above. `USBIP_READAHEAD_ALL=1` forces read-ahead everywhere.
- **Control requests answered locally.** `SET_CONFIGURATION`, `SET_INTERFACE` with alt 0 (other alternates fail with `EPIPE`) and the port reset. `CLEAR_HALT` is forwarded and the host pipe un-halted.
- **Unplug and disconnect.** The device is closed from `usb_host` in small steps between event batches, because the host stack releases it only after the completions of its queued transfers, which are delivered on that same task. The client socket is shut down at once; the session then cancels what the USB host still owns, waits for it (bounded), and only then recycles the pool.
- **Extra clients while the device is held.** They are answered, not dropped (as in usbipdcpp): `DEVLIST` returns an empty list because the device is in use, and `IMPORT` is refused with status `NA`. A client that polls the server while attached therefore gets a clean reply.
- **Generic.** `DEVLIST` / `IMPORT` report the device's real speed, descriptors and class triples, so the client's own native driver binds (CDC-ACM, CH341, FTDI, usblp, HID, ...).

---

## Design reference

The engine follows the design of **usbipdcpp** ([yunsmall/usbipdcpp](https://github.com/yunsmall/usbipdcpp))
and its ESP-IDF port ([yunsmall/usbipdcpp_esp32](https://github.com/yunsmall/usbipdcpp_esp32)):
a per-device handler that submits one `usb_transfer_t` per URB, completion callbacks that only queue
a response, a single FIFO for `RET_SUBMIT` / `RET_UNLINK`, the four `CMD_UNLINK` cases, and the
wire structs with explicit byte-order helpers. This library is an independent Arduino
implementation of that design (no asio, spdlog or C++20), not a copy of its source. usbipdcpp is
LGPL-3.0; check its terms if you reuse its code.

Differences on purpose: a read-ahead ring for serial-style bulk IN endpoints (`USBIP_READAHEAD_ALL`),
pipelined two-buffer chunking for bulk OUT larger than a pool slot (usbipdcpp has chunking disabled),
and bulk IN lengths rounded down to the packet size (usbipdcpp rounds up).

## License

MIT — see [LICENSE](LICENSE).
