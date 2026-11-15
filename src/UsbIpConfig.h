#pragma once
// Build-time tunables. Every value can be overridden with a -D flag.

// ── Tasks ────────────────────────────────────────────────────────────────────
// Core used by every task this library creates (USB host, accept, session, sender).
// Core 1, as in usbipdcpp_esp32: the USB host ISR is installed from the Arduino loop task
// (core 1), and this keeps a fast URB stream away from the WiFi stack on core 0.
#ifndef USBIP_CORE
#define USBIP_CORE 1
#endif
#ifndef USBIP_PRIO_USB
#define USBIP_PRIO_USB 5           // usb_host: library + client events + completion callbacks
#endif
#ifndef USBIP_PRIO_NET
#define USBIP_PRIO_NET 8           // accept, session, sender
#endif
#ifndef USBIP_STACK_USB
#define USBIP_STACK_USB 8192          // enumeration and every completion callback run here
#endif
#ifndef USBIP_USB_POLL_MS
#define USBIP_USB_POLL_MS 2           // longest the host library waits to be serviced
#endif
#ifndef USBIP_STACK_ACCEPT
#define USBIP_STACK_ACCEPT 3072
#endif
#ifndef USBIP_STACK_SESSION
#define USBIP_STACK_SESSION 8192
#endif
#ifndef USBIP_STACK_SENDER
#define USBIP_STACK_SENDER 4096
#endif

// ── Transfer pool ────────────────────────────────────────────────────────────
// Transfers are allocated once at begin() (the ESP32-S3 can only DMA from internal
// RAM, about 300 KB) and recycled. A URB takes the smallest free slot that fits.
// Control transfers need 8 extra bytes for the setup packet.
#ifndef USBIP_POOL0_SIZE
#define USBIP_POOL0_SIZE   136     // small reads/writes, control with <=128 B of data
#endif
#ifndef USBIP_POOL0_COUNT
#define USBIP_POOL0_COUNT  24      // cdc-acm alone keeps 16 read URBs pending
#endif
#ifndef USBIP_POOL1_SIZE
#define USBIP_POOL1_SIZE   520
#endif
#ifndef USBIP_POOL1_COUNT
#define USBIP_POOL1_COUNT  8
#endif
#ifndef USBIP_POOL2_SIZE
#define USBIP_POOL2_SIZE   8200    // largest single transfer (8 KB data)
#endif
#ifndef USBIP_POOL2_COUNT
#define USBIP_POOL2_COUNT  3
#endif
#define USBIP_MAX_URBS (USBIP_POOL0_COUNT + USBIP_POOL1_COUNT + USBIP_POOL2_COUNT)

// Extra room in the response queue for RET_UNLINK / locally answered requests.
#ifndef USBIP_UNLINK_SLOTS
#define USBIP_UNLINK_SLOTS 16
#endif
#define USBIP_RESP_QUEUE (USBIP_MAX_URBS + USBIP_UNLINK_SLOTS)

// A bulk OUT longer than one pool slot is sent in slot-sized chunks. Anything above
// this is refused (the client sent a bogus length).
#ifndef USBIP_MAX_URB_LEN
#define USBIP_MAX_URB_LEN (1024 * 1024)
#endif

// ── Bulk IN read-ahead ───────────────────────────────────────────────────────
// A real host polls an IN endpoint continuously; waiting for the client to send
// the next read URB lets a streaming device (a CH340 at 115200 baud) overflow its
// FIFO. The ESP therefore keeps bulk IN endpoints armed and buffers the data.
// Read-ahead copies every packet twice and costs one callback per 64-byte packet, so it
// is used only for streaming, serial-like endpoints (vendor-specific, CDC control, CDC
// data), where the device has a tiny FIFO and loses data if nobody polls. Other classes
// (mass storage, bulk-only devices, ...) read straight into the client URB's DMA buffer, as
// usbipdcpp does: one transfer of up to a full pool slot, no copy. Set to 1 to use
// read-ahead on every bulk IN endpoint.
#ifndef USBIP_READAHEAD_ALL
#define USBIP_READAHEAD_ALL 0
#endif
#ifndef USBIP_RX_PKT_MAX
#define USBIP_RX_PKT_MAX 64        // largest full-speed bulk packet
#endif
#ifndef USBIP_RX_RING
#define USBIP_RX_RING 128          // packets buffered per endpoint
#endif
#ifndef USBIP_RX_ARMS
#define USBIP_RX_ARMS 12           // single-packet transfers kept queued
#endif
#ifndef USBIP_RX_CARRY
#define USBIP_RX_CARRY 4096        // bytes handed back by an unlinked, part-filled URB
#endif
#ifndef USBIP_MAX_RX_EPS
#define USBIP_MAX_RX_EPS 2
#endif

// ── Diagnostics ──────────────────────────────────────────────────────────────
// 1 = log the URB rate every 5 s while a client is attached (also fills the small crash log).
#ifndef USBIP_STATS
#define USBIP_STATS 0
#endif

// ── Timeouts (ms) ────────────────────────────────────────────────────────────
#ifndef USBIP_HANDSHAKE_TIMEOUT_MS
#define USBIP_HANDSHAKE_TIMEOUT_MS 10000   // client must finish OP_REQ_IMPORT in time
#endif
#ifndef USBIP_PAYLOAD_TIMEOUT_MS
#define USBIP_PAYLOAD_TIMEOUT_MS   5000    // rest of a URB after its header arrived
#endif
#ifndef USBIP_SOCK_TICK_MS
#define USBIP_SOCK_TICK_MS         250     // recv wakes this often to check abort flags
#endif
#ifndef USBIP_SEND_TIMEOUT_MS
#define USBIP_SEND_TIMEOUT_MS      5000
#endif
#ifndef USBIP_SLOT_WAIT_MS
#define USBIP_SLOT_WAIT_MS         200     // wait for a free pool slot before ENOMEM
#endif
#ifndef USBIP_CANCEL_WAIT_MS
#define USBIP_CANCEL_WAIT_MS       300     // endpoint flush after UNLINK
#endif
#ifndef USBIP_CHUNK_TIMEOUT_MS
#define USBIP_CHUNK_TIMEOUT_MS     8000
#endif
#ifndef USBIP_TEARDOWN_WAIT_MS
#define USBIP_TEARDOWN_WAIT_MS     2000
#endif
#ifndef USBIP_CLOSE_RETRY_MS
#define USBIP_CLOSE_RETRY_MS       3000    // how long to wait for the host stack to free a gone device
#endif

// ── Descriptor storage ───────────────────────────────────────────────────────
#define USBIP_DEV_DESC_SIZE   18
#define USBIP_CFG_DESC_SIZE   512   // copy kept for the config value; enumeration walks the live descriptor
#define USBIP_MAX_IFACES      8
#define USBIP_MAX_EPS         16
