#pragma once
// USB/IP wire format (https://www.kernel.org/doc/html/latest/usb/usbip_protocol.html).
//
// Every multi-byte field on the wire is big-endian, except the 8-byte USB setup
// packet, which is copied through untouched (little-endian, as USB defines it).
// Structs hold the fields in network order; always go through be16()/be32().
// Never address these buffers with hand-computed offsets.

#include <stdint.h>

namespace usbipproto {

static inline uint32_t be32(uint32_t v) { return __builtin_bswap32(v); }
static inline uint16_t be16(uint16_t v) { return __builtin_bswap16(v); }

constexpr uint16_t VERSION = 0x0111;

constexpr uint16_t OP_REQ_DEVLIST = 0x8005;
constexpr uint16_t OP_REP_DEVLIST = 0x0005;
constexpr uint16_t OP_REQ_IMPORT  = 0x8003;
constexpr uint16_t OP_REP_IMPORT  = 0x0003;

constexpr uint32_t OP_STATUS_OK     = 0;
constexpr uint32_t OP_STATUS_NA     = 1;
constexpr uint32_t OP_STATUS_NO_DEV = 4;

constexpr uint32_t CMD_SUBMIT = 1;
constexpr uint32_t CMD_UNLINK = 2;
constexpr uint32_t RET_SUBMIT = 3;
constexpr uint32_t RET_UNLINK = 4;

constexpr uint32_t DIR_OUT = 0;
constexpr uint32_t DIR_IN  = 1;

constexpr uint32_t SPEED_LOW  = 1;
constexpr uint32_t SPEED_FULL = 2;

constexpr uint32_t NOT_ISO = 0xFFFFFFFFu;   // number_of_packets of a non-isochronous URB
constexpr uint32_t ISO_DESC_SIZE = 16;
constexpr uint32_t MAX_ISO_PACKETS = 1024;
constexpr uint32_t URB_ZERO_PACKET = 0x40;  // transfer_flags bit: terminate OUT with a ZLP

// errno values carried in the status field (negative on the wire)
constexpr int32_t ST_OK         = 0;
constexpr int32_t ST_ENOMEM     = -12;
constexpr int32_t ST_ENODEV     = -19;
constexpr int32_t ST_EPIPE      = -32;
constexpr int32_t ST_ECONNRESET = -104;
constexpr int32_t ST_ETIMEDOUT  = -110;

struct __attribute__((packed)) OpHeader {          //  8 bytes
    uint16_t version;
    uint16_t code;
    uint32_t status;
};

struct __attribute__((packed)) DeviceRecord {      // 312 bytes
    char     path[256];
    char     busid[32];
    uint32_t busnum;
    uint32_t devnum;
    uint32_t speed;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bConfigurationValue;
    uint8_t  bNumConfigurations;
    uint8_t  bNumInterfaces;
};

struct __attribute__((packed)) InterfaceRecord {   //   4 bytes
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t padding;
};

// All four URB messages share a 48-byte header: 20 common bytes + a 28-byte body.
struct __attribute__((packed)) UrbHeader {         //  48 bytes
    uint32_t command;
    uint32_t seqnum;
    uint32_t devid;
    uint32_t direction;
    uint32_t ep;
    union __attribute__((packed)) {
        struct __attribute__((packed)) {
            uint32_t transfer_flags;
            uint32_t transfer_buffer_length;
            uint32_t start_frame;
            uint32_t number_of_packets;
            uint32_t interval;
            uint8_t  setup[8];
        } submit;
        struct __attribute__((packed)) {
            uint32_t unlink_seqnum;
            uint8_t  padding[24];
        } unlink;
        struct __attribute__((packed)) {
            uint32_t status;
            uint32_t actual_length;
            uint32_t start_frame;
            uint32_t number_of_packets;
            uint32_t error_count;
            uint8_t  padding[8];
        } ret_submit;
        struct __attribute__((packed)) {
            uint32_t status;
            uint8_t  padding[24];
        } ret_unlink;
    } u;
};

static_assert(sizeof(OpHeader) == 8,          "OpHeader must be 8 bytes");
static_assert(sizeof(DeviceRecord) == 312,    "DeviceRecord must be 312 bytes");
static_assert(sizeof(InterfaceRecord) == 4,   "InterfaceRecord must be 4 bytes");
static_assert(sizeof(UrbHeader) == 48,        "UrbHeader must be 48 bytes");

static inline void makeRetSubmit(UrbHeader& h, uint32_t seqnum, int32_t status, uint32_t actual) {
    __builtin_memset(&h, 0, sizeof(h));
    h.command = be32(RET_SUBMIT);
    h.seqnum  = be32(seqnum);
    h.u.ret_submit.status        = be32((uint32_t)status);
    h.u.ret_submit.actual_length = be32(actual);
}

static inline void makeRetUnlink(UrbHeader& h, uint32_t seqnum, int32_t status) {
    __builtin_memset(&h, 0, sizeof(h));
    h.command = be32(RET_UNLINK);
    h.seqnum  = be32(seqnum);
    h.u.ret_unlink.status = be32((uint32_t)status);
}

}  // namespace usbipproto
