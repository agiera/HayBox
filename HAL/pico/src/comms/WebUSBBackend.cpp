// WebUSB "single-port GameCube adapter" backend for HayBox on RP2040.
//
// Entered by holding Y at plugin (see config setup()). It reconfigures the
// Adafruit TinyUSB composite device to advertise WebUSB (VID 0x057E /
// PID 0x0337, vendor interface class 0xFF) so the existing browser metadata
// host (gcc-metadata-interface) works unchanged. The host claims interface 1
// and drives two vendor control transfers:
//   OUT request 10: [0x02, port, cmd_len(LE16), resp_len(LE16), ...cmd bytes]
//   IN  request 11: [0x02, port, resp_len(LE16), ...response bytes]
// Only the metadata opcodes are serviced (0xA0 read chunk, 0xB0 write chunk);
// this device *is* the controller, so commands are handled in-firmware rather
// than forwarded onto a physical Joybus line.
//
// USB integration notes:
//  - The device/configuration descriptors are produced by Adafruit_USBD_Device
//    (we drive it via setID/setVersion/addInterface), so we cannot fully own
//    them like the raw-pico-sdk firmware does.
//  - The BOS descriptor, the app class driver, and device-level vendor control
//    requests are provided through TinyUSB's weak callbacks. The XInput library
//    also defines those weak callbacks, so the *strong* overrides live in the
//    dependency-free C file webusb_usb_hooks.c and forward to the helpers
//    below. Those helpers dispatch between the WebUSB and XInput personalities
//    on g_webusb_mode.

#include "comms/WebUSBBackend.hpp"

#include "comms/GcMetadataStore.hpp"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <gamecube_definitions.h>

#include "device/usbd_pvt.h"
#include "tusb.h"

#include <string.h>

// Suggested landing page shown by Chrome when the device is connected.
#define WEBUSB_LANDING_URL "gcc.dizzyforpresident.org"

bool g_webusb_mode = false;

//--------------------------------------------------------------------+
// Vendor request codes (must match the browser host)
//--------------------------------------------------------------------+
enum {
    VENDOR_REQUEST_WEBUSB = 1,
    VENDOR_REQUEST_MICROSOFT = 2,
    VENDOR_REQUEST_WEBUSB_CMD = 10,
    VENDOR_REQUEST_WEBUSB_RESP = 11,
};

// XInput's own MS OS 2.0 vendor request code (bRequest it advertises in its BOS).
#define XINPUT_VENDOR_REQUEST_MICROSOFT 1

//--------------------------------------------------------------------+
// WebUSB command protocol
//--------------------------------------------------------------------+
#define WEBUSB_CMD_JOYBUS_CMD 0x02
#define WEBUSB_RESP_HEADER_LEN 4
#define WEBUSB_RAW_PAYLOAD_MAX 256
#define WEBUSB_CTRL_BUF_SIZE (6 + WEBUSB_RAW_PAYLOAD_MAX)

//--------------------------------------------------------------------+
// WUP-028 GameCube adapter report channel (spoken by Dolphin over libusb).
// The device enumerates as the official adapter (VID 0x057E / PID 0x0337), so
// interface 0 carries two interrupt endpoints: IN reports controller state,
// OUT receives the host's start (0x13) / rumble (0x11) commands. Only the
// first of the adapter's four ports is populated (this controller).
//--------------------------------------------------------------------+
#define ADAPTER_EP_IN 0x81
#define ADAPTER_EP_OUT 0x02
#define ADAPTER_EP_IN_SIZE 37
#define ADAPTER_EP_OUT_SIZE 64
#define ADAPTER_REPORT_LEN 37 // 0x21 header + 4 ports * 9 bytes

//--------------------------------------------------------------------+
// Metadata store: flash persistence is shared with GamecubeBackend via
// GcMetadataStore so both personalities use the same on-flash record.
//--------------------------------------------------------------------+

// In-RAM working copy of the metadata, plus the currently advertised chunk
// count. While a multi-chunk write is in flight the count is held at 0 so the
// host keeps its just-written data; it is published on the final chunk.
static uint8_t s_metadata[gc_metadata_max_size];
static uint8_t s_metadataChunks = 0;

// Set by the (0xB0) final-chunk handler in USB-callback context; serviced by
// SendReport() on core0 so the flash erase/program never runs from the USB
// interrupt path.
static volatile bool s_pendingCommit = false;

//--------------------------------------------------------------------+
// Metadata command handler (WebUSB transport). Mirrors the 0xA0/0xB0 subset of
// the physical-Joybus handlers. Returns the number of raw response bytes, or 0
// if unhandled / not ready.
//--------------------------------------------------------------------+
static int handleWebusbCommand(const uint8_t *cmd, int cmdLen, uint8_t *resp, int maxResp) {
    if (cmdLen < 1) {
        return 0;
    }

    switch (cmd[0]) {
        case 0xA0: { // Read metadata chunk: [0xA0, chunkIndex]
            if (cmdLen < 2 || maxResp < gc_metadata_chunk_transfer_size) {
                return 0;
            }
            uint8_t chunkIndex = cmd[1];
            if (chunkIndex >= s_metadataChunks) {
                chunkIndex = 0;
            }
            resp[0] = s_metadataChunks;
            resp[1] = chunkIndex;
            memcpy(
                &resp[2],
                &s_metadata[chunkIndex * gc_metadata_chunk_data_size],
                gc_metadata_chunk_data_size
            );
            return gc_metadata_chunk_transfer_size;
        }
        case 0xB0: { // Write metadata chunk: [0xB0, totalChunks, chunkIndex, ...78 data]
            if (cmdLen < 3 + gc_metadata_chunk_data_size || maxResp < 1) {
                return 0;
            }
            uint8_t totalChunks = cmd[1];
            uint8_t chunkIndex = cmd[2];
            const uint8_t *chunkData = &cmd[3];

            bool validChunk =
                totalChunks >= 1 && totalChunks <= gc_metadata_max_chunks && chunkIndex < totalChunks;
            if (validChunk) {
                memcpy(
                    &s_metadata[chunkIndex * gc_metadata_chunk_data_size],
                    chunkData,
                    gc_metadata_chunk_data_size
                );

                // Keep metadata hidden while a burst is still in flight; publish
                // and request a single flash commit once the final chunk lands.
                bool isFinalChunk = (chunkIndex + 1 == totalChunks);
                s_metadataChunks = isFinalChunk ? totalChunks : 0;
                if (isFinalChunk) {
                    s_pendingCommit = true;
                }
            }

            resp[0] = 0x01; // ack
            return 1;
        }
        default:
            return 0;
    }
}

//--------------------------------------------------------------------+
// WebUSB command channel state (control-transfer only).
//--------------------------------------------------------------------+
static uint8_t s_respBuf[WEBUSB_RESP_HEADER_LEN + WEBUSB_RAW_PAYLOAD_MAX];
static volatile uint16_t s_respLen = 0; // 0 = no response ready

static void webusb_command_processor(const uint8_t *data) {
    if (data[0] != WEBUSB_CMD_JOYBUS_CMD) {
        return;
    }
    uint8_t port = data[1];
    uint16_t cmd_len = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    uint16_t resp_len = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    if (cmd_len < 1 || cmd_len > WEBUSB_RAW_PAYLOAD_MAX || resp_len > WEBUSB_RAW_PAYLOAD_MAX) {
        return;
    }

    static uint8_t rawResp[WEBUSB_RAW_PAYLOAD_MAX];
    int rlen = handleWebusbCommand(&data[6], cmd_len, rawResp, sizeof(rawResp));
    if (rlen <= 0) {
        // Unhandled / not ready: leave the response empty so the host retries.
        s_respLen = 0;
        return;
    }
    if ((uint16_t)rlen > resp_len) {
        rlen = resp_len;
    }
    s_respBuf[0] = WEBUSB_CMD_JOYBUS_CMD;
    s_respBuf[1] = port;
    s_respBuf[2] = (uint8_t)(rlen & 0xff);
    s_respBuf[3] = (uint8_t)((rlen >> 8) & 0xff);
    memcpy(&s_respBuf[WEBUSB_RESP_HEADER_LEN], rawResp, (size_t)rlen);
    s_respLen = (uint16_t)(WEBUSB_RESP_HEADER_LEN + rlen);
}

//--------------------------------------------------------------------+
// WebUSB descriptors
//--------------------------------------------------------------------+
#define WEBUSB_BOS_TOTAL_LEN \
    (TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
#define WEBUSB_MS_OS_20_DESC_LEN 0xB2

static const uint8_t webusb_bos_descriptor[] = {
    TUD_BOS_DESCRIPTOR(WEBUSB_BOS_TOTAL_LEN, 2),
    // WebUSB: vendor request code, landing page index
    TUD_BOS_WEBUSB_DESCRIPTOR(VENDOR_REQUEST_WEBUSB, 1),
    // Microsoft OS 2.0
    TUD_BOS_MS_OS_20_DESCRIPTOR(WEBUSB_MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT),
};

// Bound to the WebUSB command interface (USB interface index 1).
#define WEBUSB_CMD_ITF_NUM 1

static const uint8_t webusb_ms_os_20[] = {
    // Set header: length, type, windows version, total length
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(WEBUSB_MS_OS_20_DESC_LEN),

    // Configuration subset header
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0,
    U16_TO_U8S_LE(WEBUSB_MS_OS_20_DESC_LEN - 0x0A),

    // Function subset header (first interface = WebUSB command interface)
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), WEBUSB_CMD_ITF_NUM, 0,
    U16_TO_U8S_LE(WEBUSB_MS_OS_20_DESC_LEN - 0x0A - 0x08),

    // Compatible ID: WINUSB
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S',
    'B', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // Registry property: DeviceInterfaceGUIDs
    U16_TO_U8S_LE(WEBUSB_MS_OS_20_DESC_LEN - 0x0A - 0x08 - 0x08 - 0x14),
    U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY), U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00,
    'e', 0x00, 'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00,
    'D', 0x00, 's', 0x00, 0x00, 0x00, U16_TO_U8S_LE(0x0050),
    // "{8B3E9D2E-7EEC-4994-AAE7-0C40DE84D36E}"
    '{', 0x00, '8', 0x00, 'B', 0x00, '3', 0x00, 'E', 0x00, '9', 0x00, 'D', 0x00, '2', 0x00, 'E', 0x00,
    '-', 0x00, '7', 0x00, 'E', 0x00, 'E', 0x00, 'C', 0x00, '-', 0x00, '4', 0x00, '9', 0x00, '9', 0x00,
    '4', 0x00, '-', 0x00, 'A', 0x00, 'A', 0x00, 'E', 0x00, '7', 0x00, '-', 0x00, '0', 0x00, 'C', 0x00,
    '4', 0x00, '0', 0x00, 'E', 0x00, 'E', 0x00, '8', 0x00, '4', 0x00, 'D', 0x00, '3', 0x00, '6', 0x00,
    'D', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00
};

static_assert(sizeof(webusb_ms_os_20) == WEBUSB_MS_OS_20_DESC_LEN, "Incorrect MS OS 2.0 descriptor size");

// WebUSB landing-page URL descriptor. tusb_desc_webusb_url_t uses a flexible
// array member which cannot be initialised in C++, so use a fixed-size packed
// equivalent (bLength excludes the trailing null terminator).
struct webusb_url_desc_t {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bScheme;
    char url[sizeof(WEBUSB_LANDING_URL)];
} __attribute__((packed));

static const webusb_url_desc_t webusb_landing_url = {
    (uint8_t)(3 + sizeof(WEBUSB_LANDING_URL) - 1),
    3, // WEBUSB URL descriptor type
    1, // https
    WEBUSB_LANDING_URL,
};

//--------------------------------------------------------------------+
// Reconstructed XInput BOS (the library's desc_bos has internal linkage, so it
// is rebuilt byte-for-byte here for the non-WebUSB personality).
//--------------------------------------------------------------------+
#define XINPUT_BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
#define XINPUT_MS_OS_20_DESC_LEN 0xB2

static const uint8_t xinput_bos_descriptor[] = {
    TUD_BOS_DESCRIPTOR(XINPUT_BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(XINPUT_MS_OS_20_DESC_LEN, XINPUT_VENDOR_REQUEST_MICROSOFT),
};

// XInput's MS OS 2.0 descriptor (external, unmangled symbol in the XInput lib).
extern "C" uint8_t desc_ms_os_20[];

//--------------------------------------------------------------------+
// XInput class-driver callbacks (external, C++-mangled symbols in the XInput
// lib). Declared here to rebuild the driver struct for the shared app-driver
// list (the library's own xinput_driver has internal linkage).
//--------------------------------------------------------------------+
extern uint16_t xinput_open(
    uint8_t rhport,
    tusb_desc_interface_t const *itf_descriptor,
    uint16_t max_length
);
extern bool xinput_control_xfer_callback(
    uint8_t rhport,
    uint8_t stage,
    tusb_control_request_t const *request
);
extern bool xinput_xfer_callback(
    uint8_t rhport,
    uint8_t ep_addr,
    xfer_result_t result,
    uint32_t xferred_bytes
);

static void xinput_driver_init(void) {}
static void xinput_driver_reset(uint8_t rhport) {
    (void)rhport;
}

static const usbd_class_driver_t reconstructed_xinput_driver = {
#if CFG_TUSB_DEBUG >= 2
    .name = "XINPUT",
#endif
    .init = xinput_driver_init,
    .reset = xinput_driver_reset,
    .open = xinput_open,
    .control_xfer_cb = xinput_control_xfer_callback,
    .xfer_cb = xinput_xfer_callback,
    .sof = NULL
};

//--------------------------------------------------------------------+
// WebUSB class driver: owns the two vendor interfaces. Interface 0 (the
// adapter) has two interrupt endpoints for the GameCube report channel;
// interface 1 (command) is control-transfer only for the metadata channel.
//--------------------------------------------------------------------+

// Adapter report-channel state (interface 0 endpoints). Written from USB
// callbacks and the main loop; endpoint claim/busy tracking keeps IN submits
// serialised.
static volatile bool s_adapterReady = false; // endpoints open
static uint8_t s_adapterInEp = ADAPTER_EP_IN;
static uint8_t s_adapterOutEp = ADAPTER_EP_OUT;
static uint8_t s_adapterOutBuf[ADAPTER_EP_OUT_SIZE];
static uint8_t s_adapterReport[ADAPTER_REPORT_LEN];

// Map the controller outputs into one 9-byte adapter port block (Dolphin
// layout: status, buttons1, buttons2, stickX, stickY, cX, cY, lAnalog, rAnalog).
static void fill_adapter_report(uint8_t *report, const OutputState &o) {
    memset(report, 0, ADAPTER_REPORT_LEN);
    report[0] = 0x21; // adapter input report id

    uint8_t *b = &report[1]; // port 1 (first controller); ports 2-4 stay zeroed
    b[0] = 0x10;             // connected, wired
    b[1] = (uint8_t)((o.a ? 0x01 : 0) | (o.b ? 0x02 : 0) | (o.x ? 0x04 : 0) |
                     (o.y ? 0x08 : 0) | ((o.dpadLeft || o.select) ? 0x10 : 0) |
                     ((o.dpadRight || o.home) ? 0x20 : 0) | (o.dpadDown ? 0x40 : 0) |
                     (o.dpadUp ? 0x80 : 0));
    b[2] = (uint8_t)((o.start ? 0x01 : 0) | (o.buttonR ? 0x02 : 0) |
                     (o.triggerRDigital ? 0x04 : 0) | (o.triggerLDigital ? 0x08 : 0));
    b[3] = o.leftStickX;
    b[4] = o.leftStickY;
    b[5] = o.rightStickX;
    b[6] = o.rightStickY;
    b[7] = o.triggerLAnalog;
    b[8] = o.triggerRAnalog;
}

static void webusb_driver_init(void) {}
static void webusb_driver_reset(uint8_t rhport) {
    (void)rhport;
    s_adapterReady = false;
}
static bool webusb_driver_xfer_cb(
    uint8_t rhport,
    uint8_t ep_addr,
    xfer_result_t result,
    uint32_t xferred_bytes
) {
    (void)result;
    (void)xferred_bytes;
    if (ep_addr == s_adapterOutEp) {
        // Host command received (0x13 start / 0x11 rumble). Ignored here; just
        // re-arm the OUT endpoint so subsequent host writes keep completing.
        usbd_edpt_xfer(rhport, s_adapterOutEp, s_adapterOutBuf, sizeof(s_adapterOutBuf));
    }
    return true;
}

// Claim the vendor-specific interfaces owned by this driver (class 0xFF,
// subclass 0, protocol 0). The XInput interface uses subclass 0x5D, so the two
// drivers never contend. Interface 0 additionally opens its two interrupt
// endpoints (the report channel); interface 1 has none.
static uint16_t webusb_driver_open(
    uint8_t rhport,
    tusb_desc_interface_t const *itf_descriptor,
    uint16_t max_length
) {
    if (itf_descriptor->bInterfaceClass != TUSB_CLASS_VENDOR_SPECIFIC ||
        itf_descriptor->bInterfaceSubClass != 0x00 || itf_descriptor->bInterfaceProtocol != 0x00) {
        return 0;
    }

    uint16_t driver_length = sizeof(tusb_desc_interface_t);

    if (itf_descriptor->bNumEndpoints >= 2) {
        driver_length += 2 * sizeof(tusb_desc_endpoint_t);
        TU_VERIFY(max_length >= driver_length, 0);

        uint8_t const *p_ep = tu_desc_next(itf_descriptor);
        uint8_t ep_out = 0, ep_in = 0;
        TU_VERIFY(
            usbd_open_edpt_pair(rhport, p_ep, 2, TUSB_XFER_INTERRUPT, &ep_out, &ep_in),
            0
        );
        s_adapterOutEp = ep_out;
        s_adapterInEp = ep_in;
        s_adapterReady = true;

        // Arm the OUT endpoint so host start/rumble writes have somewhere to land.
        usbd_edpt_xfer(rhport, s_adapterOutEp, s_adapterOutBuf, sizeof(s_adapterOutBuf));
        return driver_length;
    }

    TU_VERIFY(max_length >= driver_length, 0);
    return driver_length;
}

// This driver owns no endpoints and receives no class-type control requests
// (the WebUSB command channel uses vendor-type requests, which TinyUSB routes
// to tud_vendor_control_xfer_cb -> webusb_vendor_control below, never here).
static bool webusb_driver_control_xfer_cb(
    uint8_t rhport,
    uint8_t stage,
    tusb_control_request_t const *request
) {
    (void)rhport;
    (void)stage;
    (void)request;
    return true;
}

static const usbd_class_driver_t webusb_driver = {
#if CFG_TUSB_DEBUG >= 2
    .name = "WEBUSB",
#endif
    .init = webusb_driver_init,
    .reset = webusb_driver_reset,
    .open = webusb_driver_open,
    .control_xfer_cb = webusb_driver_control_xfer_cb,
    .xfer_cb = webusb_driver_xfer_cb,
    .sof = NULL
};

//--------------------------------------------------------------------+
// Helpers invoked from the strong C overrides in webusb_usb_hooks.c.
//--------------------------------------------------------------------+
extern "C" const void *webusb_get_app_drivers(unsigned char *driver_count) {
    static usbd_class_driver_t drivers[2];
    drivers[0] = webusb_driver;
    drivers[1] = reconstructed_xinput_driver;
    *driver_count = 2;
    return drivers;
}

extern "C" const unsigned char *webusb_get_bos(void) {
    return g_webusb_mode ? webusb_bos_descriptor : xinput_bos_descriptor;
}

// Device-level vendor control requests. TinyUSB routes *all* vendor-type
// control transfers here (regardless of recipient), so this implements the
// entire WebUSB command channel: landing page / MS OS 2.0 plus the command
// (10) and response (11) requests. In the XInput personality only the MS OS
// 2.0 request is served. Returns 1 to accept, 0 to stall.
extern "C" unsigned char webusb_vendor_control(
    unsigned char rhport,
    unsigned char stage,
    const void *request_
) {
    tusb_control_request_t const *request = (tusb_control_request_t const *)request_;
    static uint8_t webusb_ctrl_buf[WEBUSB_CTRL_BUF_SIZE];

    // The command payload for request 10 finishes arriving in the DATA stage.
    if (stage == CONTROL_STAGE_DATA) {
        if (g_webusb_mode && request->bRequest == VENDOR_REQUEST_WEBUSB_CMD) {
            webusb_command_processor(webusb_ctrl_buf);
        }
        return 1;
    }

    // Nothing to do for the ACK stage.
    if (stage != CONTROL_STAGE_SETUP) {
        return 1;
    }
    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR) {
        return 0;
    }

    if (g_webusb_mode) {
        switch (request->bRequest) {
            case VENDOR_REQUEST_WEBUSB:
                return tud_control_xfer(
                           rhport,
                           request,
                           (void *)(uintptr_t)&webusb_landing_url,
                           webusb_landing_url.bLength
                       )
                    ? 1
                    : 0;
            case VENDOR_REQUEST_MICROSOFT:
                if (request->wIndex == 7) {
                    uint16_t total_len;
                    memcpy(&total_len, webusb_ms_os_20 + 8, 2);
                    return tud_control_xfer(
                               rhport,
                               request,
                               (void *)(uintptr_t)webusb_ms_os_20,
                               total_len
                           )
                        ? 1
                        : 0;
                }
                return 0;
            case VENDOR_REQUEST_WEBUSB_CMD: {
                // Receive the command payload into webusb_ctrl_buf; the DATA
                // stage above then processes it.
                uint16_t len = tu_min16(request->wLength, sizeof(webusb_ctrl_buf));
                return tud_control_xfer(rhport, request, webusb_ctrl_buf, len) ? 1 : 0;
            }
            case VENDOR_REQUEST_WEBUSB_RESP: {
                uint16_t rlen = s_respLen;
                if (rlen == 0) {
                    // No response ready — return zero-length so the host retries.
                    return tud_control_xfer(rhport, request, NULL, 0) ? 1 : 0;
                }
                s_respLen = 0; // consumed
                uint16_t len = tu_min16(request->wLength, rlen);
                return tud_control_xfer(rhport, request, s_respBuf, len) ? 1 : 0;
            }
            default:
                return 0;
        }
    }

    // XInput personality (only reached when the XInput BOS was served).
    if (request->bRequest == XINPUT_VENDOR_REQUEST_MICROSOFT && request->wIndex == 7) {
        uint16_t total_len;
        memcpy(&total_len, desc_ms_os_20 + 8, 2);
        return tud_control_xfer(rhport, request, (void *)desc_ms_os_20, total_len) ? 1 : 0;
    }
    return 0;
}

//--------------------------------------------------------------------+
// USB interface: a bare vendor-specific interface with no endpoints.
//--------------------------------------------------------------------+
class WebUsbVendorInterface : public Adafruit_USBD_Interface {
  public:
    uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf, uint16_t bufsize) {
        const uint8_t desc[9] = {
            9,        TUSB_DESC_INTERFACE,        itfnum, 0x00, 0x00,
            (uint8_t)TUSB_CLASS_VENDOR_SPECIFIC, 0x00,   0x00, 0x00
        };
        if (buf == NULL) {
            return sizeof(desc);
        }
        if (bufsize < sizeof(desc)) {
            return 0;
        }
        memcpy(buf, desc, sizeof(desc));
        return sizeof(desc);
    }
};

//--------------------------------------------------------------------+
// USB interface: the WUP-028 adapter interface — vendor-specific with two
// interrupt endpoints (IN report channel, OUT command channel).
//--------------------------------------------------------------------+
class WebUsbAdapterInterface : public Adafruit_USBD_Interface {
  public:
    uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf, uint16_t bufsize) {
        const uint8_t desc[] = {
            // Interface: vendor-specific, 2 endpoints
            9,
            TUSB_DESC_INTERFACE,
            itfnum,
            0x00,
            0x02,
            (uint8_t)TUSB_CLASS_VENDOR_SPECIFIC,
            0x00,
            0x00,
            0x00,
            // Endpoint OUT (host -> adapter: start / rumble)
            7,
            TUSB_DESC_ENDPOINT,
            ADAPTER_EP_OUT,
            TUSB_XFER_INTERRUPT,
            (uint8_t)(ADAPTER_EP_OUT_SIZE & 0xFF),
            (uint8_t)(ADAPTER_EP_OUT_SIZE >> 8),
            1,
            // Endpoint IN (adapter -> host: input reports)
            7,
            TUSB_DESC_ENDPOINT,
            ADAPTER_EP_IN,
            TUSB_XFER_INTERRUPT,
            (uint8_t)(ADAPTER_EP_IN_SIZE & 0xFF),
            (uint8_t)(ADAPTER_EP_IN_SIZE >> 8),
            1,
        };
        if (buf == NULL) {
            return sizeof(desc);
        }
        if (bufsize < sizeof(desc)) {
            return 0;
        }
        memcpy(buf, desc, sizeof(desc));
        return sizeof(desc);
    }
};

//--------------------------------------------------------------------+
// Backend
//--------------------------------------------------------------------+
WebUSBBackend::WebUSBBackend(InputSource **input_sources, size_t input_source_count)
    : CommunicationBackend(input_sources, input_source_count) {
    // Take over the USB personality before touching the descriptors so the
    // overrides serve the WebUSB BOS during (re-)enumeration.
    g_webusb_mode = true;

    if (!gc_metadata_load(s_metadata, s_metadataChunks)) {
        gc_metadata_build_default(s_metadata, s_metadataChunks);
        gc_metadata_save(s_metadata, s_metadataChunks);
    }

    // Rebuild the composite as a WebUSB-only device with two vendor interfaces:
    // the adapter (interface 0, two interrupt endpoints — the GameCube report
    // channel Dolphin reads) and the command interface (interface 1, control
    // transfers only) that the browser metadata host claims.
    static WebUsbAdapterInterface itf_adapter;
    static WebUsbVendorInterface itf_command;

    TinyUSBDevice.detach();
    Serial.end(); // clearConfiguration(): drops CDC and resets the descriptors
    TinyUSBDevice.setManufacturerDescriptor("HayBox");
    TinyUSBDevice.setProductDescriptor("HayBox");
    TinyUSBDevice.addInterface(itf_adapter);
    TinyUSBDevice.addInterface(itf_command);
    TinyUSBDevice.setID(0x057E, 0x0337);
    TinyUSBDevice.setVersion(0x0210); // USB 2.1: required for BOS / WebUSB
    delay(10);
    TinyUSBDevice.attach();
}

WebUSBBackend::~WebUSBBackend() {}

void WebUSBBackend::SendReport() {
    // USB is serviced by the core (yield()/ISR); here we only commit a completed
    // metadata write to flash, on core0, outside the USB callback context.
    if (s_pendingCommit) {
        s_pendingCommit = false;
        gc_metadata_save(s_metadata, s_metadataChunks);
    }

    // Stream GameCube controller state on the adapter IN endpoint (once Dolphin
    // has configured the device and opened the endpoints).
    if (!s_adapterReady) {
        return;
    }

    ScanInputs(InputScanSpeed::FAST);
    UpdateOutputs();

    // Only build/submit when the endpoint is free; claim fails while a report
    // is still in flight, in which case we simply try again next loop.
    if (usbd_edpt_claim(0, s_adapterInEp)) {
        fill_adapter_report(s_adapterReport, _outputs);
        if (!usbd_edpt_xfer(0, s_adapterInEp, s_adapterReport, ADAPTER_REPORT_LEN)) {
            usbd_edpt_release(0, s_adapterInEp);
        }
    }
}
