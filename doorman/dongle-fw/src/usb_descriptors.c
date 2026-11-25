// USB descriptors: four vendor-defined HID interfaces, 64-byte IN/OUT reports, no report IDs.
// Interface 0: commands/replies; slot s streams on interface s % 4. A full-speed interrupt endpoint
// sends one 64-byte report per 1 ms frame (~61 kB/s of stream); with two interfaces the even slots'
// endpoint ran near that limit and their data queued here for tens of ms. Four keep each one at
// two devices, well under it.
#include "tusb.h"
#include "dongle.h"

static const tusb_desc_device_t dev_desc = {
    .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, .bDeviceClass = 0, .bDeviceSubClass = 0, .bDeviceProtocol = 0,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = DONGLE_VID, .idProduct = DONGLE_PID, .bcdDevice = DONGLE_FW_VERSION,
    .iManufacturer = 1, .iProduct = 2, .iSerialNumber = 3, .bNumConfigurations = 1,
};
const uint8_t *tud_descriptor_device_cb(void) { return (const uint8_t *)&dev_desc; }

static const uint8_t hid_report_desc[] = { TUD_HID_REPORT_DESC_GENERIC_INOUT(64) };
const uint8_t *tud_hid_descriptor_report_cb(uint8_t itf) { (void)itf; return hid_report_desc; }

#define CFG_LEN (TUD_CONFIG_DESC_LEN + DONGLE_NIF * TUD_HID_INOUT_DESC_LEN)
static const uint8_t cfg_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, DONGLE_NIF, 0, CFG_LEN, 0, 100),
    TUD_HID_INOUT_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, sizeof(hid_report_desc), 0x01, 0x81, 64, 1),
    TUD_HID_INOUT_DESCRIPTOR(1, 0, HID_ITF_PROTOCOL_NONE, sizeof(hid_report_desc), 0x02, 0x82, 64, 1),
    TUD_HID_INOUT_DESCRIPTOR(2, 0, HID_ITF_PROTOCOL_NONE, sizeof(hid_report_desc), 0x03, 0x83, 64, 1),
    TUD_HID_INOUT_DESCRIPTOR(3, 0, HID_ITF_PROTOCOL_NONE, sizeof(hid_report_desc), 0x04, 0x84, 64, 1),
};
const uint8_t *tud_descriptor_configuration_cb(uint8_t i) { (void)i; return cfg_desc; }

static uint16_t str_buf[33];
const uint16_t *tud_descriptor_string_cb(uint8_t idx, uint16_t langid) {
    (void)langid;
    char serial[17];
    const char *s;
    switch (idx) {
    case 0: str_buf[1] = 0x0409; str_buf[0] = (TUSB_DESC_STRING << 8) | 4; return str_buf;
    case 1: s = "openhouse"; break;
    case 2: s = DONGLE_PRODUCT; break;
    case 3: {
        static const char hx[] = "0123456789ABCDEF";
        uint32_t id[2] = { NRF_FICR->DEVICEID[1], NRF_FICR->DEVICEID[0] };
        for (int i = 0; i < 16; i++) serial[i] = hx[(id[i / 8] >> (28 - 4 * (i % 8))) & 15];
        serial[16] = 0; s = serial; break;
    }
    default: return NULL;
    }
    int n = 0;
    while (s[n] && n < 32) { str_buf[1 + n] = (uint8_t)s[n]; n++; }
    str_buf[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    return str_buf;
}
