#pragma once
#define CFG_TUSB_MCU            OPT_MCU_NRF5X
#define CFG_TUSB_OS             OPT_OS_NONE
#define CFG_TUD_ENABLED         1
#define CFG_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED
#define CFG_TUD_ENDPOINT0_SIZE  64
#define CFG_TUD_HID             4
#define CFG_TUD_HID_EP_BUFSIZE  64
#define CFG_TUSB_MEM_ALIGN      __attribute__((aligned(4)))
