#pragma once
#include "nrf.h"

#define DONGLE_VID         0x6767
#define DONGLE_PID         0x0727
#define DONGLE_FW_VERSION  0x002A
/* USB data interfaces: slot s streams on interface s % DONGLE_NIF (interface 0 also carries commands) */
#define DONGLE_NIF         4
#define STREAM_KIND        0x80   /* IN report: [0x80][slot][n][n bytes of [len][frame]... stream] */

/* host <-> dongle, 64-byte HID reports (no report ID)
   OUT: [cmd][seq][args...]      IN: [cmd][seq][status][data...]   status 0 = ok */
enum {
    CMD_INFO = 0x01, CMD_STATS = 0x02, CMD_ECHO = 0x03,
    CMD_DFU_UF2 = 0x10, CMD_DFU_SERIAL = 0x11, CMD_RESET = 0x12, CMD_CRASH_TEST = 0x13, CMD_CLEAR_CRASH = 0x14,
    CMD_LINK = 0x20, CMD_DOWN = 0x21, CMD_RADIO_RSSI_SCAN = 0x22, CMD_LINKSTATS = 0x23,
    /* self-update (banks.h): [size u32][crc u32] erases the other bank; [offset u32][n][n bytes] writes
       into it; commit checks the CRC, writes its header (next sequence number) and restarts into it */
    CMD_BANK_BEGIN = 0x40, CMD_BANK_WRITE = 0x41, CMD_BANK_COMMIT = 0x42,
};
enum { ST_OK = 0, ST_BAD_CMD = 1, ST_BAD_ARG = 2, ST_TIMEOUT = 3 };

/* Boards on Nordic's open USB DFU bootloader (not UF2): Holyiot 21017 (amplifier) and Nordic's own
   nRF52840 Dongle PCA10059. Same flash layout, same update path. */
#ifndef BOARD_HOLYIOT
#define BOARD_HOLYIOT 0
#endif
#ifndef BOARD_PCA10059
#define BOARD_PCA10059 0
#endif
/* Kuroshin: PCA10059-like; its bootloader can't be entered from software (pin reset only). Every board
   updates itself over USB anyway (banks.h, commands 0x40-0x42). */
#ifndef BOARD_KUROSHIN
#define BOARD_KUROSHIN 0
#endif
#define BOARD_NORDIC_DFU (BOARD_HOLYIOT || BOARD_PCA10059 || BOARD_KUROSHIN)
#define BOARD_ID (BOARD_HOLYIOT ? 1 : BOARD_PCA10059 ? 2 : BOARD_KUROSHIN ? 3 : 0)   /* cmd 0x01 byte 57 */
/* USB product string: the board this image is for. Host code matches the "Doorman Dongle" prefix. */
#define DONGLE_PRODUCT (BOARD_HOLYIOT ? "Doorman Dongle Holyiot" : BOARD_PCA10059 ? "Doorman Dongle PCA10059" \
                        : BOARD_KUROSHIN ? "Doorman Dongle Kuroshin" : "Doorman Dongle Pro Micro")

/* Adafruit nRF52 bootloader magic values in NRF_POWER->GPREGRET */
#if BOARD_NORDIC_DFU
#define GPREGRET_UF2_DFU     0xB1   /* Nordic open USB DFU bootloader (nRF Connect / nrfutil) */
#else
#define GPREGRET_UF2_DFU     0x57
#endif
#define GPREGRET_SERIAL_DFU  0x4E
