#pragma once
#include <stdint.h>
void link_init(void);
void link_poll(void);
int  link_stream_pop(uint8_t *rep64, int itf);   /* the slots s with s % DONGLE_NIF == itf */
int  link_cmd(const uint8_t *in, uint8_t *d, uint8_t *st);
void link_rssi_scan(uint8_t first, uint8_t *out, int n);
void link_set_idle_hook(void (*f)(void));
