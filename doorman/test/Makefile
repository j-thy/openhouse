# Doorman codec and link tests (ai/doorman/testing.md). Host tools build with cc; the chip timing needs
# arm-none-eabi-gcc and Python's unicorn. run.sh runs everything on a capture.
#   make            host tools: codec_roundtrip, cycle_sim, emu/batch_ref
#   make emu        the radio's batch path for the Cortex-M4 (build/radio_batch.elf / .bin)
B = build
COMMON = ../common
CC ?= cc
HOSTFLAGS = -O2 -Wall -Wextra -std=gnu11 -I$(COMMON) -I../dongle-fw/src
ARMFLAGS = -g -mcpu=cortex-m4 -mthumb -Os -ffreestanding -ffunction-sections -fdata-sections -std=gnu11 -I$(COMMON) \
  -nostdlib -nostartfiles -Wl,--gc-sections -Wl,-e,init -Wl,-u,push -Wl,-u,close_b -T emu/emu.ld
HDRS = $(COMMON)/doorman_codec.h $(COMMON)/doorman_lh2p.h $(COMMON)/doorman_link.h ../dongle-fw/src/sizing.h

comma := ,
all: $(B)/codec_roundtrip $(B)/cycle_sim $(B)/batch_ref $(B)/lh2gen_check
emu: $(B)/radio_batch.bin $(B)/gen_emu.bin $(B)/gen_ref

$(B)/%: %.c $(HDRS)
	@mkdir -p $(B)
	$(CC) $(HOSTFLAGS) $< -o $@
$(B)/batch_ref: emu/batch_ref.c $(HDRS)
	@mkdir -p $(B)
	$(CC) $(HOSTFLAGS) $< -o $@
$(B)/radio_batch.elf: emu/radio_batch.c emu/emu.ld $(HDRS)
	@mkdir -p $(B)
	arm-none-eabi-gcc $(ARMFLAGS) $< -lgcc -o $@
$(B)/radio_batch.bin: $(B)/radio_batch.elf
	arm-none-eabi-objcopy -O binary -j .text $< $@
$(B)/gen_ref: emu/gen_ref.c $(HDRS) $(COMMON)/doorman_lh2gen.h
	@mkdir -p $(B)
	$(CC) $(HOSTFLAGS) $< -o $@
$(B)/gen_emu.elf: emu/gen_emu.c emu/emu.ld $(COMMON)/doorman_lh2gen.h
	@mkdir -p $(B)
	arm-none-eabi-gcc $(subst -Wl$(comma)-u$(comma)push -Wl$(comma)-u$(comma)close_b,-Wl$(comma)-u$(comma)run,$(ARMFLAGS)) $< -lgcc -o $@
$(B)/gen_emu.bin: $(B)/gen_emu.elf
	arm-none-eabi-objcopy -O binary -j .text $< $@
$(B)/lh2gen_check: lh2gen_check.c $(COMMON)/doorman_lh2gen.h
	@mkdir -p $(B)
	$(CC) $(HOSTFLAGS) $< -o $@
clean:
	rm -rf $(B)
.PHONY: all emu clean
