# doorman radio firmware (ai/doorman/radio-fw.md).
#   make             Vive Tracker 1.0 (nRF51822)       -> build/radio.bin
#                    install: lighthouse_watchman_update -rnt build/radio.bin   (one tracker on USB)
#   make TARGET=idx  Valve nRF52 radio (Index family)  -> build-idx/radio.bin + build-idx/radio_idx.fw
#                    install: lighthouse_watchman_update -s <serial> --target=nrf52 build-idx/radio_idx.fw
#   make TARGET=fake Pro Micro (nRF52840) as a test device: synthetic LH2 load, no main MCU (DML_FAKE)
#                    -> build-fake/radio_fake_sd.uf2 / radio_fake_nosd.uf2 (by the board's UF2 bootloader)
#   DIAG=1           dev build: staging counters + SPI trace (tools/air_update.py info / trace)
MDK = ../third_party/nrfx/mdk
COMMON = -mthumb -Os -g -ffunction-sections -fdata-sections -Wall -Wextra -std=gnu11 \
  -I../common -I$(MDK) -I../third_party/CMSIS_5/CMSIS/Core/Include -ffreestanding
ifeq ($(DIAG),1)
COMMON += -DDML_DIAG
endif
ifeq ($(TARGET),fake)
B = build-fake
TUSB = ../third_party/tinyusb
NRFX = ../third_party/nrfx
CFLAGS = -mcpu=cortex-m4 $(COMMON) -DNRF52840_XXAA -DDML_IDX -DDML_FAKE -DDML_MULTI -DDLG_EV=256 $(EXTRA) -DCFG_TUSB_MCU=OPT_MCU_NRF5X \
  -Isrc/fake -I../dongle-fw/src -I$(TUSB)/src -I$(NRFX) -I$(NRFX)/hal -I$(NRFX)/drivers/include -I$(NRFX)/drivers/src -I$(NRFX)/templates -Wno-override-init
SRCS = src/main.c src/startup.c $(TUSB)/src/tusb.c $(TUSB)/src/common/tusb_fifo.c $(TUSB)/src/device/usbd.c \
  $(TUSB)/src/device/usbd_control.c $(TUSB)/src/class/hid/hid_device.c $(TUSB)/src/portable/nordic/nrf5x/dcd_nrf5x.c
LD = radio_fake_sd.ld
OUT = $(B)/radio_fake_sd.uf2 $(B)/radio_fake_nosd.uf2
else ifeq ($(TARGET),idx)
B = build-idx
CFLAGS = -mcpu=cortex-m4 $(COMMON) -DNRF52832_XXAA -DDML_IDX
SRCS = src/main.c src/startup.c
LD = radio_idx.ld
FIX = --idx
OUT = $(B)/radio.bin $(B)/radio_idx.fw
else
B = build
CFLAGS = -mcpu=cortex-m0 $(COMMON) -DNRF51 -DNRF51822_XXAB
SRCS = src/main.c src/startup.c $(MDK)/system_nrf51.c
LD = radio.ld
FIX =
OUT = $(B)/radio.bin
endif
all: $(OUT)
$(B)/radio.elf: $(SRCS) $(LD) ../common/doorman_link.h ../common/doorman_codec.h ../common/doorman_lh2p.h
	@mkdir -p $(B)
	arm-none-eabi-gcc $(CFLAGS) $(SRCS) -nostartfiles -nostdlib -lgcc -Wl,--gc-sections -T$(LD) -Wl,-Map=$(B)/radio.map -o $@ -lc -lgcc
	arm-none-eabi-size $@
$(B)/radio.bin: $(B)/radio.elf
	arm-none-eabi-objcopy -O binary $< $@
	python3 ../tools/radio_image.py fix $(FIX) $@
$(B)/radio_idx.fw: $(B)/radio.bin
	python3 ../tools/radio_image.py container $< $@
$(B)/radio_fake_%.elf: $(SRCS) src/fake/*.c src/fake/*.h radio_fake_%.ld ../common/doorman_link.h ../common/doorman_codec.h ../common/doorman_lh2p.h ../common/doorman_lh2gen.h
	@mkdir -p $(B)
	arm-none-eabi-gcc $(CFLAGS) $(SRCS) -nostartfiles -nostdlib -lgcc -Wl,--gc-sections -Tradio_fake_$*.ld -Wl,-Map=$(B)/radio_fake_$*.map -o $@ -lc -lgcc
	arm-none-eabi-size $@
$(B)/radio_fake_%.uf2: $(B)/radio_fake_%.elf
	arm-none-eabi-objcopy -O ihex $< $(B)/radio_fake_$*.hex
	python3 ../dongle-fw/tools/uf2conv.py -f 0xADA52840 -c -o $@ $(B)/radio_fake_$*.hex > /dev/null
clean:
	rm -rf build build-idx build-fake
