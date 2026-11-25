# doorman dongle firmware (nRF52840 Dongle, ai/doorman/dongle-fw.md)
# make            -> every board's install image and bank images (below)
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
TUSB = ../third_party/tinyusb
NRFX = ../third_party/nrfx
CFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -g -ffunction-sections -fdata-sections \
  -Wall -Wextra -Wno-unused-parameter -std=gnu11 -DNRF52840_XXAA -DCFG_TUSB_MCU=OPT_MCU_NRF5X \
  -Isrc -I../common -I$(TUSB)/src -I$(NRFX) -I$(NRFX)/mdk -I$(NRFX)/hal -I$(NRFX)/drivers/include -I$(NRFX)/drivers/src -I$(NRFX)/templates \
  -I../third_party/CMSIS_5/CMSIS/Core/Include -D__STACK_SIZE=4096 -D__HEAP_SIZE=0
SRCS = src/main.c src/link.c src/usb_descriptors.c $(NRFX)/mdk/system_nrf52840.c \
  $(TUSB)/src/tusb.c $(TUSB)/src/common/tusb_fifo.c $(TUSB)/src/device/usbd.c $(TUSB)/src/device/usbd_control.c \
  $(TUSB)/src/class/hid/hid_device.c $(TUSB)/src/portable/nordic/nrf5x/dcd_nrf5x.c
LDFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wl,--gc-sections -L$(NRFX)/mdk -specs=nano.specs -specs=nosys.specs

# Every board runs the two-bank layout (src/banks.h, ai/doorman/dongle-fw.md):
#   build/doorman_dongle_<v>_a.bin / _b.bin   firmware for bank A / B: self-updates over USB (cmd 0x40-0x42)
#   build/doorman_dongle_<v>.hex (.uf2)       boot stub + bank A's image: what a bootloader installs
#                                             (first install or rescue)
VARIANTS = sd nosd holyiot pca10059 kuroshin
layout = $(if $(filter sd,$(1)),sd,std)
board_flags = -DVARIANT_SD=$(if $(filter sd,$(1)),1,0) -DBANK_LAYOUT_SD=$(if $(filter sd,$(1)),1,0) \
  -DBOARD_HOLYIOT=$(if $(filter holyiot,$(1)),1,0) -DBOARD_PCA10059=$(if $(filter pca10059,$(1)),1,0) \
  -DBOARD_KUROSHIN=$(if $(filter kuroshin,$(1)),1,0)

all: $(foreach v,$(VARIANTS),build/doorman_dongle_$(v).hex build/doorman_dongle_$(v)_a.bin build/doorman_dongle_$(v)_b.bin) \
  build/doorman_dongle_sd.uf2 build/doorman_dongle_nosd.uf2

define variant
build/doorman_dongle_$(1)_%.elf: $(SRCS) $(NRFX)/mdk/gcc_startup_nrf52840.S src/*.h ../common/*.h ld/bank_$(call layout,$(1))_%.ld
	@mkdir -p build
	$(CC) $(CFLAGS) $(call board_flags,$(1)) $(SRCS) $(NRFX)/mdk/gcc_startup_nrf52840.S \
	  $(LDFLAGS) -Tld/bank_$(call layout,$(1))_$$*.ld -Wl,-Map=build/doorman_dongle_$(1)_$$*.map -o $$@
	arm-none-eabi-size $$@

# the boot stub, with bank A's image as payload
build/doorman_dongle_$(1).elf: src/stub.c src/banks.h ld/stub_$(call layout,$(1)).ld build/doorman_dongle_$(1)_a.bin
	$(CC) $(CFLAGS) $(call board_flags,$(1)) -DPAYLOAD_BIN='"build/doorman_dongle_$(1)_a.bin"' src/stub.c -nostartfiles -nostdlib \
	  $(LDFLAGS) -Tld/stub_$(call layout,$(1)).ld -Wl,-Map=build/doorman_dongle_$(1).map -o $$@
	arm-none-eabi-size $$@
endef
$(foreach v,$(VARIANTS),$(eval $(call variant,$(v))))

# a bank image as raw bytes from its first address, padded to whole words
build/%_a.bin: build/%_a.elf
	$(OBJCOPY) -O binary $< $@
	truncate -s %4 $@
build/%_b.bin: build/%_b.elf
	$(OBJCOPY) -O binary $< $@
	truncate -s %4 $@
build/%.hex: build/%.elf
	$(OBJCOPY) -O ihex $< $@
build/%.uf2: build/%.hex
	python3 tools/uf2conv.py -f 0xADA52840 -c -o $@ $< >/dev/null

clean:
	rm -rf build
.SECONDARY:
.PHONY: all clean
