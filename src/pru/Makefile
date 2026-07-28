# Additional PRU build rules

ifeq ($(CONFIG_PRU_HOST_BUILD),y)

# Host-emulation build: produce a single host-arch klipper.elf that
# contains the PRU pru1 firmware code (src/pru/main.c + gpio.c) plus
# generic timer_irq, with a host-side replacement for the pru-cgt
# intrinsics, the IEP timer, the PRU INTC and the pru0 rpmsg shuttle.
# scripts/test_klippy.py treats the resulting linuxpru.dict as a
# linuxprocess-style backend (host binary spawned with -I<slave-link>;
# the binary creates the pty itself - see src/pru/host_pru.c).

CROSS_PREFIX=

dirs-y += src/pru src/generic

# Force-include host_pru.h before every translation unit so the
# PRU sources don't need per-file #include reshuffling, and add
# host_inc/ to the include search path so `<pru/io.h>` and
# `<pru_iep.h>` resolve to the host stubs (the real headers ship
# with pru-cgt and aren't on the host gcc include path anyway).
CFLAGS += -Isrc/pru/host_inc -include src/pru/host_pru.h
CFLAGS_klipper.elf += -lpthread -lutil -lrt

src-y += pru/main.c pru/gpio.c pru/host_pru.c generic/timer_irq.c
# command.c (pulled in by src/Makefile) calls crc16_ccitt for the
# message-block trailer; on real PRU that lives in the pru0 ELF
# alongside command.c. The host build links command.c into the
# single host ELF, so it needs crc16_ccitt.c too.
src-y += generic/crc16_ccitt.c

else

# Setup the toolchain
CROSS_PREFIX=pru-

dirs-y += src/pru src/generic
dirs-y += lib/pru_rpmsg

CFLAGS += -Os -minrt -mmcu=am335x.pru1
CFLAGS += -Ilib/pru_rpmsg/include -Ilib/pru_rpmsg/include/am335x

CFLAGS_klipper.elf := $(filter-out -mmcu=%, $(CFLAGS))
CFLAGS_klipper.elf += -Wl,-r -nostdlib -Wl,-T,src/pru/pru.lds
CFLAGS_pru0.elf := $(filter-out -mmcu=%, $(CFLAGS)) -mmcu=am335x.pru0
CFLAGS_pru1.elf := $(CFLAGS)

# Add source files
src-y += pru/main.c pru/gpio.c generic/timer_irq.c
src-$(CONFIG_WANT_ADC) += pru/adc.c

pru0-y := pru/pru0.c generic/crc16_ccitt.c command.c
pru0-y += ../lib/pru_rpmsg/pru_rpmsg.c ../lib/pru_rpmsg/pru_virtqueue.c

# Build the PRU binaries
target-y += $(OUT)pru0.elf $(OUT)pru1.elf

$(OUT)pru0.elf: $(patsubst %.c, $(OUT)src/%.o,$(pru0-y))
	@echo "  Linking $@"
	$(Q)$(CC) $(CFLAGS_klipper.elf) $^ -o $(OUT)pru0.o
	# dumping info about INTC
	$(Q)$(OBJCOPY) --dump-section '.pru_irq_map'=$(OUT)pru0.pru_irq_map.bin $(OUT)src/pru/pru0.o $(OUT)pru0.o.discard
	$(Q)$(CC) $(CFLAGS_pru0.elf) $(OUT)pru0.o -o $@
	# in previous step linker did loose info about INTC during optimization, restoring it here (P.S. pru.lds is not helping)
	$(Q)$(OBJCOPY) --add-section '.pru_irq_map'=$(OUT)pru0.pru_irq_map.bin $@

$(OUT)pru1.elf: $(OUT)klipper.elf
	@echo "  Linking $@"
	$(Q)$(CC) $(CFLAGS_pru1.elf) $< -o $@

flash: $(OUT)pru0.elf $(OUT)pru1.elf
	@echo "  Flashing"
	$(Q)sudo ./scripts/flash-pru.sh

endif
