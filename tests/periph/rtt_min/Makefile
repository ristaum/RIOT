BOARD ?= samr21-xpro
include ../Makefile.periph_common

USEMODULE += xtimer

FEATURES_REQUIRED += periph_rtt
DISABLE_MODULE += periph_init_rtt

ifeq (native, $(BOARD))
  RIOT_TERMINAL ?= native
endif
RIOT_TERMINAL ?= socat

# microbit qemu lacks rtt
TEST_ON_CI_BLACKLIST += microbit

include $(RIOTBASE)/Makefile.include

# use highest possible RTT_FREQUENCY for boards that allow it
ifneq (,$(filter stm32 nrf5% sam% kinetis efm32 fe310,$(CPU)))
  RTT_FREQUENCY ?= RTT_MAX_FREQUENCY
  CFLAGS += -DRTT_FREQUENCY=$(RTT_FREQUENCY)
endif

SAMPLES ?= 1024
CFLAGS += -DSAMPLES=$(SAMPLES)
