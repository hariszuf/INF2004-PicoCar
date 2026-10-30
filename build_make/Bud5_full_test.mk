# Separate micro-T-Kernel driving test. Run from build_make/.
# Original Buddy 1/2/3/5 source files and main/config are not edited.
SMP ?= 0
CONSOLE ?= usb_cdc
HEADLESS ?= 0
TIGHT_COURSE ?= 0
WIFI ?= none
ifeq ($(HEADLESS),1)
override CONSOLE := none
endif
ifneq ($(SMP),0)
$(error This driving test supports SMP=0 only)
endif
ifeq ($(HEADLESS),0)
ifneq ($(CONSOLE),usb_cdc)
$(error Use CONSOLE=usb_cdc, or set HEADLESS=1 for power-bank operation)
endif
endif
# Optional existing Buddy 1 radio profile uses the original makefile gates.
override BUDDY1_ONLY := 1
override PROFILE_ID = buddy5-open-auto20-v6-smp0-headless$(HEADLESS)-tight$(TIGHT_COURSE)-tick1-wifi$(WIFI)-join$(WIFI_JOIN)-netif$(WIFI_NETIF)-static$(WIFI_STATIC)-dhcp$(WIFI_DHCP)-dns$(WIFI_DNS)-udp$(WIFI_UDP)-tcp$(WIFI_TCP)-bulk$(WIFI_TCPBULK)-mqtt$(WIFI_MQTT)
TIGHT_SUFFIX :=
ifeq ($(TIGHT_COURSE),1)
TIGHT_SUFFIX := _tight
endif
ifeq ($(HEADLESS),1)
override EXE_FILE := picocar_full_test_headless$(TIGHT_SUFFIX)
else
override EXE_FILE := picocar_full_test$(TIGHT_SUFFIX)
endif
B5_FULL_SRCS := $(wildcard ../app_program/buddy5_full/*.c)
ifeq ($(HEADLESS),1)
B5_FULL_SRCS := $(filter-out ../app_program/buddy5_full/usb_input.c,$(B5_FULL_SRCS))
endif
override TEMP_SRCS := $(wildcard ../app_program/buddy1/*.c) \
 ../app_program/buddy2/motion_control.c ../app_program/buddy2/motion_pid.c \
 ../app_program/buddy2/motion_motor.c ../app_program/buddy2/motion_encoder.c \
 ../app_program/buddy3/line_follow.c ../app_program/buddy3/line_sensors.c \
 $(B5_FULL_SRCS)
include makefile
INCPATH := -I"../app_program/buddy5_full/rtos_config" $(INCPATH) \
 -I"../app_program/buddy2" -I"../app_program/buddy3"
CFLAGS += -DB5_FULL_TEST=1 -UTM_CONSOLE_UART -DTM_CONSOLE_UART=0 \
 -DMOTION_MIN_DUTY=0.10f -DMOTION_BREAKAWAY_DUTY=0.22f \
 -DMOTION_FEEDFORWARD_DUTY_PER_CM_S=0.0125f \
 -DMOTION_STALL_DUTY=0.6f -DMOTION_STALL_TIME_MS=1500u
ifeq ($(HEADLESS),1)
CFLAGS += -DB5_HEADLESS=1 -DB5_BOOT_DELAY_MS=300u -DB5_READY_DELAY_MS=300u
endif
ifeq ($(TIGHT_COURSE),1)
CFLAGS += -DB5_TIGHT_COURSE=1
endif
ASFLAGS += -DB5_FULL_TEST=1 -UTM_CONSOLE_UART -DTM_CONSOLE_UART=0
ifeq ($(HEADLESS),1)
LFLAGS += -Wl,--wrap=__errno
else
LFLAGS += -Wl,--wrap=tm_tusb_discard_input -Wl,--wrap=__errno
endif
# Only these original Pico SDK-facing files see the small RTOS compatibility API.
B5_COMPAT_OBJS := mtkernel_3/app_program/buddy2/motion_control.o \
 mtkernel_3/app_program/buddy2/motion_motor.o \
 mtkernel_3/app_program/buddy2/motion_encoder.o \
 mtkernel_3/app_program/buddy3/line_sensors.o
$(B5_COMPAT_OBJS): INCPATH := -I"../app_program/buddy5_full/compat" $(INCPATH)
# Resolve Arm libm and link it after object files (motion profiles use sqrtf).
B5_LIBM := $(shell $(GCC) -mcpu=cortex-m0plus -mthumb -print-file-name=libm.a)
# Target-specific so the inherited clean recipe can never delete the SDK archive.
$(EXE_FILE).elf: OBJS += $(B5_LIBM)
