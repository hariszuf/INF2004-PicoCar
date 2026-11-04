# PicoCar micro-T-Kernel integration. Run from build_make/.
# Buddy 2/3 own motion/line, Buddy 4 owns IMU observations, Buddy 5 owns scanning.
# Set TELEMETRY_MQTT=1 to add Buddy 1 networking to the driving profile.
SMP ?= 0
CONSOLE ?= usb_cdc
HEADLESS ?= 0
TIGHT_COURSE ?= 0
TELEMETRY_MQTT ?= 0
ifeq ($(TELEMETRY_MQTT),1)
WIFI ?= cyw43
WIFI_JOIN ?= 1
WIFI_NETIF ?= 1
WIFI_DHCP ?= 1
WIFI_DNS ?= 1
WIFI_MQTT ?= 1
endif
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
override PROFILE_ID = picocar-integration-v2-smp0-headless$(HEADLESS)-tight$(TIGHT_COURSE)-tick1-wifi$(WIFI)-join$(WIFI_JOIN)-netif$(WIFI_NETIF)-static$(WIFI_STATIC)-dhcp$(WIFI_DHCP)-dns$(WIFI_DNS)-udp$(WIFI_UDP)-tcp$(WIFI_TCP)-bulk$(WIFI_TCPBULK)-mqtt$(WIFI_MQTT)
TIGHT_SUFFIX :=
ifeq ($(TIGHT_COURSE),1)
TIGHT_SUFFIX := _tight
endif
ifeq ($(WIFI_MQTT),1)
TIGHT_SUFFIX := $(TIGHT_SUFFIX)_mqtt
endif
ifeq ($(HEADLESS),1)
override EXE_FILE := picocar_integration_headless$(TIGHT_SUFFIX)
else
override EXE_FILE := picocar_integration$(TIGHT_SUFFIX)
endif
B1_SRCS := $(wildcard ../app_program/buddy1/*.c)
B2_SRCS := ../app_program/buddy2/motion_control.c \
 ../app_program/buddy2/motion_pid.c ../app_program/buddy2/motion_motor.c \
 ../app_program/buddy2/motion_encoder.c
B3_SRCS := ../app_program/buddy3/line_follow.c ../app_program/buddy3/line_sensors.c
ifeq ($(CONSOLE),usb_cdc)
B3_SRCS += ../app_program/buddy3/barcode_scan.c ../app_program/buddy3/barcode39.c
endif
B4_CORE_SRCS := ../app_program/buddy4/buddy4_imu.c \
 ../app_program/buddy4/buddy4_impact.c ../app_program/buddy4/buddy4_motion.c \
 ../app_program/buddy4/buddy4_encoder_yaw.c
B4_SRCS := $(wildcard ../app_program/buddy4/rtos/*.c)
B5_SRCS := $(wildcard ../app_program/buddy5/rtos/*.c)
SHARED_SRCS := $(wildcard ../app_program/buddy5/vehicle/*.c)
ifeq ($(HEADLESS),1)
SHARED_SRCS := $(filter-out ../app_program/buddy5/vehicle/usb_input.c,$(SHARED_SRCS))
endif
INTEGRATION_OBJS := $(patsubst ../%,mtkernel_3/%,$(B4_CORE_SRCS:.c=.o) $(B4_SRCS:.c=.o) $(B5_SRCS:.c=.o) $(SHARED_SRCS:.c=.o))
override TEMP_SRCS := $(B1_SRCS) $(B2_SRCS) $(B3_SRCS) $(B4_CORE_SRCS) $(B4_SRCS) $(B5_SRCS) $(SHARED_SRCS)
include makefile
# The generic application pattern rule does not create nested object folders.
.PHONY: b5_integration_dirs
b5_integration_dirs:
	@mkdir -p mtkernel_3/app_program/buddy4 mtkernel_3/app_program/buddy4/rtos mtkernel_3/app_program/buddy5/rtos mtkernel_3/app_program/buddy5/vehicle
$(INTEGRATION_OBJS): | b5_integration_dirs
INCPATH := -I"../app_program/buddy5/vehicle/rtos_config" $(INCPATH) \
 -I"../app_program/buddy5/vehicle" -I"../app_program/buddy4/rtos" \
 -I"../app_program/buddy4" -I"../app_program/buddy5/rtos" \
 -I"../app_program/buddy2" -I"../app_program/buddy3"
CFLAGS += -DB5_FULL_TEST=1 -UTM_CONSOLE_UART -DTM_CONSOLE_UART=0 \
 -DMOTION_MIN_DUTY=0.10f -DMOTION_BREAKAWAY_DUTY=0.22f \
 -DMOTION_FEEDFORWARD_DUTY_PER_CM_S=0.0125f \
 -DMOTION_STALL_DUTY=0.6f -DMOTION_STALL_TIME_MS=1500u \
 -DMOTION_TURN_SPEED_CM_S=7.0f
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
$(B5_COMPAT_OBJS): INCPATH := -I"../app_program/buddy5/vehicle/compat" $(INCPATH)
# Resolve Arm libm and link it after object files (motion profiles use sqrtf).
B5_LIBM := $(shell $(GCC) -mcpu=cortex-m0plus -mthumb -print-file-name=libm.a)
# Target-specific so the inherited clean recipe can never delete the SDK archive.
$(EXE_FILE).elf: OBJS += $(B5_LIBM)
