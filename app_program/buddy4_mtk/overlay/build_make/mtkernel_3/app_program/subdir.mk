################################################################################
# micro T-Kernel 3.0 BSP - Application Sources
################################################################################

# Buddy 1 development profile:
# Compile only shared application files and Buddy 1.
# Default builds the registered RTOS subsystems; standalone SDK programs are excluded.

ifeq ($(BUDDY1_ONLY),1)
TEMP_SRCS = $(wildcard ../app_program/*.c) \
            $(wildcard ../app_program/buddy1/*.c)
else
# Final RTOS entry plus existing Buddy 1/5 tasks. Buddy 2/3 remain standalone
# until their owners provide task adapters; never glob SDK mains or host tests.
TEMP_SRCS = $(wildcard ../app_program/*.c) \
            $(wildcard ../app_program/buddy1/*.c) \
            $(wildcard ../app_program/buddy5/*.c)
ifeq ($(BUDDY4_ENABLED),1)
TEMP_SRCS += ../app_program/buddy4/buddy4_imu.c \
             ../app_program/buddy4/buddy4_impact.c \
             ../app_program/buddy4/buddy4_motion.c \
             ../app_program/buddy4/buddy4_encoder_yaw.c \
             ../app_program/buddy4/buddy4_device.c \
             ../app_program/buddy4/buddy4_task.c
endif
endif

TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

mtkernel_3/app_program/%.o: ../app_program/%.c
	@mkdir -p $(@D)
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
