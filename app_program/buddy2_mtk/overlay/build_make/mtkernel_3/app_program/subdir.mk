################################################################################
# micro T-Kernel 3.0 BSP - Application Sources
# Buddy 2 micro T-Kernel trial (app_program/buddy2_mtk overlay)
################################################################################

# Sources are listed explicitly: the buddy folders also hold Pico SDK (CMake)
# programs, SDK-only drivers and host tests that must never be compiled into
# the micro T-Kernel image. This trial builds Buddy 1 + Buddy 2 only.

# BUDDY1_ONLY is ignored here: this image always contains Buddy 2.
TEMP_SRCS = $(wildcard ../app_program/*.c) \
            $(wildcard ../app_program/buddy1/*.c) \
            ../app_program/buddy2/motion_control.c \
            ../app_program/buddy2/motion_pid.c \
            ../app_program/buddy2/mtk/motion_mtk_hw.c \
            ../app_program/buddy2/mtk/motion_motor_mtk.c \
            ../app_program/buddy2/mtk/motion_encoder_mtk.c \
            ../app_program/buddy2/mtk/motion_task_mtk.c \
            ../app_program/buddy2/mtk/motion_selftest_mtk.c

# Buddy 2 headers are included by name ("motion_control.h" ...).
APP_INCPATH = -I"../app_program/buddy2" -I"../app_program/buddy2/mtk"
APP_CFLAGS  = -DMOTION_USE_UTKERNEL

TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

mtkernel_3/app_program/%.o: ../app_program/%.c
	@mkdir -p $(@D)
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) $(APP_CFLAGS) -D$(TARGET) $(INCPATH) $(APP_INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
