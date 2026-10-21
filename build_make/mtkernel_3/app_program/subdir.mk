################################################################################
# micro T-Kernel 3.0 BSP - Application Sources
################################################################################

# Buddy 1 development profile:
# Compile only shared application files and Buddy 1.
# Default remains the complete application source tree.

ifeq ($(BUDDY1_ONLY),1)
TEMP_SRCS = $(wildcard ../app_program/*.c) \
            $(wildcard ../app_program/buddy1/*.c)
else
TEMP_SRCS = $(wildcard ../app_program/*.c) \
            $(wildcard ../app_program/*/*.c)
endif

TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '