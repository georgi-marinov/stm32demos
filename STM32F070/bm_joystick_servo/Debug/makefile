################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
bm_joystick_servo.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
bm_joystick_servo.list \

OBJCOPY_HEX += \
bm_joystick_servo.hex \

OBJCOPY_BIN += \
bm_joystick_servo.bin \


# All Target
all: bm_joystick_servo.elf secondary-outputs

# Tool invocations
bm_joystick_servo.elf: $(OBJS) $(USER_OBJS) D:\Projects\STMDemos\github\STM32F070\bm_joystick_servo\STM32F070F6PX_FLASH.ld
	arm-none-eabi-gcc -o "bm_joystick_servo.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"D:\Projects\STMDemos\github\STM32F070\bm_joystick_servo\STM32F070F6PX_FLASH.ld" --specs=nosys.specs -Wl,-Map="bm_joystick_servo.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

bm_joystick_servo.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "bm_joystick_servo.list"
	@echo 'Finished building: $@'
	@echo ' '

bm_joystick_servo.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "bm_joystick_servo.hex"
	@echo 'Finished building: $@'
	@echo ' '

bm_joystick_servo.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "bm_joystick_servo.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
