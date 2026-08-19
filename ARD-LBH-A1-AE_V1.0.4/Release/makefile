################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include App/Queue/subdir.mk
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
ARD-LBH-C-4CB_V1.2.0.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
ARD-LBH-C-4CB_V1.2.0.list \

OBJCOPY_BIN += \
ARD-LBH-C-4CB_V1.2.0.bin \


# All Target
all: ARD-LBH-C-4CB_V1.2.0.elf secondary-outputs

# Tool invocations
ARD-LBH-C-4CB_V1.2.0.elf: $(OBJS) $(USER_OBJS) F:\AREED_FILE\007\ NEW\20\ MDR-LBH\02\ LBH_C_V1.1.0\ARD-LBH-C-4CB_V1.2.0\STM32F103VETX_FLASH.ld
	arm-none-eabi-gcc -o "ARD-LBH-C-4CB_V1.2.0.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"F:\AREED_FILE\007 NEW\20 MDR-LBH\02 LBH_C_V1.1.0\ARD-LBH-C-4CB_V1.2.0\STM32F103VETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="ARD-LBH-C-4CB_V1.2.0.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

ARD-LBH-C-4CB_V1.2.0.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "ARD-LBH-C-4CB_V1.2.0.list"
	@echo 'Finished building: $@'
	@echo ' '

ARD-LBH-C-4CB_V1.2.0.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "ARD-LBH-C-4CB_V1.2.0.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
