################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include tests/subdir.mk
-include src/subdir.mk
-include gecko_sdk_4.1.1/platform/service/udelay/src/subdir.mk
-include gecko_sdk_4.1.1/platform/service/system/src/subdir.mk
-include gecko_sdk_4.1.1/platform/service/device_init/src/subdir.mk
-include gecko_sdk_4.1.1/platform/emlib/src/subdir.mk
-include gecko_sdk_4.1.1/platform/emdrv/ustimer/src/subdir.mk
-include gecko_sdk_4.1.1/platform/emdrv/gpiointerrupt/src/subdir.mk
-include gecko_sdk_4.1.1/platform/driver/pwm/src/subdir.mk
-include gecko_sdk_4.1.1/platform/driver/debug/src/subdir.mk
-include gecko_sdk_4.1.1/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_4.1.1/platform/common/src/subdir.mk
-include gecko_sdk_4.1.1/platform/Device/SiliconLabs/EFR32FG12P/Source/subdir.mk
-include gecko_sdk_4.1.1/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_usart/subdir.mk
-include gecko_sdk_4.1.1/hardware/driver/configuration_over_swo/src/subdir.mk
-include gecko_sdk_4.1.1/hardware/board/src/subdir.mk
-include autogen/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := fly_controller
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: main-build

# Main-build Target
main-build: fly_controller.axf

# Tool invocations
fly_controller.axf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "D:\gitRepository\simplicity studio\fly_controller\autogen\linkerfile.ld" -Xlinker --gc-sections -Xlinker -Map="fly_controller.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o fly_controller.axf -Wl,--start-group "./app.o" "./main.o" "./autogen/sl_board_default_init.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_emlib_gpio_simple_init.o" "./autogen/sl_event_handler.o" "./autogen/sl_pwm_init.o" "./gecko_sdk_4.1.1/hardware/board/src/sl_board_control_gpio.o" "./gecko_sdk_4.1.1/hardware/board/src/sl_board_init.o" "./gecko_sdk_4.1.1/hardware/driver/configuration_over_swo/src/sl_cos.o" "./gecko_sdk_4.1.1/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_usart/sl_mx25_flash_shutdown.o" "./gecko_sdk_4.1.1/platform/Device/SiliconLabs/EFR32FG12P/Source/startup_efr32fg12p.o" "./gecko_sdk_4.1.1/platform/Device/SiliconLabs/EFR32FG12P/Source/system_efr32fg12p.o" "./gecko_sdk_4.1.1/platform/common/src/sl_assert.o" "./gecko_sdk_4.1.1/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_4.1.1/platform/driver/debug/src/sl_debug_swo.o" "./gecko_sdk_4.1.1/platform/driver/pwm/src/sl_pwm.o" "./gecko_sdk_4.1.1/platform/emdrv/gpiointerrupt/src/gpiointerrupt.o" "./gecko_sdk_4.1.1/platform/emdrv/ustimer/src/ustimer.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_cmu.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_core.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_emu.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_gpio.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_msc.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_system.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_timer.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_usart.o" "./gecko_sdk_4.1.1/platform/service/device_init/src/sl_device_init_dcdc_s1.o" "./gecko_sdk_4.1.1/platform/service/device_init/src/sl_device_init_emu_s1.o" "./gecko_sdk_4.1.1/platform/service/device_init/src/sl_device_init_hfxo_s1.o" "./gecko_sdk_4.1.1/platform/service/device_init/src/sl_device_init_lfxo_s1.o" "./gecko_sdk_4.1.1/platform/service/device_init/src/sl_device_init_nvic.o" "./gecko_sdk_4.1.1/platform/service/system/src/sl_system_init.o" "./gecko_sdk_4.1.1/platform/service/system/src/sl_system_process_action.o" "./gecko_sdk_4.1.1/platform/service/udelay/src/sl_udelay.o" "./gecko_sdk_4.1.1/platform/service/udelay/src/sl_udelay_armv6m_gcc.o" "./src/main_controller.o" "./src/motorDriverMain.o" "./tests/test_pwm.o" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: fly_controller.hex'
	arm-none-eabi-objcopy -O ihex "fly_controller.axf" "fly_controller.hex"
	@echo ' '

	@echo 'Building bin file: fly_controller.bin'
	arm-none-eabi-objcopy -O binary "fly_controller.axf" "fly_controller.bin"
	@echo ' '

	@echo 'Building s37 file: fly_controller.s37'
	arm-none-eabi-objcopy -O srec "fly_controller.axf" "fly_controller.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "fly_controller.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) fly_controller.axf
	-@echo ' '

.PHONY: all clean dependents main-build

-include ../makefile.targets
