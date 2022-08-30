################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../autogen/sl_board_default_init.c \
../autogen/sl_device_init_clocks.c \
../autogen/sl_emlib_gpio_simple_init.c \
../autogen/sl_event_handler.c \
../autogen/sl_pwm_init.c 

OBJS += \
./autogen/sl_board_default_init.o \
./autogen/sl_device_init_clocks.o \
./autogen/sl_emlib_gpio_simple_init.o \
./autogen/sl_event_handler.o \
./autogen/sl_pwm_init.o 

C_DEPS += \
./autogen/sl_board_default_init.d \
./autogen/sl_device_init_clocks.d \
./autogen/sl_emlib_gpio_simple_init.d \
./autogen/sl_event_handler.d \
./autogen/sl_pwm_init.d 


# Each subdirectory must supply rules for building sources it contributes
autogen/sl_board_default_init.o: ../autogen/sl_board_default_init.c autogen/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32FG12P433F1024GL125=1' '-DSL_BOARD_NAME="BRD4253A"' '-DSL_BOARD_REV="A03"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"D:\gitRepository\simplicity studio\fly_controller\config" -I"D:\gitRepository\simplicity studio\fly_controller\autogen" -I"D:\gitRepository\simplicity studio\fly_controller" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32FG12P/Include" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Core/Include" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/configuration_over_swo/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/driver/debug/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/init/gpio_simple" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/gpiointerrupt/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/driver/pwm/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/rtcdrv/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/udelay/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/ustimer/inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"autogen/sl_board_default_init.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

autogen/sl_device_init_clocks.o: ../autogen/sl_device_init_clocks.c autogen/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32FG12P433F1024GL125=1' '-DSL_BOARD_NAME="BRD4253A"' '-DSL_BOARD_REV="A03"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"D:\gitRepository\simplicity studio\fly_controller\config" -I"D:\gitRepository\simplicity studio\fly_controller\autogen" -I"D:\gitRepository\simplicity studio\fly_controller" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32FG12P/Include" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Core/Include" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/configuration_over_swo/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/driver/debug/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/init/gpio_simple" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/gpiointerrupt/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/driver/pwm/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/rtcdrv/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/udelay/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/ustimer/inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"autogen/sl_device_init_clocks.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

autogen/sl_emlib_gpio_simple_init.o: ../autogen/sl_emlib_gpio_simple_init.c autogen/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32FG12P433F1024GL125=1' '-DSL_BOARD_NAME="BRD4253A"' '-DSL_BOARD_REV="A03"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"D:\gitRepository\simplicity studio\fly_controller\config" -I"D:\gitRepository\simplicity studio\fly_controller\autogen" -I"D:\gitRepository\simplicity studio\fly_controller" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32FG12P/Include" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Core/Include" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/configuration_over_swo/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/driver/debug/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/init/gpio_simple" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/gpiointerrupt/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/driver/pwm/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/rtcdrv/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/udelay/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/ustimer/inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"autogen/sl_emlib_gpio_simple_init.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

autogen/sl_event_handler.o: ../autogen/sl_event_handler.c autogen/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32FG12P433F1024GL125=1' '-DSL_BOARD_NAME="BRD4253A"' '-DSL_BOARD_REV="A03"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"D:\gitRepository\simplicity studio\fly_controller\config" -I"D:\gitRepository\simplicity studio\fly_controller\autogen" -I"D:\gitRepository\simplicity studio\fly_controller" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32FG12P/Include" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Core/Include" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/configuration_over_swo/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/driver/debug/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/init/gpio_simple" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/gpiointerrupt/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/driver/pwm/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/rtcdrv/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/udelay/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/ustimer/inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"autogen/sl_event_handler.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

autogen/sl_pwm_init.o: ../autogen/sl_pwm_init.c autogen/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32FG12P433F1024GL125=1' '-DSL_BOARD_NAME="BRD4253A"' '-DSL_BOARD_REV="A03"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"D:\gitRepository\simplicity studio\fly_controller\config" -I"D:\gitRepository\simplicity studio\fly_controller\autogen" -I"D:\gitRepository\simplicity studio\fly_controller" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32FG12P/Include" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Core/Include" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/configuration_over_swo/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/driver/debug/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/init/gpio_simple" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/gpiointerrupt/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/driver/pwm/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/rtcdrv/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/service/udelay/inc" -I"C:/Users/KOBIED/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/ustimer/inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"autogen/sl_pwm_init.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


