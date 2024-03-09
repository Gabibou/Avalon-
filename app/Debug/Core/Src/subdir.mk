################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/adc_voltage_current_measuremet.c \
../Core/Src/app_freertos.c \
../Core/Src/at25x041b.c \
../Core/Src/bmp390.c \
../Core/Src/bno055.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/l80-m39.c \
../Core/Src/lora_wioe5.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/propulsion_and_control.c \
../Core/Src/spi.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_hal_timebase_tim.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/adc_voltage_current_measuremet.o \
./Core/Src/app_freertos.o \
./Core/Src/at25x041b.o \
./Core/Src/bmp390.o \
./Core/Src/bno055.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/l80-m39.o \
./Core/Src/lora_wioe5.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/propulsion_and_control.o \
./Core/Src/spi.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_hal_timebase_tim.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/adc_voltage_current_measuremet.d \
./Core/Src/app_freertos.d \
./Core/Src/at25x041b.d \
./Core/Src/bmp390.d \
./Core/Src/bno055.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/l80-m39.d \
./Core/Src/lora_wioe5.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/propulsion_and_control.d \
./Core/Src/spi.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_hal_timebase_tim.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/33768/STM32CubeIDE/workspace_1.11.0/AVALON_PRJ/Avalon-/app/Drivers/CMSIS/DSP" -I"C:/Users/33768/STM32CubeIDE/workspace_1.11.0/AVALON_PRJ/Avalon-/app/Drivers/CMSIS/DSP/Include" -I"C:/Users/33768/STM32CubeIDE/workspace_1.11.0/AVALON_PRJ/Avalon-/app/Drivers/CMSIS/Lib" -I"C:/Users/33768/STM32CubeIDE/workspace_1.11.0/AVALON_PRJ/Avalon-/app/Drivers/CMSIS/Lib/ARM" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/adc_voltage_current_measuremet.cyclo ./Core/Src/adc_voltage_current_measuremet.d ./Core/Src/adc_voltage_current_measuremet.o ./Core/Src/adc_voltage_current_measuremet.su ./Core/Src/app_freertos.cyclo ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/app_freertos.su ./Core/Src/at25x041b.cyclo ./Core/Src/at25x041b.d ./Core/Src/at25x041b.o ./Core/Src/at25x041b.su ./Core/Src/bmp390.cyclo ./Core/Src/bmp390.d ./Core/Src/bmp390.o ./Core/Src/bmp390.su ./Core/Src/bno055.cyclo ./Core/Src/bno055.d ./Core/Src/bno055.o ./Core/Src/bno055.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/l80-m39.cyclo ./Core/Src/l80-m39.d ./Core/Src/l80-m39.o ./Core/Src/l80-m39.su ./Core/Src/lora_wioe5.cyclo ./Core/Src/lora_wioe5.d ./Core/Src/lora_wioe5.o ./Core/Src/lora_wioe5.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/propulsion_and_control.cyclo ./Core/Src/propulsion_and_control.d ./Core/Src/propulsion_and_control.o ./Core/Src/propulsion_and_control.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_hal_timebase_tim.cyclo ./Core/Src/stm32g4xx_hal_timebase_tim.d ./Core/Src/stm32g4xx_hal_timebase_tim.o ./Core/Src/stm32g4xx_hal_timebase_tim.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

