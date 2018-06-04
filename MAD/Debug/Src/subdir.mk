################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/bsp_button.c \
../Src/bsp_ds1302.c \
../Src/bsp_led.c \
../Src/bsp_rs485.c \
../Src/bsp_spi_flash.c \
../Src/bsp_usart_lcd.c \
../Src/dma.c \
../Src/gpio.c \
../Src/main.c \
../Src/spi.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c \
../Src/tim.c \
../Src/usart.c 

OBJS += \
./Src/adc.o \
./Src/bsp_button.o \
./Src/bsp_ds1302.o \
./Src/bsp_led.o \
./Src/bsp_rs485.o \
./Src/bsp_spi_flash.o \
./Src/bsp_usart_lcd.o \
./Src/dma.o \
./Src/gpio.o \
./Src/main.o \
./Src/spi.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o \
./Src/tim.o \
./Src/usart.o 

C_DEPS += \
./Src/adc.d \
./Src/bsp_button.d \
./Src/bsp_ds1302.d \
./Src/bsp_led.d \
./Src/bsp_rs485.d \
./Src/bsp_spi_flash.d \
./Src/bsp_usart_lcd.d \
./Src/dma.d \
./Src/gpio.d \
./Src/main.d \
./Src/spi.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d \
./Src/tim.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F103xE -I"C:/Users/Ethan/STM32/MAD/Inc" -I"C:/Users/Ethan/STM32/MAD/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Ethan/STM32/MAD/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Ethan/STM32/MAD/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Ethan/STM32/MAD/Drivers/CMSIS/Include" -I"C:/Users/Ethan/STM32/MAD/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


