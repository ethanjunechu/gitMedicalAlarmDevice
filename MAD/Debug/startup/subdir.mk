################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32f103xe.s 

C_SRCS += \
../startup/syscalls.c 

OBJS += \
./startup/startup_stm32f103xe.o \
./startup/syscalls.o 

C_DEPS += \
./startup/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

startup/%.o: ../startup/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F103xE -I"C:/Users/Ethan/STM32/MAD/Inc" -I"C:/Users/Ethan/STM32/MAD/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Ethan/STM32/MAD/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Ethan/STM32/MAD/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Ethan/STM32/MAD/Drivers/CMSIS/Include" -I"C:/Users/Ethan/STM32/MAD/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


