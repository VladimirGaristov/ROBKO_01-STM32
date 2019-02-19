################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main.c \
../src/motor_control.c \
../src/stm32l4xx_it.c \
../src/syscalls.c \
../src/system_stm32l4xx.c \
../src/test_functions.c 

OBJS += \
./src/main.o \
./src/motor_control.o \
./src/stm32l4xx_it.o \
./src/syscalls.o \
./src/system_stm32l4xx.o \
./src/test_functions.o 

C_DEPS += \
./src/main.d \
./src/motor_control.d \
./src/stm32l4xx_it.d \
./src/syscalls.d \
./src/system_stm32l4xx.d \
./src/test_functions.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32L4 -DSTM32L476RGTx -DNUCLEO_L476RG -DSTM32 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/Vlado/workspace/ROBKO_01-STM32/MCU_Firmware/inc" -I"C:/Users/Vlado/workspace/ROBKO_01-STM32/MCU_Firmware/HAL_Driver/Inc" -I"C:/Users/Vlado/workspace/ROBKO_01-STM32/MCU_Firmware/HAL_Driver/Inc/Legacy" -I"C:/Users/Vlado/workspace/ROBKO_01-STM32/MCU_Firmware/Utilities/STM32L4xx_Nucleo" -I"C:/Users/Vlado/workspace/ROBKO_01-STM32/MCU_Firmware/CMSIS/core" -I"C:/Users/Vlado/workspace/ROBKO_01-STM32/MCU_Firmware/CMSIS/device" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


