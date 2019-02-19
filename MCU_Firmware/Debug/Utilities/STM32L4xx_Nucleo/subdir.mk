################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/STM32L4xx_Nucleo/stm32l4xx_nucleo.c 

OBJS += \
./Utilities/STM32L4xx_Nucleo/stm32l4xx_nucleo.o 

C_DEPS += \
./Utilities/STM32L4xx_Nucleo/stm32l4xx_nucleo.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/STM32L4xx_Nucleo/%.o: ../Utilities/STM32L4xx_Nucleo/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32L4 -DSTM32L476RGTx -DNUCLEO_L476RG -DSTM32 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/Vlado/workspace/ROBKO_01-STM32/MCU_Firmware/inc" -I"C:/Users/Vlado/workspace/ROBKO_01-STM32/MCU_Firmware/HAL_Driver/Inc" -I"C:/Users/Vlado/workspace/ROBKO_01-STM32/MCU_Firmware/HAL_Driver/Inc/Legacy" -I"C:/Users/Vlado/workspace/ROBKO_01-STM32/MCU_Firmware/Utilities/STM32L4xx_Nucleo" -I"C:/Users/Vlado/workspace/ROBKO_01-STM32/MCU_Firmware/CMSIS/core" -I"C:/Users/Vlado/workspace/ROBKO_01-STM32/MCU_Firmware/CMSIS/device" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


