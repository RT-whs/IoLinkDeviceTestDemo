################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
F:/WHS/Projects/2023/PCB_Chapadlo/IO_LINK/Project/sw/nucleo_example/en.stsw-iod003/STSW-IOD003_V1.0.0/Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.c 

C_DEPS += \
./BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.d 

OBJS += \
./BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.o 


# Each subdirectory must supply rules for building sources it contributes
BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.o: F:/WHS/Projects/2023/PCB_Chapadlo/IO_LINK/Project/sw/nucleo_example/en.stsw-iod003/STSW-IOD003_V1.0.0/Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.c BSP/STM32F4xx-Nucleo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BSP-2f-STM32F4xx-2d-Nucleo

clean-BSP-2f-STM32F4xx-2d-Nucleo:
	-$(RM) ./BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.d ./BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.o ./BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.su

.PHONY: clean-BSP-2f-STM32F4xx-2d-Nucleo

