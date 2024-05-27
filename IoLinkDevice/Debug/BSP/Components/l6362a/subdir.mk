################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
F:/WHS/Projects/2023/PCB_Chapadlo/IO_LINK/Project/sw/nucleo_example/en.stsw-iod003/STSW-IOD003_V1.0.0/Drivers/BSP/Components/l6362a/l6362a.c 

C_DEPS += \
./BSP/Components/l6362a/l6362a.d 

OBJS += \
./BSP/Components/l6362a/l6362a.o 


# Each subdirectory must supply rules for building sources it contributes
BSP/Components/l6362a/l6362a.o: F:/WHS/Projects/2023/PCB_Chapadlo/IO_LINK/Project/sw/nucleo_example/en.stsw-iod003/STSW-IOD003_V1.0.0/Drivers/BSP/Components/l6362a/l6362a.c BSP/Components/l6362a/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BSP-2f-Components-2f-l6362a

clean-BSP-2f-Components-2f-l6362a:
	-$(RM) ./BSP/Components/l6362a/l6362a.d ./BSP/Components/l6362a/l6362a.o ./BSP/Components/l6362a/l6362a.su

.PHONY: clean-BSP-2f-Components-2f-l6362a

