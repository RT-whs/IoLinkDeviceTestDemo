################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
F:/WHS/Projects/2023/PCB_Chapadlo/IO_LINK/Project/sw/nucleo_example/en.stsw-iod003/STSW-IOD003_V1.0.0/Drivers/BSP/Components/l6362a/l6362a.c 

C_DEPS += \
./Drivers/BSP/Components/l6362a/l6362a.d 

OBJS += \
./Drivers/BSP/Components/l6362a/l6362a.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/l6362a/l6362a.o: F:/WHS/Projects/2023/PCB_Chapadlo/IO_LINK/Project/sw/nucleo_example/en.stsw-iod003/STSW-IOD003_V1.0.0/Drivers/BSP/Components/l6362a/l6362a.c Drivers/BSP/Components/l6362a/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-l6362a

clean-Drivers-2f-BSP-2f-Components-2f-l6362a:
	-$(RM) ./Drivers/BSP/Components/l6362a/l6362a.d ./Drivers/BSP/Components/l6362a/l6362a.o ./Drivers/BSP/Components/l6362a/l6362a.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-l6362a

