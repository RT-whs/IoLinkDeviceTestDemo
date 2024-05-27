################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
F:/WHS/Projects/2023/PCB_Chapadlo/IO_LINK/Project/sw/nucleo_example/en.stsw-iod003/STSW-IOD003_V1.0.0/Drivers/BSP/STEVAL-IOD003V1/steval_iod003v1.c 

C_DEPS += \
./BSP/STEVAL-IOD003V1/steval_iod003v1.d 

OBJS += \
./BSP/STEVAL-IOD003V1/steval_iod003v1.o 


# Each subdirectory must supply rules for building sources it contributes
BSP/STEVAL-IOD003V1/steval_iod003v1.o: F:/WHS/Projects/2023/PCB_Chapadlo/IO_LINK/Project/sw/nucleo_example/en.stsw-iod003/STSW-IOD003_V1.0.0/Drivers/BSP/STEVAL-IOD003V1/steval_iod003v1.c BSP/STEVAL-IOD003V1/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BSP-2f-STEVAL-2d-IOD003V1

clean-BSP-2f-STEVAL-2d-IOD003V1:
	-$(RM) ./BSP/STEVAL-IOD003V1/steval_iod003v1.d ./BSP/STEVAL-IOD003V1/steval_iod003v1.o ./BSP/STEVAL-IOD003V1/steval_iod003v1.su

.PHONY: clean-BSP-2f-STEVAL-2d-IOD003V1

