################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/I2C_backup/I2C.c 

OBJS += \
./Core/I2C_backup/I2C.o 

C_DEPS += \
./Core/I2C_backup/I2C.d 


# Each subdirectory must supply rules for building sources it contributes
Core/I2C_backup/%.o Core/I2C_backup/%.su Core/I2C_backup/%.cyclo: ../Core/I2C_backup/%.c Core/I2C_backup/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-I2C_backup

clean-Core-2f-I2C_backup:
	-$(RM) ./Core/I2C_backup/I2C.cyclo ./Core/I2C_backup/I2C.d ./Core/I2C_backup/I2C.o ./Core/I2C_backup/I2C.su

.PHONY: clean-Core-2f-I2C_backup

