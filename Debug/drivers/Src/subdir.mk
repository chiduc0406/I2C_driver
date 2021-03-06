################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f407xx_i2c_driver.c \
../drivers/Src/stm32f407xx_rcc_driver.c 

OBJS += \
./drivers/Src/stm32f407xx_i2c_driver.o \
./drivers/Src/stm32f407xx_rcc_driver.o 

C_DEPS += \
./drivers/Src/stm32f407xx_i2c_driver.d \
./drivers/Src/stm32f407xx_rcc_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F407ZETx -DSTM32 -DSTM32F4 -c -I../Inc -I"D:/Phong/TAI_LIEU/stm32f407xx_driver/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f407xx_i2c_driver.d ./drivers/Src/stm32f407xx_i2c_driver.o ./drivers/Src/stm32f407xx_rcc_driver.d ./drivers/Src/stm32f407xx_rcc_driver.o

.PHONY: clean-drivers-2f-Src

