################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/src/stm324f407xx_spi_driver.c \
../Drivers/src/stm32f407xx_gpio_driver.c \
../Drivers/src/stm32f407xx_i2c_driver.c \
../Drivers/src/stm32f407xx_rcc_driver.c \
../Drivers/src/stm32f407xx_usart_driver.c 

OBJS += \
./Drivers/src/stm324f407xx_spi_driver.o \
./Drivers/src/stm32f407xx_gpio_driver.o \
./Drivers/src/stm32f407xx_i2c_driver.o \
./Drivers/src/stm32f407xx_rcc_driver.o \
./Drivers/src/stm32f407xx_usart_driver.o 

C_DEPS += \
./Drivers/src/stm324f407xx_spi_driver.d \
./Drivers/src/stm32f407xx_gpio_driver.d \
./Drivers/src/stm32f407xx_i2c_driver.d \
./Drivers/src/stm32f407xx_rcc_driver.d \
./Drivers/src/stm32f407xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/src/%.o: ../Drivers/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F407G_DISC1 -DDEBUG -I"C:/Users/tahay/Desktop/Programs/stm/openstm32workplace/STM32SW_workplace/stm32f4xx_drivers/Drivers/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


