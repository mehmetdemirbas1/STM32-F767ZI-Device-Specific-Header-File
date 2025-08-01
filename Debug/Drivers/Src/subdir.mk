################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/EXTI.c \
../Drivers/Src/GPIO.c \
../Drivers/Src/RCC.c \
../Drivers/Src/SPI.c 

OBJS += \
./Drivers/Src/EXTI.o \
./Drivers/Src/GPIO.o \
./Drivers/Src/RCC.o \
./Drivers/Src/SPI.o 

C_DEPS += \
./Drivers/Src/EXTI.d \
./Drivers/Src/GPIO.d \
./Drivers/Src/RCC.d \
./Drivers/Src/SPI.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32 -DNUCLEO_F767ZI -DSTM32F7 -DSTM32F767ZITx -c -I../Inc -I"D:/STM32/WorkSpace0x1/STM32-F767ZI-Device-Specific-Header-File/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/EXTI.cyclo ./Drivers/Src/EXTI.d ./Drivers/Src/EXTI.o ./Drivers/Src/EXTI.su ./Drivers/Src/GPIO.cyclo ./Drivers/Src/GPIO.d ./Drivers/Src/GPIO.o ./Drivers/Src/GPIO.su ./Drivers/Src/RCC.cyclo ./Drivers/Src/RCC.d ./Drivers/Src/RCC.o ./Drivers/Src/RCC.su ./Drivers/Src/SPI.cyclo ./Drivers/Src/SPI.d ./Drivers/Src/SPI.o ./Drivers/Src/SPI.su

.PHONY: clean-Drivers-2f-Src

