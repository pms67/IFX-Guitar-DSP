################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CS4270-Codec.c \
../Core/Src/IFX_Butterworth3LPF.c \
../Core/Src/IFX_Chorus.c \
../Core/Src/IFX_Delay.c \
../Core/Src/IFX_DelayLine.c \
../Core/Src/IFX_LogAudioPot.c \
../Core/Src/IFX_MovingRMS.c \
../Core/Src/IFX_NoiseGate.c \
../Core/Src/IFX_Overdrive.c \
../Core/Src/IFX_PeakingFilter.c \
../Core/Src/main.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c 

OBJS += \
./Core/Src/CS4270-Codec.o \
./Core/Src/IFX_Butterworth3LPF.o \
./Core/Src/IFX_Chorus.o \
./Core/Src/IFX_Delay.o \
./Core/Src/IFX_DelayLine.o \
./Core/Src/IFX_LogAudioPot.o \
./Core/Src/IFX_MovingRMS.o \
./Core/Src/IFX_NoiseGate.o \
./Core/Src/IFX_Overdrive.o \
./Core/Src/IFX_PeakingFilter.o \
./Core/Src/main.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o 

C_DEPS += \
./Core/Src/CS4270-Codec.d \
./Core/Src/IFX_Butterworth3LPF.d \
./Core/Src/IFX_Chorus.d \
./Core/Src/IFX_Delay.d \
./Core/Src/IFX_DelayLine.d \
./Core/Src/IFX_LogAudioPot.d \
./Core/Src/IFX_MovingRMS.d \
./Core/Src/IFX_NoiseGate.d \
./Core/Src/IFX_Overdrive.d \
./Core/Src/IFX_PeakingFilter.d \
./Core/Src/main.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H7B0xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/CS4270-Codec.cyclo ./Core/Src/CS4270-Codec.d ./Core/Src/CS4270-Codec.o ./Core/Src/CS4270-Codec.su ./Core/Src/IFX_Butterworth3LPF.cyclo ./Core/Src/IFX_Butterworth3LPF.d ./Core/Src/IFX_Butterworth3LPF.o ./Core/Src/IFX_Butterworth3LPF.su ./Core/Src/IFX_Chorus.cyclo ./Core/Src/IFX_Chorus.d ./Core/Src/IFX_Chorus.o ./Core/Src/IFX_Chorus.su ./Core/Src/IFX_Delay.cyclo ./Core/Src/IFX_Delay.d ./Core/Src/IFX_Delay.o ./Core/Src/IFX_Delay.su ./Core/Src/IFX_DelayLine.cyclo ./Core/Src/IFX_DelayLine.d ./Core/Src/IFX_DelayLine.o ./Core/Src/IFX_DelayLine.su ./Core/Src/IFX_LogAudioPot.cyclo ./Core/Src/IFX_LogAudioPot.d ./Core/Src/IFX_LogAudioPot.o ./Core/Src/IFX_LogAudioPot.su ./Core/Src/IFX_MovingRMS.cyclo ./Core/Src/IFX_MovingRMS.d ./Core/Src/IFX_MovingRMS.o ./Core/Src/IFX_MovingRMS.su ./Core/Src/IFX_NoiseGate.cyclo ./Core/Src/IFX_NoiseGate.d ./Core/Src/IFX_NoiseGate.o ./Core/Src/IFX_NoiseGate.su ./Core/Src/IFX_Overdrive.cyclo ./Core/Src/IFX_Overdrive.d ./Core/Src/IFX_Overdrive.o ./Core/Src/IFX_Overdrive.su ./Core/Src/IFX_PeakingFilter.cyclo ./Core/Src/IFX_PeakingFilter.d ./Core/Src/IFX_PeakingFilter.o ./Core/Src/IFX_PeakingFilter.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su

.PHONY: clean-Core-2f-Src

