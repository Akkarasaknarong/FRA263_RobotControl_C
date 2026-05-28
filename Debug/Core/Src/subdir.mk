################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/cascade.c \
../Core/Src/disturbance_feedforward.c \
../Core/Src/kalman_filter.c \
../Core/Src/main.c \
../Core/Src/qei.c \
../Core/Src/quintic_traj.c \
../Core/Src/reference_feedforward.c \
../Core/Src/serial_frame.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c 

OBJS += \
./Core/Src/cascade.o \
./Core/Src/disturbance_feedforward.o \
./Core/Src/kalman_filter.o \
./Core/Src/main.o \
./Core/Src/qei.o \
./Core/Src/quintic_traj.o \
./Core/Src/reference_feedforward.o \
./Core/Src/serial_frame.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o 

C_DEPS += \
./Core/Src/cascade.d \
./Core/Src/disturbance_feedforward.d \
./Core/Src/kalman_filter.d \
./Core/Src/main.d \
./Core/Src/qei.d \
./Core/Src/quintic_traj.d \
./Core/Src/reference_feedforward.d \
./Core/Src/serial_frame.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/cascade.cyclo ./Core/Src/cascade.d ./Core/Src/cascade.o ./Core/Src/cascade.su ./Core/Src/disturbance_feedforward.cyclo ./Core/Src/disturbance_feedforward.d ./Core/Src/disturbance_feedforward.o ./Core/Src/disturbance_feedforward.su ./Core/Src/kalman_filter.cyclo ./Core/Src/kalman_filter.d ./Core/Src/kalman_filter.o ./Core/Src/kalman_filter.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/qei.cyclo ./Core/Src/qei.d ./Core/Src/qei.o ./Core/Src/qei.su ./Core/Src/quintic_traj.cyclo ./Core/Src/quintic_traj.d ./Core/Src/quintic_traj.o ./Core/Src/quintic_traj.su ./Core/Src/reference_feedforward.cyclo ./Core/Src/reference_feedforward.d ./Core/Src/reference_feedforward.o ./Core/Src/reference_feedforward.su ./Core/Src/serial_frame.cyclo ./Core/Src/serial_frame.d ./Core/Src/serial_frame.o ./Core/Src/serial_frame.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su

.PHONY: clean-Core-2f-Src

