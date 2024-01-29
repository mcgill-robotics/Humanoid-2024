################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/microros_transports/dma_transport.c \
../Core/Src/microros_transports/it_transport.c \
../Core/Src/microros_transports/udp_transport.c \
../Core/Src/microros_transports/usb_cdc_transport.c 

OBJS += \
./Core/Src/microros_transports/dma_transport.o \
./Core/Src/microros_transports/it_transport.o \
./Core/Src/microros_transports/udp_transport.o \
./Core/Src/microros_transports/usb_cdc_transport.o 

C_DEPS += \
./Core/Src/microros_transports/dma_transport.d \
./Core/Src/microros_transports/it_transport.d \
./Core/Src/microros_transports/udp_transport.d \
./Core/Src/microros_transports/usb_cdc_transport.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/microros_transports/%.o Core/Src/microros_transports/%.su Core/Src/microros_transports/%.cyclo: ../Core/Src/microros_transports/%.c Core/Src/microros_transports/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -Imicro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../../Middlewares/Third_Party/LwIP/src/include -I../../Middlewares/Third_Party/LwIP/system -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Drivers/BSP/Components/lan8742 -I../../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../../Middlewares/Third_Party/LwIP/src/include/lwip -I../../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../../Middlewares/Third_Party/LwIP/src/include/netif -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-microros_transports

clean-Core-2f-Src-2f-microros_transports:
	-$(RM) ./Core/Src/microros_transports/dma_transport.cyclo ./Core/Src/microros_transports/dma_transport.d ./Core/Src/microros_transports/dma_transport.o ./Core/Src/microros_transports/dma_transport.su ./Core/Src/microros_transports/it_transport.cyclo ./Core/Src/microros_transports/it_transport.d ./Core/Src/microros_transports/it_transport.o ./Core/Src/microros_transports/it_transport.su ./Core/Src/microros_transports/udp_transport.cyclo ./Core/Src/microros_transports/udp_transport.d ./Core/Src/microros_transports/udp_transport.o ./Core/Src/microros_transports/udp_transport.su ./Core/Src/microros_transports/usb_cdc_transport.cyclo ./Core/Src/microros_transports/usb_cdc_transport.d ./Core/Src/microros_transports/usb_cdc_transport.o ./Core/Src/microros_transports/usb_cdc_transport.su

.PHONY: clean-Core-2f-Src-2f-microros_transports

