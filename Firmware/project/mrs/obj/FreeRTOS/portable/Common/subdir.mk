################################################################################
# MRS Version: 1.9.1
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Administrator1/Desktop/Demo_gyroscope/libraries/FreeRTOS/portable/Common/mpu_wrappers.c 

OBJS += \
./FreeRTOS/portable/Common/mpu_wrappers.o 

C_DEPS += \
./FreeRTOS/portable/Common/mpu_wrappers.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/portable/Common/mpu_wrappers.o: C:/Users/Administrator1/Desktop/Demo_gyroscope/libraries/FreeRTOS/portable/Common/mpu_wrappers.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O1 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\Libraries\doc" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\libraries\FreeRTOS\include" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\libraries\FreeRTOS\portable\Common" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\libraries\FreeRTOS\portable\GCC\RISC-V" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\libraries\FreeRTOS\portable\GCC\RISC-V\chip_specific_extensions\RV32I_PFIC_no_extensions" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\libraries\FreeRTOS\portable\MemMang" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\libraries\FreeRTOS" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\libraries\sdk\Core" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\libraries\sdk\Ld" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\libraries\sdk\Peripheral" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\libraries\sdk\Startup" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\project\user\inc" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\libraries\zf_common" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\libraries\zf_device" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\project\code" -I"C:\Users\Administrator1\Desktop\Demo_gyroscope\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

