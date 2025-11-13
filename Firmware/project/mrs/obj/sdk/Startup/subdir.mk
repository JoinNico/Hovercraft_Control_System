################################################################################
# MRS Version: 1.9.1
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
C:/Users/Administrator1/Desktop/Hovercraft_Control_System/libraries/sdk/Startup/startup_ch32v30x_D8C.S 

OBJS += \
./sdk/Startup/startup_ch32v30x_D8C.o 

S_UPPER_DEPS += \
./sdk/Startup/startup_ch32v30x_D8C.d 


# Each subdirectory must supply rules for building sources it contributes
sdk/Startup/startup_ch32v30x_D8C.o: C:/Users/Administrator1/Desktop/Hovercraft_Control_System/libraries/sdk/Startup/startup_ch32v30x_D8C.S
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O1 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -x assembler-with-cpp -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\sdk\Startup" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\include" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\portable\Common" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\portable\GCC\RISC-V" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\portable\GCC\RISC-V\chip_specific_extensions\RV32I_PFIC_no_extensions" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\portable\MemMang" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\zf_driver" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

