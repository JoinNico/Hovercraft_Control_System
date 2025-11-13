################################################################################
# MRS Version: 1.9.1
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Administrator1/Desktop/Hovercraft_Control_System/libraries/FreeRTOS/portable/GCC/RISC-V/port.c 

S_UPPER_SRCS += \
C:/Users/Administrator1/Desktop/Hovercraft_Control_System/libraries/FreeRTOS/portable/GCC/RISC-V/portASM.S 

OBJS += \
./FreeRTOS/portable/GCC/RISC-V/port.o \
./FreeRTOS/portable/GCC/RISC-V/portASM.o 

S_UPPER_DEPS += \
./FreeRTOS/portable/GCC/RISC-V/portASM.d 

C_DEPS += \
./FreeRTOS/portable/GCC/RISC-V/port.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/portable/GCC/RISC-V/port.o: C:/Users/Administrator1/Desktop/Hovercraft_Control_System/libraries/FreeRTOS/portable/GCC/RISC-V/port.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O1 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\Libraries\doc" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\include" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\portable\Common" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\portable\GCC\RISC-V" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\portable\GCC\RISC-V\chip_specific_extensions\RV32I_PFIC_no_extensions" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\portable\MemMang" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\sdk\Core" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\sdk\Ld" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\sdk\Peripheral" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\sdk\Startup" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\project\user\inc" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\zf_common" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\zf_device" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\project\code" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
FreeRTOS/portable/GCC/RISC-V/portASM.o: C:/Users/Administrator1/Desktop/Hovercraft_Control_System/libraries/FreeRTOS/portable/GCC/RISC-V/portASM.S
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O1 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -x assembler-with-cpp -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\sdk\Startup" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\include" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\portable\Common" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\portable\GCC\RISC-V" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\portable\GCC\RISC-V\chip_specific_extensions\RV32I_PFIC_no_extensions" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\FreeRTOS\portable\MemMang" -I"C:\Users\Administrator1\Desktop\Hovercraft_Control_System\libraries\zf_driver" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

