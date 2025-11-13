################################################################################
# MRS Version: 1.9.1
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
D:/Downloads/开发资料/runsmartcar/Demo_gyroscope07_19_cpcp/libraries/LVGL/src/draw/sw/blend/helium/lv_blend_helium.S 

OBJS += \
./LVGL/src/draw/sw/blend/helium/lv_blend_helium.o 

S_UPPER_DEPS += \
./LVGL/src/draw/sw/blend/helium/lv_blend_helium.d 


# Each subdirectory must supply rules for building sources it contributes
LVGL/src/draw/sw/blend/helium/lv_blend_helium.o: D:/Downloads/开发资料/runsmartcar/Demo_gyroscope07_19_cpcp/libraries/LVGL/src/draw/sw/blend/helium/lv_blend_helium.S
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O1 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -x assembler-with-cpp -I"D:\Downloads\开发资料\runsmartcar\Demo_gyroscope07_19_cpcp\libraries\sdk\Startup" -I"D:\Downloads\开发资料\runsmartcar\Demo_gyroscope07_19_cpcp\libraries\FreeRTOS" -I"D:\Downloads\开发资料\runsmartcar\Demo_gyroscope07_19_cpcp\libraries\FreeRTOS\include" -I"D:\Downloads\开发资料\runsmartcar\Demo_gyroscope07_19_cpcp\libraries\FreeRTOS\portable\Common" -I"D:\Downloads\开发资料\runsmartcar\Demo_gyroscope07_19_cpcp\libraries\FreeRTOS\portable\GCC\RISC-V" -I"D:\Downloads\开发资料\runsmartcar\Demo_gyroscope07_19_cpcp\libraries\FreeRTOS\portable\GCC\RISC-V\chip_specific_extensions\RV32I_PFIC_no_extensions" -I"D:\Downloads\开发资料\runsmartcar\Demo_gyroscope07_19_cpcp\libraries\FreeRTOS\portable\MemMang" -I"D:\Downloads\开发资料\runsmartcar\Demo_gyroscope07_19_cpcp\libraries\zf_driver" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

