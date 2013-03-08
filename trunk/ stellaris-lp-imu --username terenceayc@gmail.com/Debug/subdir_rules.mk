################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
adxl345.obj: ../adxl345.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/arm_5.0.3/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/ti/ccsv5/tools/compiler/arm_5.0.3/include" --include_path="C:/StellarisWare/boards/ek-lm4f120xl" --include_path="C:/StellarisWare/driverlib/" --include_path="C:/Users/Terence/workspace_v5_3/Stellaris IMU/inc" --include_path="C:/StellarisWare/inc" --include_path="C:/StellarisWare" --define=ccs="ccs" --define=PART_LM4F120H5QR --define=TARGET_IS_BLIZZARD_RA1 --diag_warning=225 --display_error_number --diag_wrap=off --printf_support=nofloat --preproc_with_compile --preproc_dependency="adxl345.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

bmp085.obj: ../bmp085.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/arm_5.0.3/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/ti/ccsv5/tools/compiler/arm_5.0.3/include" --include_path="C:/StellarisWare/boards/ek-lm4f120xl" --include_path="C:/StellarisWare/driverlib/" --include_path="C:/Users/Terence/workspace_v5_3/Stellaris IMU/inc" --include_path="C:/StellarisWare/inc" --include_path="C:/StellarisWare" --define=ccs="ccs" --define=PART_LM4F120H5QR --define=TARGET_IS_BLIZZARD_RA1 --diag_warning=225 --display_error_number --diag_wrap=off --printf_support=nofloat --preproc_with_compile --preproc_dependency="bmp085.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

hmc5883l.obj: ../hmc5883l.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/arm_5.0.3/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/ti/ccsv5/tools/compiler/arm_5.0.3/include" --include_path="C:/StellarisWare/boards/ek-lm4f120xl" --include_path="C:/StellarisWare/driverlib/" --include_path="C:/Users/Terence/workspace_v5_3/Stellaris IMU/inc" --include_path="C:/StellarisWare/inc" --include_path="C:/StellarisWare" --define=ccs="ccs" --define=PART_LM4F120H5QR --define=TARGET_IS_BLIZZARD_RA1 --diag_warning=225 --display_error_number --diag_wrap=off --printf_support=nofloat --preproc_with_compile --preproc_dependency="hmc5883l.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

i2c_IMU.obj: ../i2c_IMU.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/arm_5.0.3/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/ti/ccsv5/tools/compiler/arm_5.0.3/include" --include_path="C:/StellarisWare/boards/ek-lm4f120xl" --include_path="C:/StellarisWare/driverlib/" --include_path="C:/Users/Terence/workspace_v5_3/Stellaris IMU/inc" --include_path="C:/StellarisWare/inc" --include_path="C:/StellarisWare" --define=ccs="ccs" --define=PART_LM4F120H5QR --define=TARGET_IS_BLIZZARD_RA1 --diag_warning=225 --display_error_number --diag_wrap=off --printf_support=nofloat --preproc_with_compile --preproc_dependency="i2c_IMU.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

imu.obj: ../imu.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/arm_5.0.3/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/ti/ccsv5/tools/compiler/arm_5.0.3/include" --include_path="C:/StellarisWare/boards/ek-lm4f120xl" --include_path="C:/StellarisWare/driverlib/" --include_path="C:/Users/Terence/workspace_v5_3/Stellaris IMU/inc" --include_path="C:/StellarisWare/inc" --include_path="C:/StellarisWare" --define=ccs="ccs" --define=PART_LM4F120H5QR --define=TARGET_IS_BLIZZARD_RA1 --diag_warning=225 --display_error_number --diag_wrap=off --printf_support=nofloat --preproc_with_compile --preproc_dependency="imu.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

l3g4200d.obj: ../l3g4200d.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/arm_5.0.3/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/ti/ccsv5/tools/compiler/arm_5.0.3/include" --include_path="C:/StellarisWare/boards/ek-lm4f120xl" --include_path="C:/StellarisWare/driverlib/" --include_path="C:/Users/Terence/workspace_v5_3/Stellaris IMU/inc" --include_path="C:/StellarisWare/inc" --include_path="C:/StellarisWare" --define=ccs="ccs" --define=PART_LM4F120H5QR --define=TARGET_IS_BLIZZARD_RA1 --diag_warning=225 --display_error_number --diag_wrap=off --printf_support=nofloat --preproc_with_compile --preproc_dependency="l3g4200d.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

main.obj: ../main.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/arm_5.0.3/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/ti/ccsv5/tools/compiler/arm_5.0.3/include" --include_path="C:/StellarisWare/boards/ek-lm4f120xl" --include_path="C:/StellarisWare/driverlib/" --include_path="C:/Users/Terence/workspace_v5_3/Stellaris IMU/inc" --include_path="C:/StellarisWare/inc" --include_path="C:/StellarisWare" --define=ccs="ccs" --define=PART_LM4F120H5QR --define=TARGET_IS_BLIZZARD_RA1 --diag_warning=225 --display_error_number --diag_wrap=off --printf_support=nofloat --preproc_with_compile --preproc_dependency="main.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

startup_ccs.obj: ../startup_ccs.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/arm_5.0.3/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/ti/ccsv5/tools/compiler/arm_5.0.3/include" --include_path="C:/StellarisWare/boards/ek-lm4f120xl" --include_path="C:/StellarisWare/driverlib/" --include_path="C:/Users/Terence/workspace_v5_3/Stellaris IMU/inc" --include_path="C:/StellarisWare/inc" --include_path="C:/StellarisWare" --define=ccs="ccs" --define=PART_LM4F120H5QR --define=TARGET_IS_BLIZZARD_RA1 --diag_warning=225 --display_error_number --diag_wrap=off --printf_support=nofloat --preproc_with_compile --preproc_dependency="startup_ccs.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

stdio_console.obj: ../stdio_console.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/arm_5.0.3/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/ti/ccsv5/tools/compiler/arm_5.0.3/include" --include_path="C:/StellarisWare/boards/ek-lm4f120xl" --include_path="C:/StellarisWare/driverlib/" --include_path="C:/Users/Terence/workspace_v5_3/Stellaris IMU/inc" --include_path="C:/StellarisWare/inc" --include_path="C:/StellarisWare" --define=ccs="ccs" --define=PART_LM4F120H5QR --define=TARGET_IS_BLIZZARD_RA1 --diag_warning=225 --display_error_number --diag_wrap=off --printf_support=nofloat --preproc_with_compile --preproc_dependency="stdio_console.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

tmrsys.obj: ../tmrsys.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/arm_5.0.3/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/ti/ccsv5/tools/compiler/arm_5.0.3/include" --include_path="C:/StellarisWare/boards/ek-lm4f120xl" --include_path="C:/StellarisWare/driverlib/" --include_path="C:/Users/Terence/workspace_v5_3/Stellaris IMU/inc" --include_path="C:/StellarisWare/inc" --include_path="C:/StellarisWare" --define=ccs="ccs" --define=PART_LM4F120H5QR --define=TARGET_IS_BLIZZARD_RA1 --diag_warning=225 --display_error_number --diag_wrap=off --printf_support=nofloat --preproc_with_compile --preproc_dependency="tmrsys.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

util.obj: ../util.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/arm_5.0.3/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/ti/ccsv5/tools/compiler/arm_5.0.3/include" --include_path="C:/StellarisWare/boards/ek-lm4f120xl" --include_path="C:/StellarisWare/driverlib/" --include_path="C:/Users/Terence/workspace_v5_3/Stellaris IMU/inc" --include_path="C:/StellarisWare/inc" --include_path="C:/StellarisWare" --define=ccs="ccs" --define=PART_LM4F120H5QR --define=TARGET_IS_BLIZZARD_RA1 --diag_warning=225 --display_error_number --diag_wrap=off --printf_support=nofloat --preproc_with_compile --preproc_dependency="util.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


