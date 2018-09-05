################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
LowlevelFunc.obj: ../LowlevelFunc.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-msp430_16.9.7.LTS/bin/cl430" -vmspx --data_model=restricted --use_hw_mpy=F5 --include_path="C:/ti/ccsv7/ccs_base/msp430/include" --include_path="C:/Users/CGV/workspace_v7/Flasher" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-msp430_16.9.7.LTS/include" --advice:power=all --advice:hw_config=all --define=__MSP430FR6989__ --define=_MPU_ENABLE -g --printf_support=full --diag_warning=225 --diag_wrap=off --display_error_number --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU40 --preproc_with_compile --preproc_dependency="LowlevelFunc.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

SBW430FR.obj: ../SBW430FR.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-msp430_16.9.7.LTS/bin/cl430" -vmspx --data_model=restricted --use_hw_mpy=F5 --include_path="C:/ti/ccsv7/ccs_base/msp430/include" --include_path="C:/Users/CGV/workspace_v7/Flasher" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-msp430_16.9.7.LTS/include" --advice:power=all --advice:hw_config=all --define=__MSP430FR6989__ --define=_MPU_ENABLE -g --printf_support=full --diag_warning=225 --diag_wrap=off --display_error_number --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU40 --preproc_with_compile --preproc_dependency="SBW430FR.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

main.obj: ../main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-msp430_16.9.7.LTS/bin/cl430" -vmspx --data_model=restricted --use_hw_mpy=F5 --include_path="C:/ti/ccsv7/ccs_base/msp430/include" --include_path="C:/Users/CGV/workspace_v7/Flasher" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-msp430_16.9.7.LTS/include" --advice:power=all --advice:hw_config=all --define=__MSP430FR6989__ --define=_MPU_ENABLE -g --printf_support=full --diag_warning=225 --diag_wrap=off --display_error_number --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU40 --preproc_with_compile --preproc_dependency="main.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


