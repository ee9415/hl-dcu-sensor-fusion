################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../0_Src/AppSw/Tricore/Adas_Manager.c" \
"../0_Src/AppSw/Tricore/CCU6_PWM_Generation.c" \
"../0_Src/AppSw/Tricore/Can_SpeedTx.c" \
"../0_Src/AppSw/Tricore/Can_TrajectoryFd.c" \
"../0_Src/AppSw/Tricore/ControlLoop.c" \
"../0_Src/AppSw/Tricore/Cpu0_Main.c" \
"../0_Src/AppSw/Tricore/Drive_PWM.c" \
"../0_Src/AppSw/Tricore/ERU_Interrupt.c" \
"../0_Src/AppSw/Tricore/GPT12_Timer_Interrupt.c" \
"../0_Src/AppSw/Tricore/GraphTest.c" \
"../0_Src/AppSw/Tricore/Ps2_Controller.c" \
"../0_Src/AppSw/Tricore/Speed_Calculate.c" \
"../0_Src/AppSw/Tricore/Steering_ADC.c" \
"../0_Src/AppSw/Tricore/VehicleManager.c" 

COMPILED_SRCS += \
"0_Src/AppSw/Tricore/Adas_Manager.src" \
"0_Src/AppSw/Tricore/CCU6_PWM_Generation.src" \
"0_Src/AppSw/Tricore/Can_SpeedTx.src" \
"0_Src/AppSw/Tricore/Can_TrajectoryFd.src" \
"0_Src/AppSw/Tricore/ControlLoop.src" \
"0_Src/AppSw/Tricore/Cpu0_Main.src" \
"0_Src/AppSw/Tricore/Drive_PWM.src" \
"0_Src/AppSw/Tricore/ERU_Interrupt.src" \
"0_Src/AppSw/Tricore/GPT12_Timer_Interrupt.src" \
"0_Src/AppSw/Tricore/GraphTest.src" \
"0_Src/AppSw/Tricore/Ps2_Controller.src" \
"0_Src/AppSw/Tricore/Speed_Calculate.src" \
"0_Src/AppSw/Tricore/Steering_ADC.src" \
"0_Src/AppSw/Tricore/VehicleManager.src" 

C_DEPS += \
"./0_Src/AppSw/Tricore/Adas_Manager.d" \
"./0_Src/AppSw/Tricore/CCU6_PWM_Generation.d" \
"./0_Src/AppSw/Tricore/Can_SpeedTx.d" \
"./0_Src/AppSw/Tricore/Can_TrajectoryFd.d" \
"./0_Src/AppSw/Tricore/ControlLoop.d" \
"./0_Src/AppSw/Tricore/Cpu0_Main.d" \
"./0_Src/AppSw/Tricore/Drive_PWM.d" \
"./0_Src/AppSw/Tricore/ERU_Interrupt.d" \
"./0_Src/AppSw/Tricore/GPT12_Timer_Interrupt.d" \
"./0_Src/AppSw/Tricore/GraphTest.d" \
"./0_Src/AppSw/Tricore/Ps2_Controller.d" \
"./0_Src/AppSw/Tricore/Speed_Calculate.d" \
"./0_Src/AppSw/Tricore/Steering_ADC.d" \
"./0_Src/AppSw/Tricore/VehicleManager.d" 

OBJS += \
"0_Src/AppSw/Tricore/Adas_Manager.o" \
"0_Src/AppSw/Tricore/CCU6_PWM_Generation.o" \
"0_Src/AppSw/Tricore/Can_SpeedTx.o" \
"0_Src/AppSw/Tricore/Can_TrajectoryFd.o" \
"0_Src/AppSw/Tricore/ControlLoop.o" \
"0_Src/AppSw/Tricore/Cpu0_Main.o" \
"0_Src/AppSw/Tricore/Drive_PWM.o" \
"0_Src/AppSw/Tricore/ERU_Interrupt.o" \
"0_Src/AppSw/Tricore/GPT12_Timer_Interrupt.o" \
"0_Src/AppSw/Tricore/GraphTest.o" \
"0_Src/AppSw/Tricore/Ps2_Controller.o" \
"0_Src/AppSw/Tricore/Speed_Calculate.o" \
"0_Src/AppSw/Tricore/Steering_ADC.o" \
"0_Src/AppSw/Tricore/VehicleManager.o" 


# Each subdirectory must supply rules for building sources it contributes
"0_Src/AppSw/Tricore/Adas_Manager.src":"../0_Src/AppSw/Tricore/Adas_Manager.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/Adas_Manager.o":"0_Src/AppSw/Tricore/Adas_Manager.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"0_Src/AppSw/Tricore/CCU6_PWM_Generation.src":"../0_Src/AppSw/Tricore/CCU6_PWM_Generation.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/CCU6_PWM_Generation.o":"0_Src/AppSw/Tricore/CCU6_PWM_Generation.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"0_Src/AppSw/Tricore/Can_SpeedTx.src":"../0_Src/AppSw/Tricore/Can_SpeedTx.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/Can_SpeedTx.o":"0_Src/AppSw/Tricore/Can_SpeedTx.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"0_Src/AppSw/Tricore/Can_TrajectoryFd.src":"../0_Src/AppSw/Tricore/Can_TrajectoryFd.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/Can_TrajectoryFd.o":"0_Src/AppSw/Tricore/Can_TrajectoryFd.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"0_Src/AppSw/Tricore/ControlLoop.src":"../0_Src/AppSw/Tricore/ControlLoop.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/ControlLoop.o":"0_Src/AppSw/Tricore/ControlLoop.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"0_Src/AppSw/Tricore/Cpu0_Main.src":"../0_Src/AppSw/Tricore/Cpu0_Main.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/Cpu0_Main.o":"0_Src/AppSw/Tricore/Cpu0_Main.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"0_Src/AppSw/Tricore/Drive_PWM.src":"../0_Src/AppSw/Tricore/Drive_PWM.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/Drive_PWM.o":"0_Src/AppSw/Tricore/Drive_PWM.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"0_Src/AppSw/Tricore/ERU_Interrupt.src":"../0_Src/AppSw/Tricore/ERU_Interrupt.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/ERU_Interrupt.o":"0_Src/AppSw/Tricore/ERU_Interrupt.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"0_Src/AppSw/Tricore/GPT12_Timer_Interrupt.src":"../0_Src/AppSw/Tricore/GPT12_Timer_Interrupt.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/GPT12_Timer_Interrupt.o":"0_Src/AppSw/Tricore/GPT12_Timer_Interrupt.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"0_Src/AppSw/Tricore/GraphTest.src":"../0_Src/AppSw/Tricore/GraphTest.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/GraphTest.o":"0_Src/AppSw/Tricore/GraphTest.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"0_Src/AppSw/Tricore/Ps2_Controller.src":"../0_Src/AppSw/Tricore/Ps2_Controller.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/Ps2_Controller.o":"0_Src/AppSw/Tricore/Ps2_Controller.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"0_Src/AppSw/Tricore/Speed_Calculate.src":"../0_Src/AppSw/Tricore/Speed_Calculate.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/Speed_Calculate.o":"0_Src/AppSw/Tricore/Speed_Calculate.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"0_Src/AppSw/Tricore/Steering_ADC.src":"../0_Src/AppSw/Tricore/Steering_ADC.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/Steering_ADC.o":"0_Src/AppSw/Tricore/Steering_ADC.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"0_Src/AppSw/Tricore/VehicleManager.src":"../0_Src/AppSw/Tricore/VehicleManager.c" "0_Src/AppSw/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc27xd "-fC:/Users/junyoung/Desktop/2026_FMA/ADAS_AMG_src_build_ready/ADAS_AMG_src/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"0_Src/AppSw/Tricore/VehicleManager.o":"0_Src/AppSw/Tricore/VehicleManager.src" "0_Src/AppSw/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-0_Src-2f-AppSw-2f-Tricore

clean-0_Src-2f-AppSw-2f-Tricore:
	-$(RM) ./0_Src/AppSw/Tricore/Adas_Manager.d ./0_Src/AppSw/Tricore/Adas_Manager.o ./0_Src/AppSw/Tricore/Adas_Manager.src ./0_Src/AppSw/Tricore/CCU6_PWM_Generation.d ./0_Src/AppSw/Tricore/CCU6_PWM_Generation.o ./0_Src/AppSw/Tricore/CCU6_PWM_Generation.src ./0_Src/AppSw/Tricore/Can_SpeedTx.d ./0_Src/AppSw/Tricore/Can_SpeedTx.o ./0_Src/AppSw/Tricore/Can_SpeedTx.src ./0_Src/AppSw/Tricore/Can_TrajectoryFd.d ./0_Src/AppSw/Tricore/Can_TrajectoryFd.o ./0_Src/AppSw/Tricore/Can_TrajectoryFd.src ./0_Src/AppSw/Tricore/ControlLoop.d ./0_Src/AppSw/Tricore/ControlLoop.o ./0_Src/AppSw/Tricore/ControlLoop.src ./0_Src/AppSw/Tricore/Cpu0_Main.d ./0_Src/AppSw/Tricore/Cpu0_Main.o ./0_Src/AppSw/Tricore/Cpu0_Main.src ./0_Src/AppSw/Tricore/Drive_PWM.d ./0_Src/AppSw/Tricore/Drive_PWM.o ./0_Src/AppSw/Tricore/Drive_PWM.src ./0_Src/AppSw/Tricore/ERU_Interrupt.d ./0_Src/AppSw/Tricore/ERU_Interrupt.o ./0_Src/AppSw/Tricore/ERU_Interrupt.src ./0_Src/AppSw/Tricore/GPT12_Timer_Interrupt.d ./0_Src/AppSw/Tricore/GPT12_Timer_Interrupt.o ./0_Src/AppSw/Tricore/GPT12_Timer_Interrupt.src ./0_Src/AppSw/Tricore/GraphTest.d ./0_Src/AppSw/Tricore/GraphTest.o ./0_Src/AppSw/Tricore/GraphTest.src ./0_Src/AppSw/Tricore/Ps2_Controller.d ./0_Src/AppSw/Tricore/Ps2_Controller.o ./0_Src/AppSw/Tricore/Ps2_Controller.src ./0_Src/AppSw/Tricore/Speed_Calculate.d ./0_Src/AppSw/Tricore/Speed_Calculate.o ./0_Src/AppSw/Tricore/Speed_Calculate.src ./0_Src/AppSw/Tricore/Steering_ADC.d ./0_Src/AppSw/Tricore/Steering_ADC.o ./0_Src/AppSw/Tricore/Steering_ADC.src ./0_Src/AppSw/Tricore/VehicleManager.d ./0_Src/AppSw/Tricore/VehicleManager.o ./0_Src/AppSw/Tricore/VehicleManager.src

.PHONY: clean-0_Src-2f-AppSw-2f-Tricore

