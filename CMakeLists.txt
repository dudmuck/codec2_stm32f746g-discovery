
# stm32f7 Codec2 test programs
#
# CMake configuration contributed by Richard Shaw (KF5OIM)
# Please report questions, comments, problems, or patches to the freetel
# mailing list: https://lists.sourceforge.net/lists/listinfo/freetel-codec2
#
set(ARM_GCC_BIN "" CACHE STRING "Path to the bin directory of your arm-eabi-none-gcc (optional)")
# Set default toolchain file for cross-compilation (must be before project())
if(NOT CMAKE_TOOLCHAIN_FILE)
	set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/STM32_Toolchain.cmake" CACHE FILEPATH "Toolchain file" FORCE)
endif()
project(stm32f7 C ASM)

if(CMAKE_CROSSCOMPILING)
	message(STATUS "We are cross compiling...")
else()
	message(STATUS "Performing standard host build...")
endif()

cmake_minimum_required(VERSION 2.8)

set(RADIO SX12XX CACHE STRING "radio device family to use")
set(MODEM "LORA" CACHE STRING "Modulation type: LORA or FSK (LR2021 only)")

# Generate GIT_HASH for freedv_api.c
if(EXISTS ${CMAKE_SOURCE_DIR}/.git)
	find_package(Git)
	if(Git_FOUND)
		execute_process(
			COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
			WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
			RESULT_VARIABLE res
			OUTPUT_VARIABLE PROJECT_HASH
			ERROR_QUIET
			OUTPUT_STRIP_TRAILING_WHITESPACE)
		message(STATUS "Project current git hash: ${PROJECT_HASH}")
		add_definitions(-DGIT_HASH="${PROJECT_HASH}")
	else()
		message(WARNING "Git not found. Can not determine current commit hash.")
		add_definitions(-DGIT_HASH="Unknown")
	endif()
else()
	add_definitions(-DGIT_HASH="None")
endif()

# Note: gencodebooks.cmake removed - this branch uses Opus instead of codec2

#
# Prevent in-source builds
# If an in-source build is attempted, you will still need to clean up a few
# files manually.
#
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "In-source builds in ${CMAKE_BINARY_DIR} are not "
   "allowed, please remove ./CMakeCache.txt and ./CMakeFiles/, create a "
   "separate build directory and run cmake from there.")
endif("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")

###################################################

set(FLOAT_TYPE "hard" CACHE STRING "Floating point: defaults to hard.")

###################################################

# Set default C flags.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -std=gnu11 -mlittle-endian -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mthumb-interwork -u_printf_float -fmessage-length=0 -ffunction-sections -fdata-sections -Os")
#

add_definitions(-DSTM32F746xx 
	-D__TARGET_CPU_CORTEX_M7_FP_SP
	-D__TARGET_FPU_FPV4_SP
	-DARMCM7_SP
	-DARM_MATH_CM7
	-DARM_MATH_MATRIX_CHECK
	-DARM_MATH_ROUNDING
)

if(FLOAT_TYPE STREQUAL "hard")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsingle-precision-constant -Wdouble-promotion -mfpu=fpv5-sp-d16 -mthumb -mfloat-abi=hard -D__FPU_PRESENT=1 -D__FPU_USED=1")
	#CFLAGS += -fsingle-precision-constant
else()
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msoft-float")
endif()

option(VERIFY_OPT "Enable this for dump files to help verify optimization" OFF)
if(VERIFY_OPT)
	add_definitions(-DDUMP)
endif()

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug")
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	link_libraries(g m)
else()
	link_libraries(c m)
endif()

# Setup defaults that can't be set in the toolchain file
set(CMAKE_EXE_LINKER_FLAGS "-u_init -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -specs=nano.specs -T${CMAKE_SOURCE_DIR}/stm32f746_flash.ld -Xlinker --gc-sections")
set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")


# Check build flags

message(STATUS "CMAKE_C_FLAGS: " ${CMAKE_C_FLAGS})
message(STATUS "CMAKE_ASM_FLAGS: " ${CMAKE_ASM_FLAGS})


###################################################

include(STM32746_Lib.cmake)

# Opus codec library (alternative to codec2)
include(Opus_Lib.cmake)

# FreeRTOS for preemptive multitasking
include(FreeRTOS_Lib.cmake)

###################################################

# Macro for elf->bin
macro(elf2bin target)
	add_custom_command(TARGET ${target}
	POST_BUILD COMMAND ${CMAKE_OBJCOPY} -O binary ${target}.elf ${target}.bin
	COMMENT "Creating binary for ${target}")
	set_source_files_properties(${target}.bin PROPERTIES GENERATED TRUE)
	set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY
	ADDITIONAL_MAKE_CLEAN_FILES ${target}.bin)
endmacro()

macro(elf2hex target)
	add_custom_command(TARGET ${target}
	POST_BUILD COMMAND ${CMAKE_OBJCOPY} -O ihex ${target}.elf ${target}.hex
	COMMENT "Creating ihex for ${target}")
	set_source_files_properties(${target}.hex PROPERTIES GENERATED TRUE)
	set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY
	ADDITIONAL_MAKE_CLEAN_FILES ${target}.hex)
endmacro()

# Macro for elf->bin
macro(elf2dfu target)
	add_custom_command(TARGET ${target}
			POST_BUILD COMMAND ${CMAKE_OBJCOPY} -O ihex ${target}.elf ${target}.hex && ${CMAKE_SOURCE_DIR}/support/hex2dfu.py ${target}.hex ${target}.dfu
	COMMENT "Creating dfu file for ${target}")
	set_source_files_properties(${target}.bin PROPERTIES GENERATED TRUE)
	set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY
	ADDITIONAL_MAKE_CLEAN_FILES ${target}.hex ${target}.dfu)
endmacro()

# This macro just adds generation of a map file with the same name as the executable and .map suffix
# to the linker command line. This works in older Cmake version (versions >= 3.13 have target_link_options)
# it should be a one to one replacement for add_executable
macro(add_mapped_executable target)
	add_executable(${target} ${ARGN})
	target_link_libraries(${target} "-Wl,-Map=$<TARGET_PROPERTY:NAME>.map")
	set_source_files_properties(${target}.map PROPERTIES GENERATED TRUE)
	set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY
	ADDITIONAL_MAKE_CLEAN_FILES ${target}.map)
endmacro()

include(CTest)
include_directories(${PROJECT_BINARY_DIR})

# Note: codec2 library removed - this branch uses Opus instead

set(SYSTEM_SRCS
	startup_stm32f746xx.s
	syscalls.c
)

# -----------------------------------------------------------------------------

set(SECOND_SRCS
	passthru/Src/stm32f7xx_it.c
	passthru/Src/audio_loopback.c
	passthru/Src/audio_rec.c
	passthru/Src/main.c
	passthru/Src/system_stm32f7xx.c
)

add_mapped_executable(passthru ${SECOND_SRCS} ${SYSTEM_SRCS})
target_link_libraries(passthru stm32746gdiscovery stm32f7)
#target_compile_definitions(passthru PRIVATE -DMIC_DISABLE=MIC_RIGHT)
target_include_directories(passthru PRIVATE ${PROJECT_SOURCE_DIR}/passthru/Inc)
elf2bin(passthru)
elf2hex(passthru)

set(VP_SRCS
	vocoder_passthru/Src/stm32f7xx_it.c
	vocoder_passthru/Src/audio_loopback.c
	vocoder_passthru/Src/audio_rec.c
	vocoder_passthru/Src/main.c
	vocoder_passthru/Src/system_stm32f7xx.c
)

add_mapped_executable(vocoder_passthru ${VP_SRCS} ${SYSTEM_SRCS})
target_link_libraries(vocoder_passthru stm32746gdiscovery stm32f7 opus_wrapper)
#target_compile_definitions(vocoder_passthru PRIVATE -DMIC_DISABLE=MIC_RIGHT)
target_include_directories(vocoder_passthru PRIVATE ${PROJECT_SOURCE_DIR}/vocoder_passthru/Inc)
elf2bin(vocoder_passthru)
elf2hex(vocoder_passthru)

set(LORA_XCVR_SRCS
	lora_transceiver/Src/stm32f7xx_it.c
	lora_transceiver/Src/lora_xcvr.c
	lora_transceiver/Src/main.c
	lora_transceiver/Src/radio.c
	lora_transceiver/Src/system_stm32f7xx.c
	lora_transceiver/Src/freertos_tasks.c
)

if(${RADIO} STREQUAL SX12XX)
	message(SEND_ERROR "radio is sx12")
	set(LORA_XCVR_SRCS ${LORA_XCVR_SRCS}
		lora_transceiver/Src/app_sx126x.c
		lora_transceiver/Src/app_sx127x.c
		sx12xx_hal/radio_sx126x.c
		sx12xx_hal/radio_sx127x.c
		sx126x/sx126x.c
		sx127x/sx127x.c
	)
elseif(${RADIO} STREQUAL LR2021)
	set(LORA_XCVR_SRCS ${LORA_XCVR_SRCS}
		lora_transceiver/Src/app_lr20xx.c
		lr20xx_hal/radio_lr20xx.c
		lr20xx/lr20xx.c
		lr20xx/lr20xx_hal.c
		lr20xx_driver/src/lr20xx_system.c
		lr20xx_driver/src/lr20xx_radio_fifo.c
		lr20xx_driver/src/lr20xx_radio_common.c
		lr20xx_driver/src/lr20xx_workarounds.c
		lr20xx_driver/src/lr20xx_regmem.c
	)
	# Add modem-specific driver source
	if(${MODEM} STREQUAL "FSK")
		set(LORA_XCVR_SRCS ${LORA_XCVR_SRCS}
			lr20xx_driver/src/lr20xx_radio_fsk.c
		)
		message(STATUS "LR2021 modem: FSK")
	else()
		set(LORA_XCVR_SRCS ${LORA_XCVR_SRCS}
			lr20xx_driver/src/lr20xx_radio_lora.c
		)
		message(STATUS "LR2021 modem: LoRa")
	endif()
else()
	message(SEND_ERROR "other RADIO ${RADIO}")
endif()

# Set target name based on modem type
if(${MODEM} STREQUAL "FSK")
	set(LORA_TARGET fsk_transceiver_${RADIO})
else()
	set(LORA_TARGET lora_transceiver_${RADIO})
endif()
add_mapped_executable(${LORA_TARGET} ${LORA_XCVR_SRCS} ${SYSTEM_SRCS})
target_link_libraries(${LORA_TARGET} stm32746gdiscovery stm32f7 opus_wrapper freertos)
#target_compile_definitions(${LORA_TARGET} PRIVATE -DMIC_DISABLE=MIC_RIGHT)
if(${RADIO} STREQUAL SX12XX)
	set(RADIO_INCLUDES
		${PROJECT_SOURCE_DIR}/sx12xx_hal
		${PROJECT_SOURCE_DIR}/sx126x
		${PROJECT_SOURCE_DIR}/sx127x
	)
elseif(${RADIO} STREQUAL LR2021)
	set(RADIO_INCLUDES
		${PROJECT_SOURCE_DIR}/lr20xx_hal
		${PROJECT_SOURCE_DIR}/lr20xx
		${PROJECT_SOURCE_DIR}/lr20xx_driver/inc
	)
	target_compile_definitions(${LORA_TARGET} PRIVATE -DENABLE_LR20XX)
	if(${MODEM} STREQUAL "FSK")
		target_compile_definitions(${LORA_TARGET} PRIVATE -DMODEM_FSK)
	else()
		target_compile_definitions(${LORA_TARGET} PRIVATE -DMODEM_LORA)
	endif()
endif()
target_include_directories(${LORA_TARGET} PRIVATE
	${PROJECT_SOURCE_DIR}/lora_transceiver/Inc
	${RADIO_INCLUDES}
)
elf2bin(${LORA_TARGET})
elf2hex(${LORA_TARGET})

# Make radio_lr20xx.c rebuild when key source files change, so __DATE__/__TIME__ stay current.
# This ensures the build timestamp reflects changes to transceiver code.
set_source_files_properties(${PROJECT_SOURCE_DIR}/lr20xx_hal/radio_lr20xx.c PROPERTIES
	OBJECT_DEPENDS "${PROJECT_SOURCE_DIR}/lora_transceiver/Src/lora_xcvr.c;${PROJECT_SOURCE_DIR}/lora_transceiver/Src/main.c;${PROJECT_SOURCE_DIR}/lora_transceiver/Src/freertos_tasks.c;${PROJECT_SOURCE_DIR}/lora_transceiver/Inc/FreeRTOSConfig.h;${PROJECT_SOURCE_DIR}/CMakeLists.txt"
)

include_directories(${PROJECT_SOURCE_DIR})

