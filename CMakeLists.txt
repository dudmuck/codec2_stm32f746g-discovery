
# stm32f7 Codec2 test programs
#
# CMake configuration contributed by Richard Shaw (KF5OIM)
# Please report questions, comments, problems, or patches to the freetel
# mailing list: https://lists.sourceforge.net/lists/listinfo/freetel-codec2
#
set(ARM_GCC_BIN "" CACHE STRING "Path to the bin directory of your arm-eabi-none-gcc (optional)")
project(stm32f7 C ASM)

if(CMAKE_CROSSCOMPILING)
    message(STATUS "We are cross compiling...")
else()
    message(STATUS "Performing standard host build...")
endif()

cmake_minimum_required(VERSION 2.8)

include(gencodebooks.cmake)

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
set(CMAKE_TOOLCHAIN_FILE "../stm32/cmake/STM32_Toolchain.cmake" CACHE STRING "Toolchain defs")

###################################################

# Set default C flags.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -std=gnu11 -mlittle-endian -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mthumb-interwork -u_printf_float -fmessage-length=0 -ffunction-sections -fdata-sections -Os")

add_definitions(-DSTM32F746xx 
    -D__TARGET_CPU_CORTEX_M7_FP_SP
    -D__TARGET_FPU_FPV4_SP
    -DARMCM7_SP
    -DARM_MATH_CM7
    -DARM_MATH_MATRIX_CHECK
    -DARM_MATH_ROUNDING
)

if(FLOAT_TYPE STREQUAL "hard")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsingle-precision-constant -Wdouble-promotion -mfpu=fpv5-sp-d16 -mthumb -mfloat-abi=hard -D__FPU_PRESENT=1")
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
include_directories(codec2/src ../unittest ${PROJECT_BINARY_DIR})

# Codec 2

# Output path is such that #include <codec2/version.h> in codec2.h works
set(CODEC2_VERSION_PATH "${PROJECT_BINARY_DIR}/codec2")
configure_file ("${PROJECT_SOURCE_DIR}/codec2/cmake/version.h.in"
                "${CODEC2_VERSION_PATH}/version.h" )


set(CODEC2_SRC ../codec2/src)
set(CODEC2_GEN_CODEBOOK_SRC ../build/src)
set(CODEC2_SRCS
${CODEC2_SRC}/lpc.c
${CODEC2_SRC}/nlp.c
${CODEC2_SRC}/postfilter.c
${CODEC2_SRC}/sine.c
${CODEC2_SRC}/codec2.c
${CODEC2_SRC}/codec2_fft.c
${CODEC2_SRC}/gp_interleaver.c
${CODEC2_SRC}/interldpc.c
${CODEC2_SRC}/kiss_fft.c
${CODEC2_SRC}/kiss_fftr.c
${CODEC2_SRC}/interp.c
${CODEC2_SRC}/lsp.c
${CODEC2_SRC}/ofdm.c
${CODEC2_SRC}/phase.c
${CODEC2_SRC}/quantise.c
${CODEC2_SRC}/pack.c
${CODEC2_SRC}/dump.c
${CODEC2_SRC}/cohpsk.c
${CODEC2_SRC}/fdmdv.c
${CODEC2_SRC}/freedv_api.c
${CODEC2_SRC}/filter.c
${CODEC2_SRC}/varicode.c
${CODEC2_SRC}/golay23.c
${CODEC2_SRC}/freedv_data_channel.c
${CODEC2_SRC}/newamp1.c
${CODEC2_SRC}/newamp2.c
${CODEC2_SRC}/mbest.c
${CODEC2_SRC}/HRA_112_112.c
${CODEC2_SRC}/linreg.c
${CODEC2_SRC}/mpdecode_core.c
${CODEC2_SRC}/phi0.c
${CODEC2_SRC}/HRAb_396_504.c
codebook.c
codebookd.c
codebookjvm.c
codebookge.c
codebooknewamp1_energy.c
codebooknewamp1.c
codebooknewamp2.c
codebooknewamp2_energy.c
)

add_library(codec2 STATIC ${CODEC2_SRCS})
add_library(codec2_prof STATIC ${CODEC2_SRCS})
target_compile_definitions(codec2_prof PRIVATE PROFILE)

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
target_link_libraries(vocoder_passthru stm32746gdiscovery stm32f7 codec2)
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
    lora_transceiver/Src/app_sx126x.c
    lora_transceiver/Src/app_sx127x.c
    sx12xx_hal/radio_sx126x.c
    sx12xx_hal/radio_sx127x.c
    sx126x/sx126x.c
    sx127x/sx127x.c
)

add_mapped_executable(lora_transceiver ${LORA_XCVR_SRCS} ${SYSTEM_SRCS})
target_link_libraries(lora_transceiver stm32746gdiscovery stm32f7 codec2)
#target_compile_definitions(lora_transceiver PRIVATE -DMIC_DISABLE=MIC_RIGHT)
target_include_directories(lora_transceiver PRIVATE
    ${PROJECT_SOURCE_DIR}/lora_transceiver/Inc
    ${PROJECT_SOURCE_DIR}/sx12xx_hal
    ${PROJECT_SOURCE_DIR}/sx126x
    ${PROJECT_SOURCE_DIR}/sx127x
)
elf2bin(lora_transceiver)
elf2hex(lora_transceiver)

include_directories(${PROJECT_SOURCE_DIR})

