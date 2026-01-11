# FreeRTOS library configuration for STM32F746

set(FREERTOS_DIR ${PROJECT_SOURCE_DIR}/STM32CubeF7/Middlewares/Third_Party/FreeRTOS/Source)

set(FREERTOS_SRCS
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM7/r0p1/port.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)

add_library(freertos STATIC ${FREERTOS_SRCS})

target_include_directories(freertos PUBLIC
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM7/r0p1
    ${PROJECT_SOURCE_DIR}/lora_transceiver/Inc  # For FreeRTOSConfig.h
)

target_compile_definitions(freertos PUBLIC
    -DUSE_FREERTOS
)
