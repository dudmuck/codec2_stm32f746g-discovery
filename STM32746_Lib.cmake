###################################################

# Definitions for the STM32F7 Standard Peripheral Library

#set(PERIPHLIBURL  http://www.st.com/st-web-ui/static/active/en/st_prod_software_internet/resource/technical/software/firmware)
#set(PERIPHLIBZIP  stm32f4_dsp_stdperiph_lib.zip)
set(PERIPHLIBVER    1.15.0)
set(PERIPHLIBNAME   STM32Cube_FW_F7_V)

if(NOT PERIPHLIBDIR)
    set(PERIPHLIBDIR    ${CMAKE_SOURCE_DIR}/${PERIPHLIBNAME}${PERIPHLIBVER})
    message(STATUS "Using default path for StdPeriph Lib: ${PERIPHLIBDIR}")
endif()

set(CMSIS           ${PERIPHLIBDIR}/Drivers/CMSIS)
set(STM32F7LIB      ${PERIPHLIBDIR}/Drivers/STM32F7xx_HAL_Driver)
set(DSPLIB          ${PERIPHLIBDIR}/Drivers/CMSIS/DSP/Source)
set(UTILITIES       ${PERIPHLIBDIR}/Utilities)
set(MIDDLEWARES     ${PERIPHLIBDIR}/Middlewares)

#add_definitions(-DUSE_STDPERIPH_DRIVER -DARM_MATH_CM4 -DHSE_VALUE=\(\(uint32_t\)8000000\))
#include_directories(${STM32F4LIB}/inc ${STM32F4TEMPLATE}
#    ${CMSIS}/Include ${CMSIS}/Device/ST/STM32F4xx/Include)
include_directories(${STM32F7LIB}/Inc
    ${CMSIS}/Device/ST/STM32F7xx/Include
    ${CMSIS}/Include
    ${CMSIS}/DSP/Include
    ${PERIPHLIBDIR}/Drivers/BSP/STM32746G-Discovery
    ${UTILITIES}/Log
    
)


set(STM32F7LIB_SRCS
    ${STM32F7LIB}/Src/stm32f7xx_hal.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_uart.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_rcc.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_rcc_ex.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_cortex.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_gpio.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_pwr_ex.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_ltdc.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_sdram.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_dma.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_dma2d.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_sai.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_i2c.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_qspi.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_sd.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_rtc.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_rtc_ex.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_hcd.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_eth.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_rng.c
    ${STM32F7LIB}/Src/stm32f7xx_hal_spi.c
    ${STM32F7LIB}/Src/stm32f7xx_ll_fmc.c
    ${STM32F7LIB}/Src/stm32f7xx_ll_sdmmc.c
    ${STM32F7LIB}/Src/stm32f7xx_ll_usb.c
    #${STM32F4LIB}/src/misc.c
#${STM32F4LIB}/src/stm32f4xx_adc.c
#${STM32F4LIB}/src/stm32f4xx_can.c
#${STM32F4LIB}/src/stm32f4xx_cec.c
#${STM32F4LIB}/src/stm32f4xx_crc.c
#${STM32F4LIB}/src/stm32f4xx_cryp_aes.c
#${STM32F4LIB}/src/stm32f4xx_cryp.c
#${STM32F4LIB}/src/stm32f4xx_cryp_des.c
#${STM32F4LIB}/src/stm32f4xx_cryp_tdes.c
#${STM32F4LIB}/src/stm32f4xx_dac.c
#${STM32F4LIB}/src/stm32f4xx_dbgmcu.c
#${STM32F4LIB}/src/stm32f4xx_dcmi.c
#${STM32F4LIB}/src/stm32f4xx_dma2d.c
#${STM32F4LIB}/src/stm32f4xx_dma.c
#${STM32F4LIB}/src/stm32f4xx_exti.c
#${STM32F4LIB}/src/stm32f4xx_flash.c
#${STM32F4LIB}/src/stm32f4xx_flash_ramfunc.c
#${STM32F4LIB}/src/stm32f4xx_fmpi2c.c
#${STM32F4LIB}/src/stm32f4xx_fsmc.c
#${STM32F4LIB}/src/stm32f4xx_gpio.c
#${STM32F4LIB}/src/stm32f4xx_hash.c
#${STM32F4LIB}/src/stm32f4xx_hash_md5.c
#${STM32F4LIB}/src/stm32f4xx_hash_sha1.c
#${STM32F4LIB}/src/stm32f4xx_i2c.c
#${STM32F4LIB}/src/stm32f4xx_iwdg.c
#${STM32F4LIB}/src/stm32f4xx_ltdc.c
#${STM32F4LIB}/src/stm32f4xx_pwr.c
#${STM32F4LIB}/src/stm32f4xx_qspi.c
#${STM32F4LIB}/src/stm32f4xx_rcc.c
#${STM32F4LIB}/src/stm32f4xx_rng.c
#${STM32F4LIB}/src/stm32f4xx_rtc.c
#${STM32F4LIB}/src/stm32f4xx_sai.c
#${STM32F4LIB}/src/stm32f4xx_sdio.c
#${STM32F4LIB}/src/stm32f4xx_spdifrx.c
#${STM32F4LIB}/src/stm32f4xx_spi.c
#${STM32F4LIB}/src/stm32f4xx_syscfg.c
#${STM32F4LIB}/src/stm32f4xx_tim.c
#${STM32F4LIB}/src/stm32f4xx_usart.c
#${STM32F4LIB}/src/stm32f4xx_wwdg.c
# Not compiling for now
# $(STM32F4LIB)/src/stm32f4xx_fmc.c
)

add_library(stm32f7 STATIC ${STM32F7LIB_SRCS})

set(STM32F7DISCLIB_SRCS
    ${PERIPHLIBDIR}/Drivers/BSP/STM32746G-Discovery/stm32746g_discovery.c
    ${PERIPHLIBDIR}/Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_lcd.c
    ${PERIPHLIBDIR}/Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_sdram.c
    ${PERIPHLIBDIR}/Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_audio.c
    ${PERIPHLIBDIR}/Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_qspi.c
    ${PERIPHLIBDIR}/Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_eeprom.c
    ${PERIPHLIBDIR}/Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_sd.c
    ${PERIPHLIBDIR}/Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_ts.c
    ${PERIPHLIBDIR}/Drivers/BSP/Components/wm8994/wm8994.c
    ${PERIPHLIBDIR}/Drivers/BSP/Components/ft5336/ft5336.c
)

#set(USBH_SRCS
#    ${MIDDLEWARES}/ST/STM32_USB_Host_Library/Class/MSC/Src/usbh_msc.c
#    ${MIDDLEWARES}/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.c
#)
#add_library(usbh STATIC ${USBH_SRCS})
#target_include_directories(usbh PRIVATE
#    ${MIDDLEWARES}/ST/STM32_USB_Host_Library/Class/MSC/Inc
#    ${MIDDLEWARES}/ST/STM32_USB_Host_Library/Core/Inc
#)

#set(FREERTOS_SRCS
#    ${PERIPHLIBDIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
#)
#add_library(freertos STATIC ${FREERTOS_SRCS})
#target_include_directories(freertos PRIVATE ${PERIPHLIBDIR}/Middlewares/Third_Party/FreeRTOS/Source/include)

add_library(stm32746gdiscovery STATIC ${STM32F7DISCLIB_SRCS})

#set(LWIP_SRCS
#)
#add_library(lwip STATIC ${LWIP_SRCS})
#target_include_directories(lwip PRIVATE 
#    ${MIDDLEWARES}/Third_Party/LwIP/src/include
#)

set(CMSIS_SRCS
    ${DSPLIB}/BasicMathFunctions/arm_abs_f32.c
    ${DSPLIB}/BasicMathFunctions/arm_abs_q15.c
    ${DSPLIB}/BasicMathFunctions/arm_abs_q31.c
    ${DSPLIB}/BasicMathFunctions/arm_abs_q7.c
    ${DSPLIB}/BasicMathFunctions/arm_add_f32.c
    ${DSPLIB}/BasicMathFunctions/arm_add_q15.c
    ${DSPLIB}/BasicMathFunctions/arm_add_q31.c
    ${DSPLIB}/BasicMathFunctions/arm_add_q7.c
    ${DSPLIB}/BasicMathFunctions/arm_dot_prod_f32.c
    ${DSPLIB}/BasicMathFunctions/arm_dot_prod_q15.c
    ${DSPLIB}/BasicMathFunctions/arm_dot_prod_q31.c
    ${DSPLIB}/BasicMathFunctions/arm_dot_prod_q7.c
    ${DSPLIB}/BasicMathFunctions/arm_mult_f32.c
    ${DSPLIB}/BasicMathFunctions/arm_mult_q15.c
    ${DSPLIB}/BasicMathFunctions/arm_mult_q31.c
    ${DSPLIB}/BasicMathFunctions/arm_mult_q7.c
    ${DSPLIB}/BasicMathFunctions/arm_negate_f32.c
    ${DSPLIB}/BasicMathFunctions/arm_negate_q15.c
    ${DSPLIB}/BasicMathFunctions/arm_negate_q31.c
    ${DSPLIB}/BasicMathFunctions/arm_negate_q7.c
    ${DSPLIB}/BasicMathFunctions/arm_offset_f32.c
    ${DSPLIB}/BasicMathFunctions/arm_offset_q15.c
    ${DSPLIB}/BasicMathFunctions/arm_offset_q31.c
    ${DSPLIB}/BasicMathFunctions/arm_offset_q7.c
    ${DSPLIB}/BasicMathFunctions/arm_scale_f32.c
    ${DSPLIB}/BasicMathFunctions/arm_scale_q15.c
    ${DSPLIB}/BasicMathFunctions/arm_scale_q31.c
    ${DSPLIB}/BasicMathFunctions/arm_scale_q7.c
    ${DSPLIB}/BasicMathFunctions/arm_shift_q15.c
    ${DSPLIB}/BasicMathFunctions/arm_shift_q31.c
    ${DSPLIB}/BasicMathFunctions/arm_shift_q7.c
    ${DSPLIB}/BasicMathFunctions/arm_sub_f32.c
    ${DSPLIB}/BasicMathFunctions/arm_sub_q15.c
    ${DSPLIB}/BasicMathFunctions/arm_sub_q31.c
    ${DSPLIB}/BasicMathFunctions/arm_sub_q7.c
    ${DSPLIB}/CommonTables/arm_common_tables.c
    ${DSPLIB}/CommonTables/arm_const_structs.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_conj_f32.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_conj_q15.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_conj_q31.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_dot_prod_f32.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_dot_prod_q15.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_dot_prod_q31.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_mag_f32.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_mag_q15.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_mag_q31.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_mag_squared_f32.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_mag_squared_q15.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_mag_squared_q31.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_mult_real_f32.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_mult_real_q15.c
    ${DSPLIB}/ComplexMathFunctions/arm_cmplx_mult_real_q31.c
    ${DSPLIB}/ControllerFunctions/arm_pid_init_f32.c
    ${DSPLIB}/ControllerFunctions/arm_pid_init_q15.c
    ${DSPLIB}/ControllerFunctions/arm_pid_init_q31.c
    ${DSPLIB}/ControllerFunctions/arm_pid_reset_f32.c
    ${DSPLIB}/ControllerFunctions/arm_pid_reset_q15.c
    ${DSPLIB}/ControllerFunctions/arm_pid_reset_q31.c
    ${DSPLIB}/ControllerFunctions/arm_sin_cos_f32.c
    ${DSPLIB}/ControllerFunctions/arm_sin_cos_q31.c
    ${DSPLIB}/FastMathFunctions/arm_cos_f32.c
    ${DSPLIB}/FastMathFunctions/arm_cos_q15.c
    ${DSPLIB}/FastMathFunctions/arm_cos_q31.c
    ${DSPLIB}/FastMathFunctions/arm_sin_f32.c
    ${DSPLIB}/FastMathFunctions/arm_sin_q15.c
    ${DSPLIB}/FastMathFunctions/arm_sin_q31.c
    ${DSPLIB}/FastMathFunctions/arm_sqrt_q15.c
    ${DSPLIB}/FastMathFunctions/arm_sqrt_q31.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df1_32x64_init_q31.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df1_32x64_q31.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df1_f32.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df1_fast_q15.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df1_fast_q31.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df1_init_q15.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df1_init_q31.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df1_q15.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df1_q31.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df2T_f32.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df2T_f64.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df2T_init_f32.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_df2T_init_f64.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_stereo_df2T_f32.c
    ${DSPLIB}/FilteringFunctions/arm_biquad_cascade_stereo_df2T_init_f32.c
    ${DSPLIB}/FilteringFunctions/arm_conv_f32.c
    ${DSPLIB}/FilteringFunctions/arm_conv_fast_opt_q15.c
    ${DSPLIB}/FilteringFunctions/arm_conv_fast_q15.c
    ${DSPLIB}/FilteringFunctions/arm_conv_fast_q31.c
    ${DSPLIB}/FilteringFunctions/arm_conv_opt_q15.c
    ${DSPLIB}/FilteringFunctions/arm_conv_opt_q7.c
    ${DSPLIB}/FilteringFunctions/arm_conv_partial_f32.c
    ${DSPLIB}/FilteringFunctions/arm_conv_partial_fast_opt_q15.c
    ${DSPLIB}/FilteringFunctions/arm_conv_partial_fast_q15.c
    ${DSPLIB}/FilteringFunctions/arm_conv_partial_fast_q31.c
    ${DSPLIB}/FilteringFunctions/arm_conv_partial_opt_q15.c
    ${DSPLIB}/FilteringFunctions/arm_conv_partial_opt_q7.c
    ${DSPLIB}/FilteringFunctions/arm_conv_partial_q15.c
    ${DSPLIB}/FilteringFunctions/arm_conv_partial_q31.c
    ${DSPLIB}/FilteringFunctions/arm_conv_partial_q7.c
    ${DSPLIB}/FilteringFunctions/arm_conv_q15.c
    ${DSPLIB}/FilteringFunctions/arm_conv_q31.c
    ${DSPLIB}/FilteringFunctions/arm_conv_q7.c
    ${DSPLIB}/FilteringFunctions/arm_correlate_f32.c
    ${DSPLIB}/FilteringFunctions/arm_correlate_fast_opt_q15.c
    ${DSPLIB}/FilteringFunctions/arm_correlate_fast_q15.c
    ${DSPLIB}/FilteringFunctions/arm_correlate_fast_q31.c
    ${DSPLIB}/FilteringFunctions/arm_correlate_opt_q15.c
    ${DSPLIB}/FilteringFunctions/arm_correlate_opt_q7.c
    ${DSPLIB}/FilteringFunctions/arm_correlate_q15.c
    ${DSPLIB}/FilteringFunctions/arm_correlate_q31.c
    ${DSPLIB}/FilteringFunctions/arm_correlate_q7.c
    ${DSPLIB}/FilteringFunctions/arm_fir_decimate_f32.c
    ${DSPLIB}/FilteringFunctions/arm_fir_decimate_fast_q15.c
    ${DSPLIB}/FilteringFunctions/arm_fir_decimate_fast_q31.c
    ${DSPLIB}/FilteringFunctions/arm_fir_decimate_init_f32.c
    ${DSPLIB}/FilteringFunctions/arm_fir_decimate_init_q15.c
    ${DSPLIB}/FilteringFunctions/arm_fir_decimate_init_q31.c
    ${DSPLIB}/FilteringFunctions/arm_fir_decimate_q15.c
    ${DSPLIB}/FilteringFunctions/arm_fir_decimate_q31.c
    ${DSPLIB}/FilteringFunctions/arm_fir_f32.c
    ${DSPLIB}/FilteringFunctions/arm_fir_fast_q15.c
    ${DSPLIB}/FilteringFunctions/arm_fir_fast_q31.c
    ${DSPLIB}/FilteringFunctions/arm_fir_init_f32.c
    ${DSPLIB}/FilteringFunctions/arm_fir_init_q15.c
    ${DSPLIB}/FilteringFunctions/arm_fir_init_q31.c
    ${DSPLIB}/FilteringFunctions/arm_fir_init_q7.c
    ${DSPLIB}/FilteringFunctions/arm_fir_interpolate_f32.c
    ${DSPLIB}/FilteringFunctions/arm_fir_interpolate_init_f32.c
    ${DSPLIB}/FilteringFunctions/arm_fir_interpolate_init_q15.c
    ${DSPLIB}/FilteringFunctions/arm_fir_interpolate_init_q31.c
    ${DSPLIB}/FilteringFunctions/arm_fir_interpolate_q15.c
    ${DSPLIB}/FilteringFunctions/arm_fir_interpolate_q31.c
    ${DSPLIB}/FilteringFunctions/arm_fir_lattice_f32.c
    ${DSPLIB}/FilteringFunctions/arm_fir_lattice_init_f32.c
    ${DSPLIB}/FilteringFunctions/arm_fir_lattice_init_q15.c
    ${DSPLIB}/FilteringFunctions/arm_fir_lattice_init_q31.c
    ${DSPLIB}/FilteringFunctions/arm_fir_lattice_q15.c
    ${DSPLIB}/FilteringFunctions/arm_fir_lattice_q31.c
    ${DSPLIB}/FilteringFunctions/arm_fir_q15.c
    ${DSPLIB}/FilteringFunctions/arm_fir_q31.c
    ${DSPLIB}/FilteringFunctions/arm_fir_q7.c
    ${DSPLIB}/FilteringFunctions/arm_fir_sparse_f32.c
    ${DSPLIB}/FilteringFunctions/arm_fir_sparse_init_f32.c
    ${DSPLIB}/FilteringFunctions/arm_fir_sparse_init_q15.c
    ${DSPLIB}/FilteringFunctions/arm_fir_sparse_init_q31.c
    ${DSPLIB}/FilteringFunctions/arm_fir_sparse_init_q7.c
    ${DSPLIB}/FilteringFunctions/arm_fir_sparse_q15.c
    ${DSPLIB}/FilteringFunctions/arm_fir_sparse_q31.c
    ${DSPLIB}/FilteringFunctions/arm_fir_sparse_q7.c
    ${DSPLIB}/FilteringFunctions/arm_iir_lattice_f32.c
    ${DSPLIB}/FilteringFunctions/arm_iir_lattice_init_f32.c
    ${DSPLIB}/FilteringFunctions/arm_iir_lattice_init_q15.c
    ${DSPLIB}/FilteringFunctions/arm_iir_lattice_init_q31.c
    ${DSPLIB}/FilteringFunctions/arm_iir_lattice_q15.c
    ${DSPLIB}/FilteringFunctions/arm_iir_lattice_q31.c
    ${DSPLIB}/FilteringFunctions/arm_lms_f32.c
    ${DSPLIB}/FilteringFunctions/arm_lms_init_f32.c
    ${DSPLIB}/FilteringFunctions/arm_lms_init_q15.c
    ${DSPLIB}/FilteringFunctions/arm_lms_init_q31.c
    ${DSPLIB}/FilteringFunctions/arm_lms_norm_f32.c
    ${DSPLIB}/FilteringFunctions/arm_lms_norm_init_f32.c
    ${DSPLIB}/FilteringFunctions/arm_lms_norm_init_q15.c
    ${DSPLIB}/FilteringFunctions/arm_lms_norm_init_q31.c
    ${DSPLIB}/FilteringFunctions/arm_lms_norm_q15.c
    ${DSPLIB}/FilteringFunctions/arm_lms_norm_q31.c
    ${DSPLIB}/FilteringFunctions/arm_lms_q15.c
    ${DSPLIB}/FilteringFunctions/arm_lms_q31.c
    ${DSPLIB}/MatrixFunctions/arm_mat_add_f32.c
    ${DSPLIB}/MatrixFunctions/arm_mat_add_q15.c
    ${DSPLIB}/MatrixFunctions/arm_mat_add_q31.c
    ${DSPLIB}/MatrixFunctions/arm_mat_cmplx_mult_f32.c
    ${DSPLIB}/MatrixFunctions/arm_mat_cmplx_mult_q15.c
    ${DSPLIB}/MatrixFunctions/arm_mat_cmplx_mult_q31.c
    ${DSPLIB}/MatrixFunctions/arm_mat_init_f32.c
    ${DSPLIB}/MatrixFunctions/arm_mat_init_q15.c
    ${DSPLIB}/MatrixFunctions/arm_mat_init_q31.c
    ${DSPLIB}/MatrixFunctions/arm_mat_inverse_f32.c
    ${DSPLIB}/MatrixFunctions/arm_mat_inverse_f64.c
    ${DSPLIB}/MatrixFunctions/arm_mat_mult_f32.c
    ${DSPLIB}/MatrixFunctions/arm_mat_mult_fast_q15.c
    ${DSPLIB}/MatrixFunctions/arm_mat_mult_fast_q31.c
    ${DSPLIB}/MatrixFunctions/arm_mat_mult_q15.c
    ${DSPLIB}/MatrixFunctions/arm_mat_mult_q31.c
    ${DSPLIB}/MatrixFunctions/arm_mat_scale_f32.c
    ${DSPLIB}/MatrixFunctions/arm_mat_scale_q15.c
    ${DSPLIB}/MatrixFunctions/arm_mat_scale_q31.c
    ${DSPLIB}/MatrixFunctions/arm_mat_sub_f32.c
    ${DSPLIB}/MatrixFunctions/arm_mat_sub_q15.c
    ${DSPLIB}/MatrixFunctions/arm_mat_sub_q31.c
    ${DSPLIB}/MatrixFunctions/arm_mat_trans_f32.c
    ${DSPLIB}/MatrixFunctions/arm_mat_trans_q15.c
    ${DSPLIB}/MatrixFunctions/arm_mat_trans_q31.c
    ${DSPLIB}/StatisticsFunctions/arm_max_f32.c
    ${DSPLIB}/StatisticsFunctions/arm_max_q15.c
    ${DSPLIB}/StatisticsFunctions/arm_max_q31.c
    ${DSPLIB}/StatisticsFunctions/arm_max_q7.c
    ${DSPLIB}/StatisticsFunctions/arm_mean_f32.c
    ${DSPLIB}/StatisticsFunctions/arm_mean_q15.c
    ${DSPLIB}/StatisticsFunctions/arm_mean_q31.c
    ${DSPLIB}/StatisticsFunctions/arm_mean_q7.c
    ${DSPLIB}/StatisticsFunctions/arm_min_f32.c
    ${DSPLIB}/StatisticsFunctions/arm_min_q15.c
    ${DSPLIB}/StatisticsFunctions/arm_min_q31.c
    ${DSPLIB}/StatisticsFunctions/arm_min_q7.c
    ${DSPLIB}/StatisticsFunctions/arm_power_f32.c
${DSPLIB}/StatisticsFunctions/arm_power_q15.c
${DSPLIB}/StatisticsFunctions/arm_power_q31.c
${DSPLIB}/StatisticsFunctions/arm_power_q7.c
${DSPLIB}/StatisticsFunctions/arm_rms_f32.c
${DSPLIB}/StatisticsFunctions/arm_rms_q15.c
${DSPLIB}/StatisticsFunctions/arm_rms_q31.c
${DSPLIB}/StatisticsFunctions/arm_std_f32.c
${DSPLIB}/StatisticsFunctions/arm_std_q15.c
${DSPLIB}/StatisticsFunctions/arm_std_q31.c
${DSPLIB}/StatisticsFunctions/arm_var_f32.c
${DSPLIB}/StatisticsFunctions/arm_var_q15.c
${DSPLIB}/StatisticsFunctions/arm_var_q31.c
${DSPLIB}/SupportFunctions/arm_copy_f32.c
${DSPLIB}/SupportFunctions/arm_copy_q15.c
${DSPLIB}/SupportFunctions/arm_copy_q31.c
${DSPLIB}/SupportFunctions/arm_copy_q7.c
${DSPLIB}/SupportFunctions/arm_fill_f32.c
${DSPLIB}/SupportFunctions/arm_fill_q15.c
${DSPLIB}/SupportFunctions/arm_fill_q31.c
${DSPLIB}/SupportFunctions/arm_fill_q7.c
${DSPLIB}/SupportFunctions/arm_float_to_q15.c
${DSPLIB}/SupportFunctions/arm_float_to_q31.c
${DSPLIB}/SupportFunctions/arm_float_to_q7.c
${DSPLIB}/SupportFunctions/arm_q15_to_float.c
${DSPLIB}/SupportFunctions/arm_q15_to_q31.c
${DSPLIB}/SupportFunctions/arm_q15_to_q7.c
${DSPLIB}/SupportFunctions/arm_q31_to_float.c
${DSPLIB}/SupportFunctions/arm_q31_to_q15.c
${DSPLIB}/SupportFunctions/arm_q31_to_q7.c
${DSPLIB}/SupportFunctions/arm_q7_to_float.c
${DSPLIB}/SupportFunctions/arm_q7_to_q15.c
${DSPLIB}/SupportFunctions/arm_q7_to_q31.c
${DSPLIB}/TransformFunctions/arm_bitreversal.c
${DSPLIB}/TransformFunctions/arm_bitreversal2.S
${DSPLIB}/TransformFunctions/arm_cfft_f32.c
${DSPLIB}/TransformFunctions/arm_cfft_q15.c
${DSPLIB}/TransformFunctions/arm_cfft_q31.c
${DSPLIB}/TransformFunctions/arm_cfft_radix2_f32.c
${DSPLIB}/TransformFunctions/arm_cfft_radix2_init_f32.c
${DSPLIB}/TransformFunctions/arm_cfft_radix2_init_q15.c
${DSPLIB}/TransformFunctions/arm_cfft_radix2_init_q31.c
${DSPLIB}/TransformFunctions/arm_cfft_radix2_q15.c
${DSPLIB}/TransformFunctions/arm_cfft_radix2_q31.c
${DSPLIB}/TransformFunctions/arm_cfft_radix4_f32.c
${DSPLIB}/TransformFunctions/arm_cfft_radix4_init_f32.c
${DSPLIB}/TransformFunctions/arm_cfft_radix4_init_q15.c
${DSPLIB}/TransformFunctions/arm_cfft_radix4_init_q31.c
${DSPLIB}/TransformFunctions/arm_cfft_radix4_q15.c
${DSPLIB}/TransformFunctions/arm_cfft_radix4_q31.c
${DSPLIB}/TransformFunctions/arm_cfft_radix8_f32.c
${DSPLIB}/TransformFunctions/arm_dct4_f32.c
${DSPLIB}/TransformFunctions/arm_dct4_init_f32.c
${DSPLIB}/TransformFunctions/arm_dct4_init_q15.c
${DSPLIB}/TransformFunctions/arm_dct4_init_q31.c
${DSPLIB}/TransformFunctions/arm_dct4_q15.c
${DSPLIB}/TransformFunctions/arm_dct4_q31.c
${DSPLIB}/TransformFunctions/arm_rfft_f32.c
${DSPLIB}/TransformFunctions/arm_rfft_fast_f32.c
${DSPLIB}/TransformFunctions/arm_rfft_fast_init_f32.c
${DSPLIB}/TransformFunctions/arm_rfft_init_f32.c
${DSPLIB}/TransformFunctions/arm_rfft_init_q15.c
${DSPLIB}/TransformFunctions/arm_rfft_init_q31.c
${DSPLIB}/TransformFunctions/arm_rfft_q15.c
${DSPLIB}/TransformFunctions/arm_rfft_q31.c
)

add_library(CMSIS STATIC ${CMSIS_SRCS})
target_compile_options(CMSIS PRIVATE "-Wno-double-promotion")
