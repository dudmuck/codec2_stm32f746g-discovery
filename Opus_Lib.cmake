# Opus library configuration for STM32F746G (Cortex-M7)
# This builds Opus as a static library for embedded ARM targets
# Uses FIXED-POINT mode with ARM DSP optimizations for better performance

set(OPUS_SRC ${PROJECT_SOURCE_DIR}/opus)

# Core CELT sources
set(CELT_SOURCES
    ${OPUS_SRC}/celt/bands.c
    ${OPUS_SRC}/celt/celt.c
    ${OPUS_SRC}/celt/celt_encoder.c
    ${OPUS_SRC}/celt/celt_decoder.c
    ${OPUS_SRC}/celt/cwrs.c
    ${OPUS_SRC}/celt/entcode.c
    ${OPUS_SRC}/celt/entdec.c
    ${OPUS_SRC}/celt/entenc.c
    ${OPUS_SRC}/celt/kiss_fft.c
    ${OPUS_SRC}/celt/laplace.c
    ${OPUS_SRC}/celt/mathops.c
    ${OPUS_SRC}/celt/mdct.c
    ${OPUS_SRC}/celt/modes.c
    ${OPUS_SRC}/celt/pitch.c
    ${OPUS_SRC}/celt/celt_lpc.c
    ${OPUS_SRC}/celt/quant_bands.c
    ${OPUS_SRC}/celt/rate.c
    ${OPUS_SRC}/celt/vq.c
)

# Core SILK sources (common to fixed and float)
set(SILK_SOURCES
    ${OPUS_SRC}/silk/CNG.c
    ${OPUS_SRC}/silk/code_signs.c
    ${OPUS_SRC}/silk/init_decoder.c
    ${OPUS_SRC}/silk/decode_core.c
    ${OPUS_SRC}/silk/decode_frame.c
    ${OPUS_SRC}/silk/decode_parameters.c
    ${OPUS_SRC}/silk/decode_indices.c
    ${OPUS_SRC}/silk/decode_pulses.c
    ${OPUS_SRC}/silk/decoder_set_fs.c
    ${OPUS_SRC}/silk/dec_API.c
    ${OPUS_SRC}/silk/enc_API.c
    ${OPUS_SRC}/silk/encode_indices.c
    ${OPUS_SRC}/silk/encode_pulses.c
    ${OPUS_SRC}/silk/gain_quant.c
    ${OPUS_SRC}/silk/interpolate.c
    ${OPUS_SRC}/silk/LP_variable_cutoff.c
    ${OPUS_SRC}/silk/NLSF_decode.c
    ${OPUS_SRC}/silk/NSQ.c
    ${OPUS_SRC}/silk/NSQ_del_dec.c
    ${OPUS_SRC}/silk/PLC.c
    ${OPUS_SRC}/silk/shell_coder.c
    ${OPUS_SRC}/silk/tables_gain.c
    ${OPUS_SRC}/silk/tables_LTP.c
    ${OPUS_SRC}/silk/tables_NLSF_CB_NB_MB.c
    ${OPUS_SRC}/silk/tables_NLSF_CB_WB.c
    ${OPUS_SRC}/silk/tables_other.c
    ${OPUS_SRC}/silk/tables_pitch_lag.c
    ${OPUS_SRC}/silk/tables_pulses_per_block.c
    ${OPUS_SRC}/silk/VAD.c
    ${OPUS_SRC}/silk/control_audio_bandwidth.c
    ${OPUS_SRC}/silk/quant_LTP_gains.c
    ${OPUS_SRC}/silk/VQ_WMat_EC.c
    ${OPUS_SRC}/silk/HP_variable_cutoff.c
    ${OPUS_SRC}/silk/NLSF_encode.c
    ${OPUS_SRC}/silk/NLSF_VQ.c
    ${OPUS_SRC}/silk/NLSF_unpack.c
    ${OPUS_SRC}/silk/NLSF_del_dec_quant.c
    ${OPUS_SRC}/silk/process_NLSFs.c
    ${OPUS_SRC}/silk/stereo_LR_to_MS.c
    ${OPUS_SRC}/silk/stereo_MS_to_LR.c
    ${OPUS_SRC}/silk/check_control_input.c
    ${OPUS_SRC}/silk/control_SNR.c
    ${OPUS_SRC}/silk/init_encoder.c
    ${OPUS_SRC}/silk/control_codec.c
    ${OPUS_SRC}/silk/A2NLSF.c
    ${OPUS_SRC}/silk/ana_filt_bank_1.c
    ${OPUS_SRC}/silk/biquad_alt.c
    ${OPUS_SRC}/silk/bwexpander_32.c
    ${OPUS_SRC}/silk/bwexpander.c
    ${OPUS_SRC}/silk/debug.c
    ${OPUS_SRC}/silk/decode_pitch.c
    ${OPUS_SRC}/silk/inner_prod_aligned.c
    ${OPUS_SRC}/silk/lin2log.c
    ${OPUS_SRC}/silk/log2lin.c
    ${OPUS_SRC}/silk/LPC_analysis_filter.c
    ${OPUS_SRC}/silk/LPC_inv_pred_gain.c
    ${OPUS_SRC}/silk/table_LSF_cos.c
    ${OPUS_SRC}/silk/NLSF2A.c
    ${OPUS_SRC}/silk/NLSF_stabilize.c
    ${OPUS_SRC}/silk/NLSF_VQ_weights_laroia.c
    ${OPUS_SRC}/silk/pitch_est_tables.c
    ${OPUS_SRC}/silk/resampler.c
    ${OPUS_SRC}/silk/resampler_down2_3.c
    ${OPUS_SRC}/silk/resampler_down2.c
    ${OPUS_SRC}/silk/resampler_private_AR2.c
    ${OPUS_SRC}/silk/resampler_private_down_FIR.c
    ${OPUS_SRC}/silk/resampler_private_IIR_FIR.c
    ${OPUS_SRC}/silk/resampler_private_up2_HQ.c
    ${OPUS_SRC}/silk/resampler_rom.c
    ${OPUS_SRC}/silk/sigm_Q15.c
    ${OPUS_SRC}/silk/sort.c
    ${OPUS_SRC}/silk/sum_sqr_shift.c
    ${OPUS_SRC}/silk/stereo_decode_pred.c
    ${OPUS_SRC}/silk/stereo_encode_pred.c
    ${OPUS_SRC}/silk/stereo_find_predictor.c
    ${OPUS_SRC}/silk/stereo_quant_pred.c
    ${OPUS_SRC}/silk/LPC_fit.c
)

# SILK FIXED-POINT sources (optimized for embedded ARM with DSP instructions)
set(SILK_SOURCES_FIXED
    ${OPUS_SRC}/silk/fixed/LTP_analysis_filter_FIX.c
    ${OPUS_SRC}/silk/fixed/LTP_scale_ctrl_FIX.c
    ${OPUS_SRC}/silk/fixed/corrMatrix_FIX.c
    ${OPUS_SRC}/silk/fixed/encode_frame_FIX.c
    ${OPUS_SRC}/silk/fixed/find_LPC_FIX.c
    ${OPUS_SRC}/silk/fixed/find_LTP_FIX.c
    ${OPUS_SRC}/silk/fixed/find_pitch_lags_FIX.c
    ${OPUS_SRC}/silk/fixed/find_pred_coefs_FIX.c
    ${OPUS_SRC}/silk/fixed/noise_shape_analysis_FIX.c
    ${OPUS_SRC}/silk/fixed/process_gains_FIX.c
    ${OPUS_SRC}/silk/fixed/regularize_correlations_FIX.c
    ${OPUS_SRC}/silk/fixed/residual_energy16_FIX.c
    ${OPUS_SRC}/silk/fixed/residual_energy_FIX.c
    ${OPUS_SRC}/silk/fixed/warped_autocorrelation_FIX.c
    ${OPUS_SRC}/silk/fixed/apply_sine_window_FIX.c
    ${OPUS_SRC}/silk/fixed/autocorr_FIX.c
    ${OPUS_SRC}/silk/fixed/burg_modified_FIX.c
    ${OPUS_SRC}/silk/fixed/k2a_FIX.c
    ${OPUS_SRC}/silk/fixed/k2a_Q16_FIX.c
    ${OPUS_SRC}/silk/fixed/pitch_analysis_core_FIX.c
    ${OPUS_SRC}/silk/fixed/vector_ops_FIX.c
    ${OPUS_SRC}/silk/fixed/schur64_FIX.c
    ${OPUS_SRC}/silk/fixed/schur_FIX.c
)

# Core Opus sources
set(OPUS_SOURCES
    ${OPUS_SRC}/src/opus.c
    ${OPUS_SRC}/src/opus_decoder.c
    ${OPUS_SRC}/src/opus_encoder.c
    ${OPUS_SRC}/src/extensions.c
    ${OPUS_SRC}/src/opus_multistream.c
    ${OPUS_SRC}/src/opus_multistream_encoder.c
    ${OPUS_SRC}/src/opus_multistream_decoder.c
    ${OPUS_SRC}/src/repacketizer.c
    ${OPUS_SRC}/src/opus_projection_encoder.c
    ${OPUS_SRC}/src/opus_projection_decoder.c
    ${OPUS_SRC}/src/mapping_matrix.c
)

# Opus floating-point analysis (still needed for some features)
set(OPUS_SOURCES_FLOAT
    ${OPUS_SRC}/src/analysis.c
    ${OPUS_SRC}/src/mlp.c
    ${OPUS_SRC}/src/mlp_data.c
)

# Combine all sources - using FIXED-POINT SILK for ARM DSP optimization
set(OPUS_ALL_SRCS
    ${CELT_SOURCES}
    ${SILK_SOURCES}
    ${SILK_SOURCES_FIXED}
    ${OPUS_SOURCES}
    ${OPUS_SOURCES_FLOAT}
)

# Create static library
add_library(opus STATIC ${OPUS_ALL_SRCS})

# Include directories
target_include_directories(opus PUBLIC
    ${OPUS_SRC}/include
)
target_include_directories(opus PRIVATE
    ${OPUS_SRC}
    ${OPUS_SRC}/celt
    ${OPUS_SRC}/celt/arm
    ${OPUS_SRC}/silk
    ${OPUS_SRC}/silk/fixed
    ${OPUS_SRC}/silk/arm
    ${OPUS_SRC}/src
)

# Compile definitions for embedded ARM Cortex-M7 with FIXED-POINT
# FIXED_POINT enables integer DSP math instead of floating-point
# OPUS_ARM_INLINE_ASM enables ARMv4 inline assembly
# OPUS_ARM_INLINE_EDSP enables ARMv5E DSP instructions (smulbb, smlabb, etc.)
# OPUS_ARM_INLINE_MEDIA enables ARMv6 media instructions (ssat, etc.)
target_compile_definitions(opus PRIVATE
    OPUS_BUILD
    HAVE_LRINTF
    HAVE_LRINT
    VAR_ARRAYS
    FIXED_POINT=1
    OPUS_ARM_INLINE_ASM
    OPUS_ARM_INLINE_EDSP
    OPUS_ARM_INLINE_MEDIA
    DISABLE_FLOAT_API
)

# Suppress some warnings in Opus code
target_compile_options(opus PRIVATE
    -Wno-maybe-uninitialized
    -Wno-unused-parameter
)

# Opus wrapper library - provides codec2-like API
set(OPUS_WRAPPER_SRCS
    ${PROJECT_SOURCE_DIR}/opus_wrapper/opus_wrapper.c
)

add_library(opus_wrapper STATIC ${OPUS_WRAPPER_SRCS})

target_include_directories(opus_wrapper PUBLIC
    ${PROJECT_SOURCE_DIR}/opus_wrapper
    ${OPUS_SRC}/include
)

target_link_libraries(opus_wrapper opus)
