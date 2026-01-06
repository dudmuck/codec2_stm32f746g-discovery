/*
 * opus_wrapper.h - Opus codec wrapper providing codec2-like API
 *
 * This wrapper provides a simplified API similar to codec2 for easier
 * integration with existing LoRa transceiver code.
 */

#ifndef OPUS_WRAPPER_H
#define OPUS_WRAPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Opus wrapper modes - these define bitrate/quality presets
 *
 * All modes use 8kHz sample rate and 40ms frame size (320 samples)
 * to match codec2 behavior for LoRa streaming.
 *
 * LR2021 LoRa max throughput: ~100kbps at 1000kHz BW, SF5
 *
 * Approximate output sizes (VBR, actual may vary):
 *   OPUS_MODE_6K:   ~30 bytes/frame  (6 kbps)
 *   OPUS_MODE_8K:   ~40 bytes/frame  (8 kbps)
 *   OPUS_MODE_12K:  ~60 bytes/frame  (12 kbps)
 *   OPUS_MODE_16K:  ~80 bytes/frame  (16 kbps)
 *   OPUS_MODE_24K:  ~120 bytes/frame (24 kbps)
 *   OPUS_MODE_32K:  ~160 bytes/frame (32 kbps)
 *   OPUS_MODE_48K:  ~240 bytes/frame (48 kbps)
 *   OPUS_MODE_64K:  ~320 bytes/frame (64 kbps)
 *   OPUS_MODE_96K:  ~480 bytes/frame (96 kbps)
 */
#define OPUS_WRAPPER_MODE_6K    0   /* Lowest bitrate, ~6 kbps */
#define OPUS_WRAPPER_MODE_8K    1   /* ~8 kbps */
#define OPUS_WRAPPER_MODE_12K   2   /* ~12 kbps */
#define OPUS_WRAPPER_MODE_16K   3   /* ~16 kbps */
#define OPUS_WRAPPER_MODE_24K   4   /* ~24 kbps */
#define OPUS_WRAPPER_MODE_32K   5   /* ~32 kbps */
#define OPUS_WRAPPER_MODE_48K   6   /* ~48 kbps */
#define OPUS_WRAPPER_MODE_64K   7   /* ~64 kbps */
#define OPUS_WRAPPER_MODE_96K   8   /* ~96 kbps, near LR2021 max */

/* Sample rates based on mode:
 *   6K-12K:  8kHz  (narrowband, 4kHz audio bandwidth)
 *   16K-24K: 16kHz (wideband, 8kHz audio bandwidth)
 *   32K-96K: 48kHz (fullband, 20kHz audio bandwidth)
 */
#define OPUS_WRAPPER_SAMPLE_RATE_8K     8000
#define OPUS_WRAPPER_SAMPLE_RATE_16K    16000
#define OPUS_WRAPPER_SAMPLE_RATE_48K    48000

/* Frame duration in ms */
#define OPUS_WRAPPER_FRAME_MS       40

/* Samples per frame depends on sample rate (calculated at runtime) */
#define OPUS_WRAPPER_SAMPLES_PER_FRAME_8K   (OPUS_WRAPPER_SAMPLE_RATE_8K * OPUS_WRAPPER_FRAME_MS / 1000)   /* 320 */
#define OPUS_WRAPPER_SAMPLES_PER_FRAME_16K  (OPUS_WRAPPER_SAMPLE_RATE_16K * OPUS_WRAPPER_FRAME_MS / 1000)  /* 640 */
#define OPUS_WRAPPER_SAMPLES_PER_FRAME_48K  (OPUS_WRAPPER_SAMPLE_RATE_48K * OPUS_WRAPPER_FRAME_MS / 1000)  /* 1920 */

/* Maximum encoded frame size (for buffer allocation)
 * At 96kbps, 40ms frame: 96000 * 0.040 / 8 = 480 bytes */
#define OPUS_WRAPPER_MAX_FRAME_BYTES    512

/* Opaque wrapper state structure */
struct OPUS_WRAPPER;

/*
 * Create an Opus wrapper instance
 *
 * @param mode  One of OPUS_WRAPPER_MODE_* constants
 * @return      Pointer to wrapper state, or NULL on error
 */
struct OPUS_WRAPPER *opus_wrapper_create(int mode);

/*
 * Destroy an Opus wrapper instance
 *
 * @param ow    Wrapper state to destroy
 */
void opus_wrapper_destroy(struct OPUS_WRAPPER *ow);

/*
 * Encode a frame of audio
 *
 * @param ow        Wrapper state
 * @param pcm_in    Input PCM samples (320 samples for 40ms @ 8kHz)
 * @param bytes_out Output buffer for encoded data (must be at least OPUS_WRAPPER_MAX_FRAME_BYTES)
 * @return          Number of bytes written, or negative on error
 */
int opus_wrapper_encode(struct OPUS_WRAPPER *ow, const int16_t *pcm_in, uint8_t *bytes_out);

/*
 * Decode a frame of audio
 *
 * @param ow        Wrapper state
 * @param bytes_in  Input encoded data
 * @param num_bytes Number of bytes in input
 * @param pcm_out   Output PCM buffer (must hold 320 samples for 40ms @ 8kHz)
 * @return          Number of samples decoded, or negative on error
 */
int opus_wrapper_decode(struct OPUS_WRAPPER *ow, const uint8_t *bytes_in, int num_bytes, int16_t *pcm_out);

/*
 * Get samples per frame for this wrapper instance
 *
 * @param ow    Wrapper state
 * @return      Number of samples per frame (320 for 40ms @ 8kHz)
 */
int opus_wrapper_samples_per_frame(struct OPUS_WRAPPER *ow);

/*
 * Get the target bitrate in bits per second
 *
 * @param ow    Wrapper state
 * @return      Target bitrate in bps
 */
int opus_wrapper_get_bitrate(struct OPUS_WRAPPER *ow);

/*
 * Set the target bitrate in bits per second
 *
 * @param ow        Wrapper state
 * @param bitrate   Target bitrate in bps (6000-24000 recommended for voice)
 * @return          0 on success, negative on error
 */
int opus_wrapper_set_bitrate(struct OPUS_WRAPPER *ow, int bitrate);

/*
 * Get the current mode
 *
 * @param ow    Wrapper state
 * @return      Current mode (OPUS_WRAPPER_MODE_*)
 */
int opus_wrapper_get_mode(struct OPUS_WRAPPER *ow);

/*
 * Get the sample rate for this wrapper instance
 *
 * @param ow    Wrapper state
 * @return      Sample rate in Hz (8000, 16000, or 48000)
 */
int opus_wrapper_get_sample_rate(struct OPUS_WRAPPER *ow);

/*
 * Get the encoder complexity setting
 *
 * @param ow    Wrapper state
 * @return      Complexity (0-10, higher = better quality, more CPU)
 */
int opus_wrapper_get_complexity(struct OPUS_WRAPPER *ow);

/*
 * Get the recommended sample rate for a mode (static, no instance needed)
 *
 * @param mode  One of OPUS_WRAPPER_MODE_* constants
 * @return      Recommended sample rate in Hz (8000, 16000, or 48000)
 */
int opus_wrapper_mode_sample_rate(int mode);

/*
 * Perform packet loss concealment (decode with NULL input)
 *
 * @param ow        Wrapper state
 * @param pcm_out   Output PCM buffer (must hold 320 samples)
 * @return          Number of samples generated, or negative on error
 */
int opus_wrapper_decode_plc(struct OPUS_WRAPPER *ow, int16_t *pcm_out);

#ifdef __cplusplus
}
#endif

#endif /* OPUS_WRAPPER_H */
