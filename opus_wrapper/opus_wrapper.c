/*
 * opus_wrapper.c - Opus codec wrapper implementation
 */

#include "opus_wrapper.h"
#include "opus.h"
#include <stdlib.h>
#include <string.h>

/* Bitrate lookup table for modes (in bps) */
static const int mode_bitrates[] = {
    6000,   /* OPUS_WRAPPER_MODE_6K */
    8000,   /* OPUS_WRAPPER_MODE_8K */
    12000,  /* OPUS_WRAPPER_MODE_12K */
    16000,  /* OPUS_WRAPPER_MODE_16K */
    24000,  /* OPUS_WRAPPER_MODE_24K */
    32000,  /* OPUS_WRAPPER_MODE_32K */
    48000,  /* OPUS_WRAPPER_MODE_48K */
    64000,  /* OPUS_WRAPPER_MODE_64K */
    96000,  /* OPUS_WRAPPER_MODE_96K */
};

#define NUM_MODES (sizeof(mode_bitrates) / sizeof(mode_bitrates[0]))

/* Internal wrapper state structure */
struct OPUS_WRAPPER {
    OpusEncoder *encoder;
    OpusDecoder *decoder;
    int mode;
    int bitrate;
    int sample_rate;
    int frame_ms;
    int samples_per_frame;
};

/* Get recommended sample rate for a mode */
static int get_mode_sample_rate(int mode)
{
    switch (mode) {
        case OPUS_WRAPPER_MODE_6K:
        case OPUS_WRAPPER_MODE_8K:
        case OPUS_WRAPPER_MODE_12K:
            return OPUS_WRAPPER_SAMPLE_RATE_8K;   /* narrowband */
        case OPUS_WRAPPER_MODE_16K:
        case OPUS_WRAPPER_MODE_24K:
            return OPUS_WRAPPER_SAMPLE_RATE_16K;  /* wideband */
        case OPUS_WRAPPER_MODE_32K:
        case OPUS_WRAPPER_MODE_48K:
        case OPUS_WRAPPER_MODE_64K:
        case OPUS_WRAPPER_MODE_96K:
        default:
            return OPUS_WRAPPER_SAMPLE_RATE_48K;  /* fullband */
    }
}

/* Get Opus bandwidth constant for a sample rate */
static int get_bandwidth_for_rate(int sample_rate)
{
    switch (sample_rate) {
        case OPUS_WRAPPER_SAMPLE_RATE_8K:
            return OPUS_BANDWIDTH_NARROWBAND;     /* 4kHz audio BW */
        case OPUS_WRAPPER_SAMPLE_RATE_16K:
            return OPUS_BANDWIDTH_WIDEBAND;       /* 8kHz audio BW */
        case OPUS_WRAPPER_SAMPLE_RATE_48K:
        default:
            return OPUS_BANDWIDTH_FULLBAND;       /* 20kHz audio BW */
    }
}

struct OPUS_WRAPPER *opus_wrapper_create(int mode)
{
    return opus_wrapper_create_ex(mode, OPUS_WRAPPER_FRAME_MS_DEFAULT);
}

struct OPUS_WRAPPER *opus_wrapper_create_ex(int mode, int frame_ms)
{
    struct OPUS_WRAPPER *ow;
    int error;
    int bitrate;
    int sample_rate;
    int bandwidth;

    /* Validate mode */
    if (mode < 0 || mode >= (int)NUM_MODES) {
        return NULL;
    }

    /* Validate frame_ms - Opus supports 2.5, 5, 10, 20, 40, 60 ms */
    if (frame_ms != 2 && frame_ms != 5 && frame_ms != 10 &&
        frame_ms != 20 && frame_ms != 40 && frame_ms != 60) {
        return NULL;
    }

    /* Get sample rate and bandwidth for this mode */
    sample_rate = get_mode_sample_rate(mode);
    bandwidth = get_bandwidth_for_rate(sample_rate);

    /* Allocate wrapper state */
    ow = (struct OPUS_WRAPPER *)malloc(sizeof(struct OPUS_WRAPPER));
    if (ow == NULL) {
        return NULL;
    }
    memset(ow, 0, sizeof(struct OPUS_WRAPPER));

    /* Create encoder */
    ow->encoder = opus_encoder_create(
        sample_rate,
        1,                           /* mono */
        OPUS_APPLICATION_VOIP,       /* optimized for voice */
        &error
    );
    if (error != OPUS_OK || ow->encoder == NULL) {
        free(ow);
        return NULL;
    }

    /* Create decoder */
    ow->decoder = opus_decoder_create(
        sample_rate,
        1,                           /* mono */
        &error
    );
    if (error != OPUS_OK || ow->decoder == NULL) {
        opus_encoder_destroy(ow->encoder);
        free(ow);
        return NULL;
    }

    /* Store mode, sample rate, frame duration, and set bitrate */
    ow->mode = mode;
    ow->sample_rate = sample_rate;
    ow->frame_ms = frame_ms;
    bitrate = mode_bitrates[mode];
    ow->bitrate = bitrate;
    /* Calculate samples per frame: sample_rate * frame_ms / 1000 */
    ow->samples_per_frame = sample_rate * frame_ms / 1000;

    /* Configure encoder */
    opus_encoder_ctl(ow->encoder, OPUS_SET_BITRATE(bitrate));

    /* Set complexity - with fixed-point ARM DSP we have CPU headroom for quality */
    opus_encoder_ctl(ow->encoder, OPUS_SET_COMPLEXITY(5));

    /* Disable VBR for fixed frame size (required for radio transmission
     * where we don't transmit per-frame length information) */
    opus_encoder_ctl(ow->encoder, OPUS_SET_VBR(0));

    /* Set signal type to voice */
    opus_encoder_ctl(ow->encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));

    /* Disable DTX (discontinuous transmission) for streaming */
    opus_encoder_ctl(ow->encoder, OPUS_SET_DTX(0));

    /* Set bandwidth appropriate for this sample rate */
    opus_encoder_ctl(ow->encoder, OPUS_SET_MAX_BANDWIDTH(bandwidth));

    return ow;
}

void opus_wrapper_destroy(struct OPUS_WRAPPER *ow)
{
    if (ow == NULL) {
        return;
    }

    if (ow->encoder != NULL) {
        opus_encoder_destroy(ow->encoder);
    }

    if (ow->decoder != NULL) {
        opus_decoder_destroy(ow->decoder);
    }

    free(ow);
}

int opus_wrapper_encode(struct OPUS_WRAPPER *ow, const int16_t *pcm_in, uint8_t *bytes_out)
{
    opus_int32 len;

    if (ow == NULL || pcm_in == NULL || bytes_out == NULL) {
        return -1;
    }

    /* Encode frame */
    len = opus_encode(
        ow->encoder,
        pcm_in,
        ow->samples_per_frame,
        bytes_out,
        OPUS_WRAPPER_MAX_FRAME_BYTES
    );

    return (int)len;
}

int opus_wrapper_decode(struct OPUS_WRAPPER *ow, const uint8_t *bytes_in, int num_bytes, int16_t *pcm_out)
{
    int samples;

    if (ow == NULL || bytes_in == NULL || pcm_out == NULL || num_bytes <= 0) {
        return -1;
    }

    /* Decode frame */
    samples = opus_decode(
        ow->decoder,
        bytes_in,
        num_bytes,
        pcm_out,
        ow->samples_per_frame,
        0   /* no FEC */
    );

    return samples;
}

int opus_wrapper_samples_per_frame(struct OPUS_WRAPPER *ow)
{
    if (ow == NULL) {
        return OPUS_WRAPPER_SAMPLES_PER_FRAME_8K;  /* default */
    }
    return ow->samples_per_frame;
}

int opus_wrapper_get_bitrate(struct OPUS_WRAPPER *ow)
{
    if (ow == NULL) {
        return 0;
    }
    return ow->bitrate;
}

int opus_wrapper_set_bitrate(struct OPUS_WRAPPER *ow, int bitrate)
{
    int ret;

    if (ow == NULL || bitrate < 500 || bitrate > 512000) {
        return -1;
    }

    ret = opus_encoder_ctl(ow->encoder, OPUS_SET_BITRATE(bitrate));
    if (ret == OPUS_OK) {
        ow->bitrate = bitrate;
        return 0;
    }

    return -1;
}

int opus_wrapper_get_mode(struct OPUS_WRAPPER *ow)
{
    if (ow == NULL) {
        return -1;
    }
    return ow->mode;
}

int opus_wrapper_get_sample_rate(struct OPUS_WRAPPER *ow)
{
    if (ow == NULL) {
        return OPUS_WRAPPER_SAMPLE_RATE_8K;  /* default */
    }
    return ow->sample_rate;
}

int opus_wrapper_get_frame_ms(struct OPUS_WRAPPER *ow)
{
    if (ow == NULL) {
        return OPUS_WRAPPER_FRAME_MS_DEFAULT;  /* default */
    }
    return ow->frame_ms;
}

int opus_wrapper_get_complexity(struct OPUS_WRAPPER *ow)
{
    opus_int32 complexity = 0;
    if (ow == NULL || ow->encoder == NULL) {
        return -1;
    }
    opus_encoder_ctl(ow->encoder, OPUS_GET_COMPLEXITY(&complexity));
    return (int)complexity;
}

int opus_wrapper_mode_sample_rate(int mode)
{
    return get_mode_sample_rate(mode);
}

int opus_wrapper_decode_plc(struct OPUS_WRAPPER *ow, int16_t *pcm_out)
{
    int samples;

    if (ow == NULL || pcm_out == NULL) {
        return -1;
    }

    /* Decode with NULL pointer triggers PLC */
    samples = opus_decode(
        ow->decoder,
        NULL,
        0,
        pcm_out,
        ow->samples_per_frame,
        0
    );

    return samples;
}
