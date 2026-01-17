/*
 * FreeRTOS task implementations for concurrent Opus encoding and radio TX
 */

#ifdef USE_FREERTOS

#include "freertos_tasks.h"
#include "main.h"
#include "opus_wrapper.h"
#include "radio.h"  /* For lorahal_t and lorahal */
#include <stdio.h>
#include <string.h>
#include "event_groups.h"

/* Event group bits for low-latency wake on multiple events */
#define EVENT_FRAME_READY  (1 << 0)
#define EVENT_TX_DONE      (1 << 1)

/* lorahal is declared in radio.h - access it here */
extern lorahal_t lorahal;

/* External references from lora_xcvr.c */
extern struct OPUS_WRAPPER *ow;
extern uint16_t *audio_buffer_in;
extern uint16_t *audio_buffer_in_B;
extern unsigned audio_block_size;
extern volatile uint16_t tx_buf_idx;
extern volatile uint8_t txing;
extern uint16_t _bytes_per_frame;
extern volatile uint8_t user_button_pressed;
extern unsigned nsamp;
extern unsigned navgs_;
extern uint16_t encoder_sine_step;
#ifdef ENABLE_TEST_AUDIO
extern uint8_t audio_test_mode;
extern const int16_t *test_audio_ptr;
extern uint32_t test_audio_len;
extern uint32_t audio_test_idx;
#endif
extern bool micRightEn;
extern bool micLeftEn;

/* Semaphore handles */
SemaphoreHandle_t xAudioHalfSemaphore = NULL;
SemaphoreHandle_t xAudioFullSemaphore = NULL;
SemaphoreHandle_t xTxDoneSemaphore = NULL;
SemaphoreHandle_t xRadioServiceSemaphore = NULL;  /* Radio DIO8 IRQ signaling */

/* Event group for low-latency wake on frame ready OR TX done */
static EventGroupHandle_t xTxEventGroup = NULL;

/* Mutex for tx_buf access */
SemaphoreHandle_t xTxBufMutex = NULL;

/* Task handles */
TaskHandle_t xMainTaskHandle = NULL;
TaskHandle_t xEncoderTaskHandle = NULL;
TaskHandle_t xTxTaskHandle = NULL;
TaskHandle_t xRadioServiceTaskHandle = NULL;

/* Stream buffer for encoded data */
StreamBufferHandle_t xTxStreamBuffer = NULL;

/* Task active flag */
static volatile uint8_t tasks_active = 0;

/* Concurrent encoding enabled flag */
static volatile uint8_t concurrent_encode_enabled = 0;

/* TX session active flag - encoder task only runs during TX */
static volatile uint8_t tx_session_active = 0;

/* Encoder state for debugging */
static volatile const char* encoder_state = "INIT";

/* Multi-buffered encoded frames for true pipelining:
 * - Encoder writes to buf[write_idx] while main reads from buf[read_idx]
 * - When encoder finishes, it advances write_idx (if read has caught up)
 * - When main consumes, it advances read_idx
 * - 4 buffers allows encoder to work 3 frames ahead, absorbing timing jitter */
#define NUM_ENCODE_BUFS 4
static uint8_t encoded_frame_buf[NUM_ENCODE_BUFS][768];  /* Must hold _bytes_per_frame (720 for 96K@60ms) */
static volatile int32_t encoded_frame_len[NUM_ENCODE_BUFS] = {0};
static volatile uint8_t frame_ready[NUM_ENCODE_BUFS] = {0};
static volatile uint8_t write_idx = 0;  /* Buffer encoder writes to */
static volatile uint8_t read_idx = 0;   /* Buffer main reads from */
static SemaphoreHandle_t xEncodedFrameMutex = NULL;

/* Semaphore to signal main when a frame is ready (avoids polling) */
static SemaphoreHandle_t xFrameReadySemaphore = NULL;

/* External functions from lora_xcvr.c */
extern void AudioLoopback_demo(void);
extern void svc_uart(void);

/* Forward declarations */
static void vMainTask(void *pvParameters);
static void vEncoderTask(void *pvParameters);
static void vTxTask(void *pvParameters);
static void vRadioServiceTask(void *pvParameters);

/*
 * Initialize FreeRTOS primitives and create tasks
 */
void freertos_tasks_init(void)
{
    /* Create binary semaphores for audio callbacks */
    xAudioHalfSemaphore = xSemaphoreCreateBinary();
    xAudioFullSemaphore = xSemaphoreCreateBinary();
    xTxDoneSemaphore = xSemaphoreCreateBinary();
    xRadioServiceSemaphore = xSemaphoreCreateBinary();  /* Radio IRQ signaling */
    xTxBufMutex = xSemaphoreCreateMutex();
    xEncodedFrameMutex = xSemaphoreCreateMutex();
    /* Counting semaphore for frame ready signaling (up to NUM_ENCODE_BUFS frames) */
    xFrameReadySemaphore = xSemaphoreCreateCounting(NUM_ENCODE_BUFS, 0);

    /* Event group for waiting on frame ready OR TX done */
    xTxEventGroup = xEventGroupCreate();

    if (xAudioHalfSemaphore == NULL || xAudioFullSemaphore == NULL ||
        xTxDoneSemaphore == NULL || xRadioServiceSemaphore == NULL ||
        xTxBufMutex == NULL ||
        xEncodedFrameMutex == NULL || xFrameReadySemaphore == NULL ||
        xTxEventGroup == NULL) {
        printf("\e[31mFreeRTOS: semaphore/event creation failed!\e[0m\r\n");
        return;
    }

    /* Create stream buffer for encoded data */
    xTxStreamBuffer = xStreamBufferCreate(TX_STREAM_BUFFER_SIZE, TX_STREAM_TRIGGER_LEVEL);
    if (xTxStreamBuffer == NULL) {
        printf("\e[31mFreeRTOS: stream buffer creation failed!\e[0m\r\n");
        return;
    }

    /* Create main task - handles initialization and main loop.
     * This task calls AudioLoopback_demo() which contains the existing
     * initialization and main loop code. */
    BaseType_t ret = xTaskCreate(
        vMainTask,
        "Main",
        MAIN_TASK_STACK_SIZE,
        NULL,
        MAIN_TASK_PRIORITY,
        &xMainTaskHandle
    );
    if (ret != pdPASS) {
        printf("\e[31mFreeRTOS: main task creation failed!\e[0m\r\n");
        return;
    }

    /* Create encoder task - handles audio encoding in parallel with TX.
     * Task is created but waits for concurrent_encode_enabled flag. */
    ret = xTaskCreate(
        vEncoderTask,
        "Encoder",
        ENCODER_TASK_STACK_SIZE,
        NULL,
        ENCODER_TASK_PRIORITY,
        &xEncoderTaskHandle
    );
    if (ret != pdPASS) {
        printf("\e[31mFreeRTOS: encoder task creation failed!\e[0m\r\n");
        return;
    }

    /* Create radio service task - highest priority to preempt decode and
     * prevent FIFO overflow. This task wakes on DIO8 IRQ and calls
     * lorahal.service() to read the RX FIFO before it overflows. */
    ret = xTaskCreate(
        vRadioServiceTask,
        "RadioSvc",
        RADIO_SERVICE_STACK_SIZE,
        NULL,
        RADIO_SERVICE_TASK_PRIORITY,
        &xRadioServiceTaskHandle
    );
    if (ret != pdPASS) {
        printf("\e[31mFreeRTOS: radio service task creation failed!\e[0m\r\n");
        return;
    }

    printf("FreeRTOS: tasks initialized\r\n");
    tasks_active = 1;
}

/*
 * Start FreeRTOS scheduler
 */
void freertos_start(void)
{
    printf("FreeRTOS: starting scheduler\r\n");
    vTaskStartScheduler();
    /* Should never reach here */
    printf("\e[31mFreeRTOS: scheduler returned!\e[0m\r\n");
    for (;;);
}

/*
 * Signal from ISR: audio half buffer ready
 */
void freertos_audio_half_ready_FromISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xAudioHalfSemaphore != NULL) {
        xSemaphoreGiveFromISR(xAudioHalfSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*
 * Signal from ISR: audio full buffer ready
 */
void freertos_audio_full_ready_FromISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xAudioFullSemaphore != NULL) {
        xSemaphoreGiveFromISR(xAudioFullSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*
 * Signal from ISR: TX complete
 * Uses direct task notification for instant wake (no daemon task delay)
 */
void freertos_tx_done_FromISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xTxDoneSemaphore != NULL) {
        xSemaphoreGiveFromISR(xTxDoneSemaphore, &xHigherPriorityTaskWoken);
    }
    /* Direct task notification for instant wake - no timer daemon delay */
    if (xMainTaskHandle != NULL) {
        xTaskNotifyFromISR(xMainTaskHandle, EVENT_TX_DONE, eSetBits, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * Signal from ISR: Radio DIO8 interrupt
 * Wakes radio service task to read FIFO before overflow
 */
void freertos_radio_irq_FromISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xRadioServiceSemaphore != NULL) {
        xSemaphoreGiveFromISR(xRadioServiceSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*
 * Check if tasks are active
 */
uint8_t freertos_tasks_active(void)
{
    return tasks_active;
}

/*
 * Get encoder state for debugging
 */
const char* freertos_get_encoder_state(void)
{
    return encoder_state;
}

/*
 * Enable concurrent encoder task
 * Call this after AudioLoopback_demo has initialized audio
 */
void freertos_enable_concurrent_encode(void)
{
    concurrent_encode_enabled = 1;
    printf("FreeRTOS: concurrent encode enabled\r\n");
}

/*
 * Check if concurrent encoding is enabled
 */
uint8_t freertos_concurrent_encode_enabled(void)
{
    return concurrent_encode_enabled;
}

/*
 * Signal TX session start - encoder task begins processing
 */
void freertos_tx_session_start(void)
{
    printf("TX_SESSION_START\r\n");
    tx_session_active = 1;
    /* Reset multi-buffer state */
    write_idx = 0;
    read_idx = 0;
    for (int i = 0; i < NUM_ENCODE_BUFS; i++) {
        frame_ready[i] = 0;
        encoded_frame_len[i] = 0;
    }
    /* Clear any pending semaphores from previous session */
    xSemaphoreTake(xAudioHalfSemaphore, 0);
    xSemaphoreTake(xAudioFullSemaphore, 0);
    while (xSemaphoreTake(xFrameReadySemaphore, 0) == pdTRUE) {}  /* Clear counting semaphore */
    /* Clear pending task notifications */
    xTaskNotifyStateClear(xMainTaskHandle);
}

/*
 * Signal TX session end - encoder task stops
 */
void freertos_tx_session_end(void)
{
    tx_session_active = 0;
    for (int i = 0; i < NUM_ENCODE_BUFS; i++) {
        frame_ready[i] = 0;
    }
}

/*
 * Check if an encoded frame is ready (called by main loop)
 * Returns encoded length, or 0 if not ready, or -1 on error
 * Uses double-buffering: reads from read_idx buffer while encoder writes to write_idx
 *
 * Uses TASK NOTIFICATIONS for INSTANT wake on either:
 *   - FRAME_READY: encoder completed a frame
 *   - TX_DONE: radio finished transmitting (needs service call)
 * Task notifications work directly from ISR without timer daemon delay.
 */
int32_t freertos_get_encoded_frame(uint8_t *dest, uint16_t max_len)
{
    const TickType_t poll_interval = pdMS_TO_TICKS(5);  /* 5ms poll interval */
    const int max_iterations = 10;  /* 50ms total timeout */
    int iterations = 0;

    for (;;) {
        /* First check if frame is already available (non-blocking) */
        if (xSemaphoreTake(xFrameReadySemaphore, 0) == pdTRUE) {
            uint8_t ridx = read_idx;
            if (frame_ready[ridx]) {
                int32_t len = encoded_frame_len[ridx];
                if (len > 0 && len <= max_len) {
                    memcpy(dest, encoded_frame_buf[ridx], len);
                    frame_ready[ridx] = 0;  /* Mark as consumed */
                    read_idx = (ridx + 1) % NUM_ENCODE_BUFS;
                    return len;
                }
            }
        }

        /* Service UART to keep it responsive during TX.
         * Without this, UART can become unresponsive for up to 50ms per frame. */
        svc_uart();

        /* Wait for task notification with short timeout */
        uint32_t notif_value = 0;
        BaseType_t got_notif = xTaskNotifyWait(
            0,                              /* Don't clear bits on entry */
            EVENT_FRAME_READY | EVENT_TX_DONE,  /* Clear these bits on exit */
            &notif_value,
            poll_interval
        );

        if (got_notif == pdTRUE) {
            /* If TX_DONE bit is set, service the radio immediately */
            if (notif_value & EVENT_TX_DONE) {
                lorahal.service();
            }
            /* If FRAME_READY bit is set, loop back to grab the frame */
            iterations = 0;  /* Reset timeout on any event */
        } else {
            /* Timeout - check if we've waited long enough */
            if (++iterations >= max_iterations) {
                return 0;  /* Total timeout reached */
            }
        }
    }
}

/*
 * Main task: handles initialization and runs the main loop.
 * This wraps the existing AudioLoopback_demo() function which
 * contains all the audio init, radio init, and main processing loop.
 */
static void vMainTask(void *pvParameters)
{
    (void)pvParameters;

    printf("FreeRTOS: Main task started\r\n");

    /* Call the existing main loop function.
     * This contains all initialization and the main processing loop.
     * The loop handles audio capture, encoding, and TX. */
    AudioLoopback_demo();

    /* AudioLoopback_demo() should never return, but if it does... */
    printf("\e[31mFreeRTOS: Main task ended unexpectedly!\e[0m\r\n");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*
 * Encoder task: waits for audio, encodes to Opus, stores for main loop pickup.
 * Uses double-buffering: encodes into write_idx buffer while main reads from read_idx.
 * This allows encoder to work ahead by one frame for true pipelining.
 */
static void vEncoderTask(void *pvParameters)
{
    (void)pvParameters;
    int32_t encoded_len;
    static uint8_t first_encode = 1;
    static uint32_t enc_count = 0;
    /* Use global enc_time_max from lora_xcvr.c so it gets printed in TX end message */
    extern uint32_t enc_time_max;

    encoder_state = "DISABLED";
    printf("ENC_TASK: started\r\n");

    for (;;) {
        /* Wait until concurrent encoding is enabled */
        if (!concurrent_encode_enabled) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Wait until TX session is active */
        if (!tx_session_active) {
            encoder_state = "IDLE";
            first_encode = 1;  /* Reset for next session */
            enc_count = 0;
            /* Note: enc_time_max is reset in lora_xcvr.c after "TX end" print */
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Check if write buffer is available (not yet consumed by main) */
        uint8_t widx = write_idx;
        if (frame_ready[widx]) {
            /* Write buffer still has unconsumed data - wait briefly */
            encoder_state = "FULL";
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /* Wait for audio ready semaphore */
        encoder_state = "WAIT";
        BaseType_t half_ready = pdFALSE;
        BaseType_t full_ready = pdFALSE;

        /* First try non-blocking check of both semaphores */
        if (xSemaphoreTake(xAudioHalfSemaphore, 0) == pdTRUE) {
            half_ready = pdTRUE;
        } else if (xSemaphoreTake(xAudioFullSemaphore, 0) == pdTRUE) {
            full_ready = pdTRUE;
        } else {
            /* Neither ready - wait for either with short timeout */
            if (xSemaphoreTake(xAudioHalfSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                half_ready = pdTRUE;
            } else if (xSemaphoreTake(xAudioFullSemaphore, 0) == pdTRUE) {
                full_ready = pdTRUE;
            } else {
                /* Timeout - check if session still active */
                continue;
            }
        }

        if (first_encode) {
            printf("ENC_TASK: got first audio (%s)\r\n", half_ready ? "HALF" : "FULL");
            first_encode = 0;
        }

        encoder_state = "ENCODE";

        /* Get pointer to audio buffer (half or full)
         * HALF uses audio_buffer_in, FULL uses audio_buffer_in_B
         * Audio buffers contain stereo interleaved data - must convert to mono */
        short *audio_ptr;
        static short mono_buf[2880];  /* max for 60ms @ 48kHz */

#ifdef ENABLE_TEST_AUDIO
        if (audio_test_mode && test_audio_ptr != NULL) {
            /* Use recorded voice sample for audio test mode */
            for (unsigned x = 0; x < nsamp; x++) {
                mono_buf[x] = test_audio_ptr[audio_test_idx];
                audio_test_idx++;
                if (audio_test_idx >= test_audio_len)
                    audio_test_idx = 0;  /* loop */
            }
            audio_ptr = mono_buf;
        } else
#endif
        {
            /* Convert stereo interleaved to mono with decimation */
            short *in = half_ready ? (short*)audio_buffer_in : (short*)audio_buffer_in_B;
            for (unsigned x = 0; x < nsamp; x++) {
                int sum = 0;
                for (unsigned i = 0; i < navgs_; i++) {
                    if (micRightEn && micLeftEn) {
                        int v = *in++;
                        v += *in++;
                        sum += (v / 2);
                    } else if (micRightEn) {
                        sum += *in++;
                        in++;
                    } else if (micLeftEn) {
                        in++;
                        sum += *in++;
                    }
                }
                mono_buf[x] = sum / navgs_;
            }
            audio_ptr = mono_buf;
        }

        /* Encode audio to Opus - write directly to double-buffer slot */
        uint32_t start_tick = xTaskGetTickCount();
        encoded_len = opus_wrapper_encode(ow, audio_ptr, encoded_frame_buf[widx]);
        uint32_t enc_time = xTaskGetTickCount() - start_tick;

        if (encoded_len < 0) {
            printf("\e[31mENC_TASK: encode error %ld\e[0m\r\n", encoded_len);
            continue;
        }

        /* Store length and mark buffer as ready */
        encoded_frame_len[widx] = encoded_len;
        frame_ready[widx] = 1;
        write_idx = (widx + 1) % NUM_ENCODE_BUFS;  /* Advance to next buffer */

        /* Signal main that a frame is ready (semaphore for counting, notification for wake) */
        xSemaphoreGive(xFrameReadySemaphore);
        /* Direct task notification for instant wake */
        if (xMainTaskHandle != NULL) {
            xTaskNotify(xMainTaskHandle, EVENT_FRAME_READY, eSetBits);
        }

        if (enc_time > enc_time_max) enc_time_max = enc_time;
        enc_count++;

        encoder_state = "DONE";
    }
}

/*
 * TX task: receives encoded data, transmits via radio
 */
static void vTxTask(void *pvParameters)
{
    (void)pvParameters;
    uint8_t tx_frame[768];
    size_t received;

    for (;;) {
        /* Wait for enough data for one frame */
        received = xStreamBufferReceive(
            xTxStreamBuffer,
            tx_frame,
            _bytes_per_frame,
            pdMS_TO_TICKS(100)
        );

        if (received == 0) {
            /* Timeout - service radio */
            lorahal.service();
            continue;
        }

        if (received < _bytes_per_frame) {
            /* Partial frame - wait for more */
            size_t more = xStreamBufferReceive(
                xTxStreamBuffer,
                tx_frame + received,
                _bytes_per_frame - received,
                pdMS_TO_TICKS(50)
            );
            received += more;
        }

        /* Copy to TX buffer (with mutex protection) */
        if (xSemaphoreTake(xTxBufMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(&lorahal.tx_buf[tx_buf_idx], tx_frame, received);
            tx_buf_idx += received;
            xSemaphoreGive(xTxBufMutex);
        }

        /* Service radio - may trigger TX */
        lorahal.service();

        /* If TX is active, wait for TxDone before processing more */
        if (txing) {
            xSemaphoreTake(xTxDoneSemaphore, pdMS_TO_TICKS(100));
        }
    }
}

/*
 * Radio service task - highest priority to preempt Opus decode
 *
 * This task wakes on DIO8 IRQ and immediately calls lorahal.service()
 * to read the RX FIFO before it overflows. In voice mode, Opus decode
 * blocks for ~28ms, during which FIFO can fill up at 95 kbps.
 * By running at highest priority, this task preempts decode to service FIFO.
 */
static void vRadioServiceTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        /* Wait for DIO8 IRQ signal - block indefinitely */
        if (xSemaphoreTake(xRadioServiceSemaphore, portMAX_DELAY) == pdTRUE) {
            /* Service radio immediately - reads FIFO and processes events */
            lorahal.service();
        }
    }
}

#endif /* USE_FREERTOS */
