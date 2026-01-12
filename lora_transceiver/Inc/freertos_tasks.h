/*
 * FreeRTOS task definitions for concurrent Opus encoding and radio TX
 *
 * Architecture:
 * - Audio DMA callback signals encoder task via semaphore
 * - Encoder task: waits for audio, encodes to Opus, signals TX task
 * - TX task: waits for encoded data, transmits via radio, waits for TxDone
 * - This allows encoding to occur concurrently with radio transmission
 */

#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#ifdef USE_FREERTOS

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "stream_buffer.h"

/* Task priorities (higher number = higher priority)
 * Radio service is highest to preempt decode and prevent FIFO overflow.
 * Main handles TX_DONE and general processing.
 * Encoder runs when main blocks. */
#define RADIO_SERVICE_TASK_PRIORITY (configMAX_PRIORITIES - 1)  /* Highest: preempt decode for FIFO reads */
#define MAIN_TASK_PRIORITY          (configMAX_PRIORITIES - 2)  /* Below radio: TX_DONE response */
#define ENCODER_TASK_PRIORITY       (configMAX_PRIORITIES - 3)  /* Below main: runs when main blocks */
#define TX_TASK_PRIORITY            (configMAX_PRIORITIES - 4)  /* Below encoder */
#define AUDIO_SERVICE_TASK_PRIORITY (configMAX_PRIORITIES - 5)  /* Background audio out servicing */

/* Task stack sizes (in words, not bytes) */
#define MAIN_TASK_STACK_SIZE        (8192)  /* Main task needs large stack for AudioLoopback_demo */
#define ENCODER_TASK_STACK_SIZE     (8192)  /* Opus encoder needs large stack (silk DSP functions) */
#define TX_TASK_STACK_SIZE          (1024)
#define AUDIO_SERVICE_STACK_SIZE    (512)
#define RADIO_SERVICE_STACK_SIZE    (1024)  /* Radio service needs moderate stack for SPI ops */

/* Stream buffer for encoded data (bytes) */
#define TX_STREAM_BUFFER_SIZE       (1024)
#define TX_STREAM_TRIGGER_LEVEL     (160)   /* Trigger when one frame ready */

/* Semaphore handles - used by ISR callbacks */
extern SemaphoreHandle_t xAudioHalfSemaphore;
extern SemaphoreHandle_t xAudioFullSemaphore;
extern SemaphoreHandle_t xTxDoneSemaphore;
extern SemaphoreHandle_t xRadioServiceSemaphore;  /* Radio DIO8 IRQ signaling */

/* Task handles */
extern TaskHandle_t xEncoderTaskHandle;
extern TaskHandle_t xTxTaskHandle;
extern TaskHandle_t xRadioServiceTaskHandle;

/* Stream buffer for encoded data */
extern StreamBufferHandle_t xTxStreamBuffer;

/* Initialize FreeRTOS tasks and synchronization primitives */
void freertos_tasks_init(void);

/* Start FreeRTOS scheduler (call from main after hardware init) */
void freertos_start(void);

/* Signal functions - call from ISR with FromISR suffix */
void freertos_audio_half_ready_FromISR(void);
void freertos_audio_full_ready_FromISR(void);
void freertos_tx_done_FromISR(void);
void freertos_radio_irq_FromISR(void);  /* Signal radio service task from DIO8 IRQ */

/* Check if FreeRTOS tasks are active */
uint8_t freertos_tasks_active(void);

/* Get encoder task state for debugging */
const char* freertos_get_encoder_state(void);

/* Enable concurrent encoder/TX tasks (call after AudioLoopback_demo init completes) */
void freertos_enable_concurrent_encode(void);

/* Check if concurrent encoding is enabled */
uint8_t freertos_concurrent_encode_enabled(void);

/* Signal from main loop: TX session started */
void freertos_tx_session_start(void);

/* Signal from main loop: TX session ended */
void freertos_tx_session_end(void);

/* Get encoded frame from encoder task (returns length, 0 if not ready) */
int32_t freertos_get_encoded_frame(uint8_t *dest, uint16_t max_len);

#endif /* USE_FREERTOS */

#endif /* FREERTOS_TASKS_H */
