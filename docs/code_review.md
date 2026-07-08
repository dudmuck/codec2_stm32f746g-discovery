# Code Review — Opus over LoRa (freertos-opus branch)

Review date: 2026-07-07. Scope: application code — `lora_transceiver/`, `opus_wrapper/`,
`lr20xx_hal/`, `lr20xx/`, `sx12xx_hal/`, `vocoder_passthru/`, `syscalls.c`, linker script,
`CMakeLists.txt`, `test_radio_link.sh`. Third-party trees (`opus/`, `codec2/`, `STM32CubeF7/`)
were not reviewed.

Per request, timing-margin issues affecting the fastest Opus rate are collected in the
**last section** of this document; everything else comes first.

Severity: **HIGH** = likely memory corruption, hang, or wrong on-air data under normal use.
**MED** = real bug or fragile behavior needing specific (often plausible) conditions.
**LOW** = latent bug, dead code, or hygiene.

---

## 1. Memory corruption (highest priority)

### 1.1 HIGH — `sine_out()` / `tone_out()` overrun the speaker buffers by 4×
`lora_transceiver/Src/lora_xcvr.c:808` (and `tone_out()` ~line 886)

`audio_block_size` is in **bytes** (`nsamp * navgs_ * 4`, line 1396), but the fill loop
writes 4 bytes (two `short`s) per iteration for `audio_block_size` iterations:

```c
for (unsigned i = 0; i < audio_block_size; i++) {
    *outPtr++ = out;   /* 2 bytes */
    *outPtr++ = out;   /* 2 bytes */
    ...
}
```

That writes `4 * audio_block_size` bytes into a half-buffer of `audio_block_size` bytes —
up to 3 buffer-lengths past `audio_buffer_out_B` into adjacent SDRAM. The
`SCB_CleanDCache_by_Addr(bufStart, audio_block_size * 4)` on line 819 confirms the 4×
footprint. Correct loop bound is `nsamp * navgs_` (as `fill_audio_buffer()` uses, line 963).
This fires on every RSSI tone / squelch tail.

### 1.2 HIGH — `sine_out()` NULL-pointer write when play state is `AUDIO_PLAY_NONE`
`lora_transceiver/Src/lora_xcvr.c:800`

`silence()` and `tone_out()` spin-wait until `audio_play_state_ != AUDIO_PLAY_NONE`;
`sine_out()` does not. If the state is `AUDIO_PLAY_NONE` (reachable at stream start via
`start_tone()` after `streaming_rx_decode()` cleared the pending flags), `outPtr` stays
NULL and the loop writes through it — a write to address 0 (ITCM alias) that bus-faults
or silently corrupts. Add the same wait, or return early when state is NONE.

### 1.3 HIGH — no D-cache invalidation on DMA-filled microphone buffers
`lora_transceiver/Src/lora_xcvr.c:1761` (encode path), buffers set up ~1429

`main.c` enables the D-cache and there is **no MPU configuration** in the app tree. SAI RX
DMA fills `audio_buffer_in`/`audio_buffer_in_B` in SDRAM, and `decimated_encode()` reads
them directly with no `SCB_InvalidateDCache_by_Addr()` anywhere (output buffers do get
`SCB_CleanDCache_by_Addr`, inputs never get the inverse). The CPU can encode stale cached
samples, and the init-time `memset(audio_buffer_in, 0, ...)` leaves dirty lines that can be
evicted **on top of** fresh DMA data. Fix: invalidate the relevant half-buffer before each
encode, or configure the audio-buffer region non-cacheable / write-through via MPU. (If
audio currently works it is by luck of eviction patterns — this is the classic F7 DMA bug.)

### 1.4 MED — RX overflow `memset` can underflow and wipe ~64 KB
`lora_transceiver/Src/lora_xcvr.c:2052`

`overflow = saved_fifo_idx - pkt_size;` then `memset(&LR20xx_rx_buf[overflow], 0,
pkt_size - overflow);`. `fifoRxCB` allows `rx_fifo_read_idx` up to `lora_payload_length*2`
(`radio.c:94`), so for a short final packet `overflow` can exceed `pkt_size`, and
`pkt_size - overflow` underflows to ~65 K — a wild `memset` over RAM. Guard with
`if (overflow < pkt_size)` and clamp.

### 1.5 MED — `opus_wrapper_encode()` has no output-size limit and can write past `tx_buf`
`opus_wrapper/opus_wrapper.c:195`

The wrapper always tells Opus it may write `OPUS_WRAPPER_MAX_FRAME_BYTES` (768), but the
non-RTOS caller passes `&lorahal.tx_buf[tx_buf_idx]` having only checked headroom for
`_bytes_per_frame` (`lora_xcvr.c:1703`). The code's own "Encode LONG … TRUNCATED!" branch
(`lora_xcvr.c:730`) proves encodes can exceed `_bytes_per_frame` — but by then the write
has already happened; near the end of the 1536-byte buffer that is an overrun. Add a
`max_bytes` parameter (as `opus_encode()` itself has) and pass the real remaining capacity.

### 1.6 HIGH — `sx126x_step_bw()` reads/writes one past the end of `loraBWs[]`
`lora_transceiver/Src/app_sx126x.c:33`

`loraBWs` has 10 entries (0–9) but the step-up guard is `if (n < 10) n++`, so stepping up
from BW 500 (or a failed search, which also leaves `n == 10`) indexes `loraBWs[10]` and
programs a garbage bandwidth. Use `n < 9` and bail out when the search doesn't match.

### 1.7 HIGH — NULL string passed to LCD for SX127x SLEEP / unknown opmode
`lora_transceiver/Src/app_sx127x.c:17`

`modeStr` starts NULL; `case 0:` has `break;` **before** `modeStr = "SLEEP";` (dead
statement), and the `default:` case fills `str` but never assigns `modeStr = str`. Either
path reaches `BSP_LCD_DisplayStringAt(..., (uint8_t*)modeStr, ...)` with NULL → hard fault
whenever the SX127x is displayed in SLEEP or an unexpected mode. (`app_sx126x.c:118-120`
shows the correct pattern.) Same shape at `main.c:557`: the bitrate `switch` default leaves
`str = NULL` yet still `sprintf`s it.

---

## 2. Concurrency (FreeRTOS / ISR)

### 2.1 HIGH — no mutual exclusion around `lorahal.service()` / the SPI bus
`lora_transceiver/Src/freertos_tasks.c:613`, `lr20xx/lr20xx_hal.c:103`

`lorahal.service()` is called from the **highest-priority** `vRadioServiceTask`
(`freertos_tasks.c:613`), from the Main task (`lora_xcvr.c:1233, 1325, 1747, 1898`), and
from inside `freertos_get_encoded_frame` (`freertos_tasks.c:367`) — with no lock. The only
mutex in the system guards `tx_buf`, not the radio. `LR20xx_service()` is a long multi-step
sequence (read-and-clear IRQ status, FIFO reads, shared-state updates), and each SPI
transaction is a multi-byte NSS-framed sequence of per-byte `HAL_SPI_TransmitReceive`
calls. If RadioSvc preempts Main mid-transaction:

- NSS framing and the shared `stat`/`saved_command` globals in `lr20xx_hal.c` are corrupted;
- the read-and-clear IRQ status means one caller consumes events the other was handling;
- `rx_fifo_read_idx` read-modify-writes interleave.

**Fix (structural, do first):** either funnel *all* radio service through the RadioSvc task,
or add a radio mutex taken around every `lorahal.*` entry point. Several findings below
(2.2, 2.3) are windows into this same hole and shrink or vanish once access is serialized.

### 2.2 HIGH — `LR20xx_chipMode` set to TX only *after* the SetTx command; ISR gate races
`lr20xx_hal/radio_lr20xx.c:609` (also 550-554, and the restart at 99-100)

`Radio_dio8_top_half()` (line 153) decides whether to wake RadioSvc via
`if (LR20xx_chipMode != TX)`. All TX entry points issue `set_tx()` first and update
`chipMode` second, and `LR20xx_service` briefly parks chipMode in STBY_RC on TX_DONE
before the multi-packet restart re-arms TX. A FIFO_TX threshold IRQ landing in that window
sees a stale non-TX mode and wakes RadioSvc to run `LR20xx_service()` concurrently with
the task already inside the TX path (feeding 2.1). Set `chipMode = TX` **before** issuing
SetTx (revert on error). Also: `LR20xx_chipMode` is shared with the ISR but not declared
`volatile` (`lr20xx/lr20xx.c:14`).

### 2.3 HIGH — Main task rewrites the FIFO producer index `rx_fifo_read_idx` with no critical section
`lora_transceiver/Src/lora_xcvr.c:2011`

The single-packet path saves `rx_fifo_read_idx`, temporarily sets it to `pkt_size`, runs
`streaming_rx_decode()`, then restores it — while the higher-priority RadioSvc task's
`fifoRxCB` (`radio.c:119-120`) uses that same index as the FIFO **write** position and
increments it. Preemption in that window writes fresh RX bytes over not-yet-decoded data,
and the restore discards the callback's increment (lost bytes). Wrap in a critical section,
or better: don't mutate the producer's index — pass a decode limit into
`streaming_rx_decode()`.

### 2.4 HIGH — `freertos_get_encoded_frame()` permanently stalls on an oversize frame
`lora_transceiver/Src/freertos_tasks.c:342`

The counting semaphore is taken, but the slot is consumed only `if (len > 0 && len <=
max_len)`. Callers pass `max_len = _bytes_per_frame`, and the codebase's own non-RTOS path
acknowledges Opus CBR can exceed that ("Encode LONG … TRUNCATED!", `lora_xcvr.c:730`).
One oversize frame → semaphore count consumed, `frame_ready[read_idx]` stuck set, every
later take drains another count against the same stuck slot → encoder pipeline wedges in
"FULL" until reset. On the invalid case, still consume the slot (clear flag, advance
`read_idx`) and truncate or drop, mirroring the non-RTOS path.

### 2.5 MED — TX session restart races the in-flight encoder
`lora_transceiver/Src/freertos_tasks.c:290`

`freertos_tx_session_start()` resets `write_idx`/`read_idx` and drains
`xFrameReadySemaphore`, but `vEncoderTask` checks `tx_session_active` only at loop top and
an encode takes tens of ms. A stale encode completing after the reset publishes a frame for
the old index and gives the semaphore, desynchronizing count vs. ring for the whole session
(same stall mechanism as 2.4). Add a handshake (wait for encoder idle) or a session
generation counter the encoder re-checks before publishing.

### 2.6 MED — `audio_rec_buffer_state` written from DMA ISRs but not `volatile`
`lora_transceiver/Src/lora_xcvr.c:52` and `vocoder_passthru/Src/audio_rec.c:55`

Set in `BSP_AUDIO_IN_*_CallBack()` (ISR) and polled/RMW'd in task context in both
projects. Neighboring flags are all `volatile`; this one is an oversight. Today external
calls in the wait loops force reloads, but inlining/LTO or a tightened loop turns this
into an infinite loop. Declare `volatile` in both copies.

### 2.7 MED — `rx_size` handshake can lose a packet notification
`lora_transceiver/Src/main.c:75`

`rxDoneCB` (RadioSvc context) writes `rx_size = size`; the main loop tests it, spends
milliseconds decoding, and only then stores `-1` (`lora_xcvr.c:1964, 2066`). A second
packet arriving mid-decode has its `rx_size` clobbered by the late `-1`, skipping
end-of-packet handling for it. Read-and-clear atomically into a local at the top instead.

### 2.8 MED — LCD drawn from the highest-priority radio task
`lora_transceiver/Src/main.c:840`

`rxDoneCB` → `lcd_print_rssi_snr()` runs inside RadioSvc. BSP_LCD keeps global
font/color state also mutated by the Main task (`radio_screen`, `lcd_print_tx_duration`),
so draws can interleave/corrupt — and the slow LCD write occupies the radio task exactly
when it should be draining a 256-byte FIFO. Defer the display update to the main loop
(`rx_rssi`/`rx_snr` are already published for that).

### 2.9 MED — `pinEvent` bitfield RMW races between ISR and thread (SX127x)
`sx12xx_hal/radio_sx127x.c:7`

`dio0`/`dio1`/`txing` share one byte; bitfield writes are load-modify-store. The DIO0 ISR
setting `.dio0 = 1` can be overwritten by a concurrent thread-side `.txing = 1`
(`Send_sx127x`, line 151), dropping the event until the pin-level fallback polls it. Use
separate `volatile uint8_t` flags or bracket thread-side updates with IRQ disable.

### 2.10 MED — `_write()` UART ring buffer unsafe for multiple producers; can spin forever
`syscalls.c:150`

Linked into the FreeRTOS target where multiple tasks `printf`. `uart_tx_buf[head] = …;
tx_head = next_head;` is non-atomic, so concurrent writers corrupt/lose output. The
buffer-full path spins on `while (next_head == tx_tail)` waiting on the UART IRQ — a
printf from a critical section or masked-IRQ context hangs forever. Guard the producer
with a critical section and add a bounded-timeout/drop policy.

### 2.11 MED — UART RX re-armed into `rxchar` before the byte is consumed
`lora_transceiver/Src/lora_xcvr.c:409`, `lora_transceiver/Src/main.c:414`,
`vocoder_passthru/Src/audio_loopback.c:144`, `vocoder_passthru/Src/main.c:333`

All four sites do `HAL_UART_Receive_IT(&UartHandle, &rxchar, 1)` and *then* parse
`rxchar`. At 115200 the next byte lands ~87 µs later — well within the echo printf —
so back-to-back characters (scripts, pastes) corrupt the command byte. This directly
affects `test_radio_link.sh` reliability. Copy `rxchar` to a local before re-arming.

---

## 3. Radio driver correctness (LR20xx)

### 3.1 MED — stale packet-params cache between streaming and non-streaming sends
`lr20xx_hal/radio_lr20xx.c:585`

`Send_lr20xx_streaming` skips `set_packet_params` when `lora_payload_length ==
last_pkt_params_len` (function-local static), but `Send_lr20xx` (542-544) writes a
different length to the chip without touching that cache. Sequence
stream(240) → send(12) → stream(240) leaves the chip configured for 12-byte packets while
streaming 240. Invalidate the cache in `Send_lr20xx` / the packet-config functions, or
track the last value actually written to the chip.

### 3.2 MED — FIFO_TX and FIFO_RX branches double-call `get_and_clear_irq_flags`
`lr20xx/lr20xx.c:153`

Each branch calls `lr20xx_radio_fifo_get_and_clear_irq_flags()` and uses only its own
direction's flags; when both bits are set in one pass, the second call sees
already-cleared flags and the RX FIFO read is skipped → RX FIFO overflow at high rates.
Call it once per pass and dispatch both results.

### 3.3 MED — CRC/length-error RX drops the event without clearing the FIFO or calling `RxError`
`lr20xx/lr20xx.c:104`

On `RX_DONE` with CRC/LEN error (or oversize), nothing happens: corrupted bytes stay in
the hardware RX FIFO and misalign every subsequent packet (the next good read returns the
stale bytes first). `RadioEvents->RxError` exists but is never invoked anywhere. Clear the
RX FIFO on error and invoke `RxError` so the app can resync.

### 3.4 MED — multi-packet TX restart ignores errors and can transmit an empty FIFO
`lr20xx_hal/radio_lr20xx.c:97`

The continuation path ignores the returns of `fifo_clear_tx`, `Send_lr20xx_fifo_continue`
(0 when the encoder is behind or SPI fails), then starts a full-length packet regardless —
garbage on air with no state cleanup. Check the bytes-loaded return and defer/abort the
restart when it's 0.

### 3.5 MED — TX timeout unimplemented; lost TxDone wedges streaming forever
`lr20xx_hal/radio_lr20xx.c:133`

`Radio_timeout_callback` handles only RX (`// else TODO tx timeout`) and every
`set_tx(NULL, 0)` disables the hardware timeout. A missed TX_DONE leaves
`streaming_tx_active` / `stream_state` / `txing` set permanently. Pass a real TX timeout
and implement the TX branch (reset stream state, notify app), or add a software watchdog.

### 3.6 MED — `lr20xx_hal_read` ignores wait-on-busy failure before the response phase
`lr20xx/lr20xx_hal.c:133`

The inter-phase `lr20xx_hal_wait_on_busy()` result is discarded; on a stuck BUSY the code
clocks out garbage and returns `OK` — e.g. a bogus IRQ status or RX length that then feeds
buffer-fill logic. Check it and return `ERROR` like the pre-command check at line 109 does.

### 3.7 LOW — assorted latent driver issues
- `lr20xx/lr20xx.c:144` — RX size is `uint16_t` validated against 1536 but truncated to
  `uint8_t` in the `rxDone` callback signature; sizes 256–1535 alias mod 256. Latent while
  LoRa caps at 255; widen or clamp explicitly.
- `lr20xx_hal/radio_lr20xx.c:90` — multi-packet accounting assumes `tx_total_size` divides
  evenly by `lora_payload_length`; a short final packet would transmit FIFO garbage.
  Current configs divide evenly (480, 720 vs 240) — assert it at config time.
- `lr20xx_hal/radio_lr20xx.c:523` — `int8_t power_half_dbm = dbm * 2;` overflows before
  the clamps, so `dbm > 63` pins to −9.5 dBm instead of +22. Clamp in `int` first.
- `lr20xx_hal/radio_lr20xx.c:191` — two unbounded retry loops in `Init_lr20xx` hang boot
  forever if the radio is absent. Bound the retries; fail into a visible error state.
- `lora_transceiver/Src/radio.c:52` — `fifo_tx_underflow` / `fifo_tx_room` are written but
  never read (TX underflow — the on-air-corruption signal — is silently discarded), and
  `fifo_rx_disabled` is never referenced. Surface the underflow flag or delete them.
- `sx12xx_hal/radio_sx126x.c:192` — duplicated `else if (bwKHz > 11)` makes the
  `LORA_BW_10` branch unreachable (second test should be `> 8`).
- `sx12xx_hal/radio_sx126x.c:21` — `loraTimeoutSymbols` is never assigned; the symbol
  timeout write is always 0 (disabled). Wire it up or delete it.

---

## 4. Audio / Opus pipeline

### 4.1 MED — negative Opus encode result used as a frame length (vocoder_passthru)
`vocoder_passthru/Src/audio_loopback.c:244`

On encode error the negative `len` is stored in `encoded_len[]` and later handed to
`opus_wrapper_decode()`; stale samples then play. Store 0 instead — Opus treats len 0 as
packet loss and runs concealment.

### 4.2 MED — unsigned division corrupts negative samples when decimating
`lora_transceiver/Src/freertos_tasks.c:511` and `lora_xcvr.c:700`

`mono_buf[x] = sum / navgs_;` — `sum` is `int`, `navgs_` is `unsigned`, so the division is
unsigned; negative audio becomes garbage whenever `navgs_ > 1`. Latent today because the
demo always picks `audioRate == opus_sr` (`navgs_ == 1`), but the loop exists precisely to
support decimation. Fix: `sum / (int)navgs_`.

### 4.3 MED — concurrent encode path doesn't zero-pad short CBR frames
`lora_transceiver/Src/freertos_tasks.c:527`

The non-RTOS path pads short encodes to `_bytes_per_frame` (`lora_xcvr.c:724-729`); the
encoder task stores the raw length while the caller still advances `tx_buf_idx` by a full
`_bytes_per_frame` — the gap is stale bytes from a previous session, and the receiver
decodes fixed-size chunks. Pad in `vEncoderTask` to match.

### 4.4 MED — binary audio semaphores silently drop DMA buffers; asymmetric 10 ms wait
`lora_transceiver/Src/freertos_tasks.c:452`

A give on an already-given binary semaphore is a no-op: when the encoder stalls across a
DMA period, an entire frame vanishes with no counter (the non-RTOS path tracks
`dma_overrun_count`). The wait also blocks 10 ms on HALF before even checking FULL. A
2-deep queue of buffer IDs (drop-with-count when full) fixes both.

### 4.5 MED — `freertos_tx_done_FromISR()` is dead; the "instant TxDone wake" never happens
`lora_transceiver/Src/freertos_tasks.c:227`

Nothing calls it (`TxDone_topHalf` is NULL in `radio.c:135`), so `EVENT_TX_DONE`,
`xTxDoneSemaphore`, and the fast-service branch at 366-368 are dead — TxDone latency is
actually the 5 ms poll, contradicting the comments. Wire the radio TxDone top-half to it
(ISR priority checks out: DIO8 EXTI at 6 ≥ configMAX_SYSCALL 5) — this is also free
timing margin (see §7) — or delete the path.

### 4.6 MED — `vTxTask` and friends are dead code carrying real bugs
`lora_transceiver/Src/freertos_tasks.c:548`

The task is never created; `xTxStreamBuffer`, `xTxBufMutex`, `TX_TASK_STACK_SIZE`,
`TX_STREAM_*` are all unused. If ever enabled, it proceeds on partial stream reads and
`memcpy`s into `tx_buf` with no bounds check. Meanwhile the real producer never takes
`xTxBufMutex`. Delete the lot (and the orphaned `xEncodedFrameMutex`, created at line 87
and never taken — it misleads readers into thinking the ring is lock-protected).

### 4.7 LOW — opus_wrapper hygiene
`opus_wrapper/opus_wrapper.c`

- Line 89: `frame_ms == 2` is accepted (meaning 2.5 ms) but `rate * 2 / 1000` yields an
  invalid Opus frame size — creation succeeds, every encode then fails `OPUS_BAD_ARG`.
  Special-case 2.5 ms (`rate / 400`) or drop 2 from the list.
- Lines 143-159: all six `opus_encoder_ctl()` results ignored. `OPUS_SET_VBR(0)` is
  load-bearing (the protocol assumes fixed-size frames, no length field) — check at least
  VBR and BITRATE and fail creation on error.
- `printf` used without `#include <stdio.h>` (implicit declaration; hard error in C23).
- `freertos_tasks.c:82` — buffers hardcode `768` / `2880` instead of
  `OPUS_WRAPPER_MAX_FRAME_BYTES` / `OPUS_WRAPPER_SAMPLES_PER_FRAME_MAX` (header already
  included).

### 4.8 LOW — misc app correctness
- `lora_xcvr.c:816` — sine table wrap uses `>` instead of `>=`: `table_idx == 1024` reads
  one past the table. And `start_tone()` (825) can pass a negative float to the `unsigned`
  parameter — UB on the conversion for weak signals (RSSI −125/SNR −15).
- `lora_xcvr.c:1191` — `HAL_IncTick` compares `uwTick == terminate_spkr_at_tick` with 0 as
  the disabled sentinel: fires spuriously at the 49.7-day wrap; use a flag plus
  wrap-safe signed comparison.
- `lora_xcvr.c:1197` — `delay_ticks()` computes `dest = uwTick + t` (not wrap-safe); use
  `while ((uwTick - start) < t)`.
- `lora_xcvr.c:1090` — decode loops assume `rx_size` is a multiple of `_bytes_per_frame`;
  a partial tail decodes trailing garbage. Clamp or skip the last partial chunk.
- `lora_xcvr.c:1474` — touch change-detect compares **array pointers** (always unequal)
  and has an X-vs-Y typo: `this_TS_State.touchX != prev_TS_State.touchY || …touchY !=
  …touchY`. Net effect: `radio_screen()` redraws (plus two `lorahal.service()` calls)
  every poll regardless of touch. Compare element values.
- `main.c:806` — `lcd_print_tx_duration(0, cycleDur)` divides by a possibly-zero
  interval; `(unsigned)(inf * 100)` is UB. Guard `interval == 0`.

---

## 5. Build, link, and scripts

### 5.1 HIGH — `CMakeLists.txt` fails to configure the default radio
`CMakeLists.txt:221`

```cmake
if(${RADIO} STREQUAL SX12XX)
    message(SEND_ERROR "radio is sx12")
```

`RADIO` defaults to `SX12XX` (line 23), and `SEND_ERROR` makes CMake skip generation — a
plain `cmake ..` per the README fails. Leftover debug; change to `message(STATUS ...)`.

### 5.2 MED — `cmake_minimum_required(VERSION 2.8)` after `project()`
`CMakeLists.txt:21`

Must precede `project()` to take effect, and CMake ≥ 3.27 refuses compatibility < 3.5
outright — the file will stop configuring on current toolchains. Move to line 1, bump to
≥ 3.13.

### 5.3 MED — `test_radio_link.sh` always exits 0 even on failures
`test_radio_link.sh:83`

The `cleanup` EXIT trap ends with a bare `exit`, whose status is the last command in the
trap (`exec 3>&- 4>&-`, closing FDs never opened → 0) — so `exit $FAIL_COUNT` (line 301)
is clobbered and CI sees success. Also: `kill $TX_PID $RX_PID` runs with empty vars on
early exits, and the duration argument is never validated as numeric (`sleep abc` errors
mid-test with boards already transmitting). Fix the trap (`trap - EXIT` inside cleanup,
preserve `$?`), guard the kills, validate args.

### 5.4 LOW — syscalls.c / linker script hygiene
- `syscalls.c:42` — dead `#define FreeRTOS` (the real guard is `USE_FREERTOS`), duplicated
  `stack_ptr` declaration, stale "Reserve 16KB" comment.
- `stm32f746_flash.ld:36` — comment says "end of 1024K RAM" on a 320 KB part. More
  importantly: under `USE_FREERTOS`, syscalls.c reuses `_Min_Heap_Size` (0xC000 = 48 KB)
  as the *runtime* malloc ceiling — a non-obvious coupling worth a comment, and worth
  checking against the largest Opus encoder+decoder state actually allocated.

### 5.5 Repo hygiene
- `.gitignore` covers only `build/` and `*.swp`; the working tree is littered with
  untracked scratch files (`foo.txt`, `dma.txt`, `rx.log`, `tx.log`, `tx_slow.txt`,
  `freertos.txt`, `frame_duration.txt`, `vbr.md`, `test_logs/`) and an untracked
  `codec2/` checkout. Extend `.gitignore` (logs, `*.txt` scratch) and either make
  `codec2/` a submodule like `opus`/`STM32CubeF7` or remove it.
- `lora_transceiver/Inc/test_audio_*.h` (~470 KB of generated sample data) are untracked —
  decide whether they're build inputs (commit or generate at build time) or scratch.

---

## 6. Duplication / structure

- **Three diverged copies of the same sources.** `passthru/`, `vocoder_passthru/`, and
  `lora_transceiver/` each carry their own `stm32f7xx_it.c` (passthru's two are
  byte-identical, lora_transceiver's has diverged), `audio_rec.c` (~3-line diff),
  `system_stm32f7xx.c`, etc. Bug fixes (e.g. the missing-`volatile` in §2.6, or the
  `DMA2D_IRQHandler` that traps in `for(;;)` — a silent hard-hang if that IRQ ever fires)
  must be applied N times and drift. Hoist shared IRQ handlers and audio callbacks into a
  common directory, as `sx12xx_hal/` already does for radio code.
- **Triplicated LCD opmode printers.** `lcd_print_sx127x_opmode` / `…sx126x…` /
  `…lr20xx…` are near-identical; a table-driven shared implementation would also have
  prevented the NULL-string bug (§1.7).
- **Copy-pasted DIO-IRQ setup** in `radio_sx126x.c` (`Send_` 57-72 vs `Rx_` 316-332)
  differing by one bit — extract `set_dio1_irq(mask)`.
- **`#if 0` mbed leftovers** (listen-before-talk blocks referencing symbols that don't
  exist here) in both `radio_sx126x.c` and `radio_sx127x.c` — delete.
- **`AudioLoopback_demo` is ~730 lines** mixing a 5-state stream machine, FreeRTOS and
  bare-metal paths (three-deep `#ifdef` nesting), a `goto`, and ~10 inline
  `static …_debug_cnt` printf blocks (`lora_xcvr.c:1370`). Most concurrency findings in
  §2 live in this function; extracting the TX state machine into `tx_stream_service()`
  and removing debug scaffolding would make them fixable and reviewable.
- Dead code: `vocoder_passthru/Src/audio_rec.c:66` — `AudioRec_demo()` has no caller,
  references undefined symbols (links only via `--gc-sections`), carries a 128 KB static
  buffer (~40 % of SRAM), and contains a latent D-cache/DMA bug. Delete all but the three
  live `BSP_AUDIO_IN_*_CallBack()` implementations. Also `main.c:273-322` (`#if 0` UART
  DMA init) and the stale comment at `main.c:358` claiming audio IRQ priority 5 (BSP uses
  0x0E/0x0F).

---

## 7. Timing margin (fastest Opus rate) — deferred, per request

Collected here because these all consume the inter-packet/inter-frame budget the fastest
rate is already short on. Roughly in order of leverage:

1. **Per-byte SPI** (`lora_transceiver/Src/radio.c:155`): every radio byte is a full
   `HAL_SPI_TransmitReceive(…, 1, 100)` call — a 240-byte FIFO write is 240 HAL
   invocations whose overhead dwarfs the 13.5 MHz wire time, and it happens inside the
   TxDone→next-packet gap. Single burst transfer (or DMA) per NSS frame is the biggest
   single lever. (Also: `in_data` is uninitialized on HAL error and gets parsed as chip
   status, and the per-byte error printfs block on UART in the hot path.)
2. **Blocking printfs in the hot paths** (`lora_xcvr.c:1694` TX_SLOW inside the encode
   cycle; DROP/DUP/CORRUPT at 615-645 in RX decode; the underflow printf in
   `Radio_txDoneBottom`, `radio_lr20xx.c:110`): each blocks ~0.1 ms/char at 115200 —
   several ms out of an already-blown budget, cascading further overruns. Rate-limit to
   per-second summaries or move to a non-blocking UART path (which §2.10's fix enables).
3. **TxDone wake latency**: wiring `freertos_tx_done_FromISR()` (§4.5) replaces the 5 ms
   poll with an immediate wake — free margin on every packet boundary.
4. **Busy-waits that don't yield** (`lora_xcvr.c:1009`): `put_spkr()`/`silence()` spin on
   `asm("nop")` for up to a full audio half-period inside `streaming_rx_decode()`,
   starving lower-priority work (and in the bare-metal build, not draining the RX FIFO at
   all). Replace with a semaphore/notification wait or a bounded wait that services the
   radio.
5. **`radio_screen()` redrawing every poll** due to the pointer-compare bug (§4.8,
   `lora_xcvr.c:1474`) — fixing the comparison removes a heavy LCD redraw plus two
   `lorahal.service()` calls per poll interval from the loop.

Suggested overall fix order: §1.1–1.3 and §5.1 (corruption + build break) → §2.1/2.2/2.3
(serialize radio access; many other windows close with it) → §2.4/2.5 (pipeline stalls) →
the rest of §2–§5 opportunistically → §7 as its own focused pass.
