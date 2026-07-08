# FHSS Implementation Review — `lr20xx_fhss` branch

Review date: 2026-07-08. Scope: `lr20xx_hal/fhss.c` / `fhss.h` (core module) and the
integration diff vs master (`lora_xcvr.c`, `main.c`, `radio.c`, `radio_lr20xx.c`,
`lr20xx.c`, `lr20xx_hal.c`). Build configuration reviewed: no FreeRTOS on this branch —
all radio service and FHSS handlers run in main-loop context (the DIO8 top-half only
records a tick), so `fhss_cfg` is single-context today.

Contents:
1. [Why the slow codec rates fail](#1-why-the-slow-codec-rates-fail-13001200700c) (root cause)
2. [High-severity findings](#2-high-severity-findings)
3. [FCC Part 15.247 compliance risks](#3-fcc-part-15247-compliance-risks)
4. [State machine & robustness](#4-state-machine--robustness)
5. [Timing & hot-path issues](#5-timing--hot-path-issues)
6. [Code quality / dead code](#6-code-quality--dead-code)
7. [Synchronization method: assessment and alternatives](#7-synchronization-method-assessment-and-alternatives)

---

## 1. Why the slow codec rates fail (1300/1200/700C)

Two interacting timing bugs explain the commit note
"working: 3200, 2400, 1600, 1400 — not working: 1300, 1200, 700C":

### 1.1 HIGH — `codec2_production_time` is 2× too small for dual-frame modes
`lr20xx_hal/radio_lr20xx.c:917` (and again ~1102-1112 feeding `inter_pkt_timeout` and
`fhss_set_proactive_hop_count`)

```c
codec2_production_time = frames_per_packet * codec2_frame_period_ms;
```

assumes one `_bytes_per_frame` per 40 ms audio callback. But CODEC2 1300 and 700C
produce one `_bytes_per_frame` per **two** callbacks (the `mid` toggle in
`lora_xcvr.c:1405-1440`), so real production time is double the computed value. The
device's own log confirms it: `test_logs/fhss_rx_1300_...log` prints "Codec2 production
time for packet: 80 ms" when the true interval is 160 ms. For 700C the computed 280 ms
is actually 560 ms — longer than both the 400 ms dwell and the 320 ms RX hop timeout, so
RX times out three times and rescans before the first data packet even arrives.
**Fix:** multiply by 2 for dual-frame modes everywhere production time is computed.

### 1.2 HIGH — RX hops on a fixed 320 ms timer while TX hops on packet count
`lora_transceiver/Src/lora_xcvr.c:1619` (and 1667)

RX arms `terminate_spkr_at_tick = uwTick + FHSS_MAX_DWELL_MS*8/10` (320 ms), modeling
"TX hops at 90% of dwell." TX actually hops when `pkts_on_channel >=
proactive_hop_count` (`fhss.c:1042-1076`), i.e. after `count × production_interval` —
~360 ms for the fast modes (close enough to 320 ms to work) but ~240 ms for 1200 and
~560 ms (real) for 700C. So RX either hops long after TX (permanently one channel
behind) or long before TX's next packet lands. **Fix:** derive the RX hop timeout from
`proactive_hop_count × actual production interval` (using the corrected §1.1 value),
not a fixed fraction of the dwell limit.

These two fixes should be made together; §1.1 alone corrects the numbers that §1.2's
formula needs.

---

## 2. High-severity findings

### 2.1 TX stalls forever in `TX_WAIT_ACK` on a corrupted ACK
`lr20xx_hal/radio_lr20xx.c:106`

When `fhss_rx_ack_packet()` rejects a received packet, the code prints "invalid ACK
received" and returns, commenting "timeout will retry" — but the RX just *completed*
with RxDone, so no RxTimeout will ever fire, and the LR20xx auto-TX/RX feature
self-disables after triggering once. Nothing polls `ack_wait_start_ms`. The state
machine sits in `TX_WAIT_ACK` until PTT release. **Fix:** on invalid ACK, restart RX
with the remaining ACK window, or call `fhss_ack_timeout_handler()` directly.

### 2.2 Default build (`ENABLE_HOPPING=OFF`) no longer compiles
`lora_transceiver/Src/main.c:708`, `lr20xx_hal/radio_lr20xx.c:1112`,
`lora_transceiver/Src/app_lr20xx.c:31`

`ENABLE_HOPPING` defaults OFF (`CMakeLists.txt:24`) and `fhss.c` is only compiled when
ON, yet three references escaped the guards: `lcd_print_rssi_snr()` uses
`fhss_cfg.current_channel` while its `#include "fhss.h"` is `#ifdef`-guarded;
`radio_lr20xx.c:1112` calls `fhss_set_proactive_hop_count()` outside any guard;
`app_lr20xx.c:31` references `fhss_cfg`. Wrap all three in `#ifdef ENABLE_HOPPING`.

### 2.3 `txing` stuck and speech tail dropped on PTT release
`lora_transceiver/Src/lora_xcvr.c:560`

`tx_encoded()` sets `txing = 1` *before* the FHSS-ready check, and when FHSS is enabled
but not data-ready it returns without transmitting — so no TxDone ever clears `txing`.
This path is reached in normal operation: the unkey handler calls `fhss_tx_stop()`
(state → RX_SCAN) and *then* `tx_encoded(tx_buf_idx)` for the buffered tail. Result:
the final partial packet is silently dropped, `txing` blocks the main-loop TX gate, and
`rx_start_at_tx_done` is consumed without either scan or RX actually starting. **Fix:**
set `txing` only after committing to a send, and don't flush leftover data after
`fhss_tx_stop()` (or flush it *before* stopping).

### 2.4 After 10 failed ACK attempts, TX goes IDLE and silently eats all audio
`lr20xx_hal/fhss.c:889` + `lora_xcvr.c:572-581`

`fhss_ack_timeout_handler()` drops to IDLE with no notification; `tx_encoded()` then
discards every frame while PTT is held — no retry of `fhss_start_tx_sync()`, no user
feedback. Restart the sync sequence after a back-off, or surface the failure (tone /
LCD) so the operator re-keys.

---

## 3. FCC Part 15.247 compliance risks

### 3.1 HIGH — sync preamble + sync packet exceeds 400 ms on one channel at SF8+
`lr20xx_hal/fhss.c:1915`

`fhss_calc_sync_preamble_len()` caps the preamble at 350 ms leaving a fixed 50 ms of
headroom for the sync packet — but the sync packet is ~30 symbols: ~31 ms at SF7 (fits,
381 ms total), ~61 ms at SF8 (411 ms — over), ~123 ms at SF9 (473 ms). Compute the
headroom from the actual sync-packet time-on-air for the current SF/BW.

### 3.2 HIGH — no 20-second-window occupancy accounting
`lr20xx_hal/fhss.c:1069`

15.247(a)(1)(i) limits average occupancy of any frequency to 0.4 s **within a 20 s
window**. The code enforces only per-visit TX time (packet-count proxy for 400 ms).
During continuous voice, TX makes ~20-40 hops per 20 s over just 50 channels — random
revisits inside the window are near-certain, and two ~350 ms visits to the same channel
put ~700 ms on it in 20 s. Add sliding-window per-channel TX-time tracking and skip (or
delay) a draw that would exceed the budget. (Related: `fhss_set_proactive_hop_count()`
divides by `pkt_toa_ms` with no zero guard.)

### 3.3 MED — sync retry channels are deterministic, not pseudorandom
`lr20xx_hal/fhss.c:903` and `:1611`

Retries use `(tx_next_channel + attempt*17) % 50`, each carrying a ~380 ms burst, up to
10 per PTT press — correlated, non-random channel use that weakens the "each frequency
used equally on average" property, and a retry can land back on the initial sync channel.
Draw retry channels from a separate LFSR sequence (RX doesn't need to track it).

### 3.4 LOW — modulo-50 bias
`lr20xx_hal/fhss.c:211`

`lfsr % 50` over the 65535-state period gives residues 1-35 one extra hit (~0.08 % bias)
— almost certainly within tolerance, trivially removed with rejection sampling (discard
draws ≥ 65500). The LFSR itself is a proper maximal-length polynomial with zero-state
guards in both seed paths.

---

## 4. State machine & robustness

- **MED — `RX_SEND_ACK` has no timeout or error path** (`fhss.c:803`). Only
  `fhss_tx_done_handler()` exits it; a failed `lorahal.send()` or lost TxDone strands
  the receiver — `fhss_poll()` returns immediately for this state and no timeout
  handler covers it. Add a tick-based watchdog falling back to `fhss_start_scan()`.
- **MED — sync packets with `hop_count != 0` are rejected** (`fhss.c:701`), making the
  fast-forward code below (731-744) unreachable and disabling late join — yet the
  non-ACK repeat path (`fhss_send_sync_repeat()`, line 1620) *does* send nonzero
  hop_count, so with ACK mode disabled a legitimate repeat sync gets rejected. Either
  restrict the rejection to ACK mode or remove the dead machinery (see also §7,
  option D, for making late join real).
- **MED — channel-mismatch LFSR catch-up can permanently desync** (`fhss.c:1270`). On
  header/channel mismatch RX advances its LFSR up to 5 draws hunting for the header's
  channel; if none matches it *continues anyway* with the LFSR 5 draws off, and the
  next proactive hop lands wrong. The search also stops at the first matching draw,
  which can be a lagging intermediate hop. An authoritative sync field in the data
  header (§7 option D) removes the guesswork.
- **MED — seq-wrap rescan** (`fhss.c:1313`). `expected_seq == 0` is overloaded to mean
  "first packet of session," but the uint8 sequence wraps every 256 packets (~26-45 s of
  voice); a stale duplicate at the wrap boundary triggers a full `fhss_start_scan()`
  mid-stream. Use a `first_packet_received` flag. Also `seq_diff == -128` falls through
  both branches and is silently accepted.
- **MED — RX still accepts 0xAA "re-sync" packets in data mode** (`fhss.c:1177`) and
  re-seeds its LFSR from them, though TX can no longer send them (`#if 0` at 1112).
  Validation passes ~1/128 of random CRC-valid contents — a foreign LoRa packet with the
  default sync word can hijack the hop sequence. Delete the branch (or gate it with the
  TX side).
- **MED — hardcoded −80 dBm noise filter** (`fhss.c:1554`) classifies CRC-errored
  packets below −80 dBm as noise, so at real range (where LoRa decodes far below −80)
  errors stop counting toward hop tracking exactly when the link is marginal — and it
  contradicts the −100 dBm sync threshold in `radio_lr20xx.c:118`. Derive the threshold
  from recently measured good-packet RSSI.
- **MED — the ACK's image-frequency check cannot work as wired** (`fhss.c:867`). The
  ACK carries `rx_sync_channel` copied from the *sync payload* (line 753), not the
  channel RX physically detected on, so TX's comparison only fails if the payload itself
  was corrupted; and on mismatch the code accepts anyway ("For now, accept anyway but
  log"). Send the physically-detected channel and reject/retry on mismatch — this is
  the exact failure mode (image reception) the docs blame for desyncs.
- **MED — startup CAD/RX collision** (`lora_xcvr.c:1219`). `main.c:524` starts the CAD
  scan, then `AudioLoopback_demo` unconditionally calls `lora_rx_begin()`, aborting the
  in-flight CAD while the state stays `RX_SCAN`; recovery depends on the aborted CAD
  still latching CAD_DONE. Guard with `if (!fhss_cfg.enabled || !fhss_is_scanning())`.

---

## 5. Timing & hot-path issues

- **MED — `wait_on_busy` timeout cut from ~40 ms to a miscalibrated counted spin**
  (`lr20xx/lr20xx_hal.c:71`). The new `++cnt > 100000` bound is a few ms at most (the
  comment claims ~500 µs), far below the old 40 ms — long-BUSY operations (the new
  front-end calibration at init, standby/oscillator transitions) make the *next*
  command print "wait_on_busy fail" and proceed with a corrupt SPI exchange. This is
  shared code; non-FHSS builds inherit the risk. Use a `HAL_GetTick()` delta (5-10 ms).
- **MED — float printf in the RX-error path before RX restarts** (`lr20xx/lr20xx.c:118`).
  Noise-triggered CRC errors are routine in implicit-header FHSS mode, and the code
  elsewhere documents that a few ms of printf before RX restart loses the 8-symbol data
  preamble. Move the print after the `rxError` callback (mirroring the RxDone fix at
  `radio_lr20xx.c:150-165`). It also fires in non-FHSS builds where errors were silent.
- **MED — per-packet "E" printf + LCD draws inside the inter-packet budget**
  (`lora_xcvr.c:551, 577`; `radio.c:30-32`). For rates where TOA ≈ production interval
  the whole margin is 5-10 ms; one Font24 LCD draw can exceed that. The "E" print is
  also a regression leaked into the non-FHSS streaming path. Gate to every Nth packet
  in FHSS data mode.
- **MED — dead "periodic hop check" spams printf in the RX decode loop**
  (`lora_xcvr.c:1536`). The block only runs in `RX_DATA`, where `fhss_check_hop()`
  no-ops (it handles TX_DATA only) — so the claimed protection doesn't exist — but
  `fhss_check_hop()` prints `check_hop#…` every 10th call *before* its state check,
  every main-loop iteration. Delete the block or implement RX_DATA hopping for real.
- **MED — preamble math is optimistic and the CAD sweep model breaks at SF8+**
  (`fhss.c:1899`). The per-channel cost model (`cad_symbols×T_sym + 1 ms`) ignores
  standby/SetRfFrequency/SetCad SPI and poll latency (the code even *measures* sweep
  time at line 562 but never feeds it back). At SF8 the computed 764 ms preamble is
  FCC-capped to 350 ms against a 460 ms modeled sweep — one RX sweep is no longer
  guaranteed to intersect the preamble, so sync leans entirely on retries. Also the
  radio's preamble-search timeout is hardcoded to 343 symbols (`fhss_configure_cad()`,
  line 361) — correct only for SF7; derive it from `preamble_len_symb`.
- **LOW — PTT lag up to ~500 ms during scan** (`lora_xcvr.c:1236`): `fhss_poll()` can
  loop in RX_SYNC until its 500 ms safety timeout while PTT is only sampled between
  polls. Check a PTT-pending flag inside the poll loop.
- **LOW — wrap-unsafe tick comparisons** (`fhss.c:835, 2120, 2140`): use the
  `now - start > limit` form as `wait_for_tx_complete()` already does.

---

## 6. Code quality / dead code

- Dead/vestigial (`fhss.c`): `fhss_calc_sync_packet_timeout_ms()` (never called);
  `rx_first_dwell` / `FHSS_FIRST_DWELL_PKT_COUNT` (init-only); the whole `#if 0`
  re-sync machinery (`fhss_send_resync_packet`, `tx_resync_pending`,
  `FHSS_RESYNC_HOP_INTERVAL`); the deferred-packet buffer (`deferred_pkt_buf` is never
  written, and its send path at line 1124 would double-send over an in-flight TX if it
  ever ran); `stats.sync_time_ms` prints a raw timestamp, never a duration; TX-side
  `dwell_start_ms` unused (hops are packet-count based); stale comments (85 %/340 ms,
  "sync_time + 2000ms") that no longer match the code.
- The "configure ACK packet format" call in `fhss_start_tx_sync()` /
  `fhss_ack_timeout_handler()` is a no-op — immediately overwritten by the sync config
  before send; it works only because explicit-header mode carries length in the air.
  Remove or reorder so the comment matches reality.
- Bad-marker debug dump reads `data[0..7]` with only `size >= 3` guaranteed
  (`fhss.c:1244`).
- Magic numbers with unenforced mutual consistency: 17 (retry stride), 1500 ms
  (post-sync wait), 320 ms (RX hop), −80 dBm, `tries < 5`, `hop_count <= 100`,
  343 symbols. Several of §1's bugs are exactly these constants disagreeing with each
  other; centralize them and derive where possible.
- `main.c:445` doubles the 3200-mode payload (24→48 B) for all builds, silently changing
  the non-FHSS 500 kHz tuning that chose 24 B so TOA < production time. Make it
  conditional on `ENABLE_HOPPING`.
- Cross-context safety is *accidental*: everything works because this branch services
  the radio only from the main loop. `fhss_cfg` has no volatile/locking; re-enabling
  `TxDone_topHalf` or merging with the freertos-opus branch makes the CAD-done vs.
  state-write pairs immediately racy. At minimum document the single-context invariant;
  better, route stop requests through a flag consumed inside service.

---

## 7. Synchronization method: assessment and alternatives

### What you have

TX: long preamble (FCC-capped ~350 ms) + sync packet carrying the full 16-bit LFSR
state on a random channel; RX: 4-symbol CAD sweep across 50 channels with two-stage
verify timeouts; optional ACK handshake with deterministic-channel retries.

Measured/computed budget at the tuned SF7/BW125 config: CAD sweep ≈ 255 ms (modeled,
optimistically), preamble 342 symbols ≈ 350 ms, sync packet ≈ 31 ms. Typical
acquisition ≈ 0.4-0.6 s; worst case in ACK mode ≈ 10 × (381 ms + 200 ms) ≈ 5.8 s, then
give-up-to-IDLE (§2.4). This cost is paid **on every PTT press**.

The scheme is sound and standard (it's essentially how many scanning FHSS radios
acquire), and shipping the LFSR state in-band is a robust primitive. Its structural
weaknesses: (a) acquisition cost on every key-up, (b) CAD false-positive sensitivity
(hence the two-stage timeouts and preamble margin arms race), (c) the preamble must
cover a whole sweep, which collides with the 400 ms dwell cap at SF8+ (§3.1, §5), and
(d) once sync is lost mid-stream the only recovery is a full rescan.

### Alternatives, in rough order of value

**A. Persistent net sync (background hopping + keepalive beacons) — recommended.**
Sync once, then never re-acquire: both radios keep hopping the shared LFSR sequence
even when idle, TX sends a tiny keepalive beacon every few seconds, and PTT starts
voice *instantly* on the current channel. Crystal budget: ±20 ppm each side → 40 µs/s
relative drift; between 5 s beacons that's 0.2 ms of skew against dwells of 240-400 ms —
three orders of magnitude of margin, so even 60 s beacon intervals hold easily. The
long-preamble CAD scan remains as the *initial/net-entry* mechanism (and after long
outages), which is exactly where its cost is acceptable. This converts the per-PTT
0.5-6 s acquisition into a one-time cost and removes the §2.4 give-up failure mode
from the talk path. Cost: idle keepalive TX (negligible duty cycle, still hopping —
FCC-friendly), plus an idle-RX current draw you already pay while scanning.

**B. Authoritative sync in every data header (self-synchronizing stream).**
The data header already carries `channel`; add the low byte of the LFSR state (or a
hop-sequence counter). Any single correctly-received data packet then fully re-syncs a
lost receiver — no LFSR guessing (§4 catch-up bug), no rescan on seq-wrap, and true
late join / third-listener support falls out for free. Cost: +1-2 bytes per packet
(~1 % at 49-72 B payloads). This is the highest robustness-per-byte change available
and pairs with A (a rejoining radio just parks on any channel and waits ≤ 50 hops, or
CAD-scans, for one data packet).

**C. Invert the search: TX sweeps, RX parks.**
Instead of RX sweeping 50 channels for one long preamble, RX parks on one
LFSR-agreed channel while TX transmits short sync packets hopping across channels;
TX is guaranteed to land on RX's channel within 50 short packets. Worst case
≈ 50 × (sync TOA + turnaround) ≈ 50 × 25 ms ≈ 1.25 s, average ~0.6 s — comparable to
the current typical case, but with **no CAD at all**: no false positives, no two-stage
verify heuristics, no preamble-vs-dwell tension (each sync burst is ~30 ms, trivially
FCC-clean), and the per-attempt ACK comes back on the same channel. If you keep the
current scheme, this is the fallback to consider whenever CAD tuning gets fragile at
other SF/BW combinations.

**D. Rendezvous-channel subset.**
Derive k (e.g. 3) calling channels from the shared seed; RX rotates slowly among them,
TX syncs there with a short preamble. Acquisition ~100-200 ms. Downside: those channels
carry disproportionate sync traffic (the §3.3 concern, amplified) and jamming/occupancy
of a calling channel delays sync; mitigations (rotating the subset per wall-clock epoch)
need loosely-synced clocks. Fine for a two-radio experiment; weaker as a design
principle than A+B.

**E. Hardware LR-FHSS — not viable here.**
The LR2021 driver in-tree exposes LR-FHSS TX only (`lr20xx_radio_lr_fhss.h` has
`build_frame`/`set_sync_word`, no RX path), and LR-FHSS physical payload rates
(~162-325 bps) are below even 700C's 700 bps before headers. It solves regulatory
hopping for telemetry uplinks, not full-duplex-ish voice. Correct call to build a
custom scheme.

### Suggested plan

1. Fix the slow-rate root cause (§1.1 + §1.2) — mechanical, unblocks 1300/1200/700C.
2. Fix the stall/stuck-state bugs (§2.1-2.4) and the FCC items (§3.1, §3.2) — the
   current design is then solid for what it is.
3. Add **B** (LFSR byte in the data header) — small, removes the whole
   guess-and-rescan class of failures and enables late join.
4. Consider **A** (persistent net sync) as the next feature: it changes PTT latency
   from ~0.5-6 s to ~0, which for a voice radio is the difference users actually feel.
