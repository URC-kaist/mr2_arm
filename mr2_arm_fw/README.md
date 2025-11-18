# MR2 Arm Firmware

Firmware targeting the STM32H523 MCU that supervises two AS5600 magnetic encoders, dual-channel limit switches on GPIOA4–GPIOA7, and broadcasts system state over classic CAN (FDCAN1). CubeMX still owns the peripheral scaffolding in `Core/Src/main.c`, while the application logic now lives in:
- `Core/Src/app_encoder.c` — encoder sampling, I2C fault handling, and encoder CAN framing.
- `Core/Src/app_limit_switch.c` — limit switch polling, fault detection, and CAN framing.
- `Core/Src/app_can.c` — shared CAN helpers and bus-off recovery orchestration.

## High-Level Flow
- System clock configured to run from the CSI oscillator via PLL1 (SYSCLK ≈ 250 MHz).
- GPIO, I2C1/I2C2, ICACHE, FDCAN1, USART2, and TIM1 peripherals are initialized.
- FDCAN1 is started in normal classic-CAN mode and ready to transmit.
- TIM1 runs as a 10 Hz scheduler (prescaler 24 999, period 999). Its update interrupt calls `AppEncoder_OnSchedulerTick()` to request a fresh encoder sample.
- TIM2 runs as a 500 Hz poller for the limit switches and invokes `AppLimitSwitch_Poll()` every tick.
- The main loop flushes queued limit-switch frames via `AppLimitSwitch_Service()`, performs encoder work via `AppEncoder_Service()`, services CAN bus-off recovery, and idles via `__WFI()` once nothing is pending.

## Sensors and Peripherals

### AS5600 Encoders (I2C1 / I2C2)
- Each bus targets an AS5600 at 0x36 (7-bit address). Reads use `HAL_I2C_Mem_Read` from register `0x0E` (`ANGLE`).
- `AppEncoder_Service()` performs the 12-bit angle read for both sensors when work is pending. Conversion to degrees is purely for UART logging; the broadcast value is kept in counts.
- Data is shifted to signed Q24 centered around mid-scale (`(angle - 2048) << 12`) and saturated to the ±24-bit range before being queued for CAN transmission.
- Each encoder publishes its own CAN frame (IDs `0x180` and `0x182` respectively); payload layout is detailed below.
- Encoder status flags track:
  - `bit0` (`ENC_FLAG_INDEX_SEEN`): at least one valid read has occurred.
  - `bit1` (`ENC_FLAG_ERROR_LATCHED`): a fault has been observed and latched.
  - `bit2` (`ENC_FLAG_SENSOR_FAULT`): latest read failed (cleared on the next good sample).
  - `bit3` (`ENC_FLAG_OVER_SPEED`): reserved for future use; never set in current code.
- I2C faults are logged to USART2 at 115 200 baud. After two consecutive failures the firmware attempts a soft bus recovery (deinit/re-init plus filter restore) before retrying the read once.

### Limit Switches (GPIOA Pins 4–7 / TIM2 sampled)
- SW1 and SW2 each provide normally-closed (NC) and normally-open (NO) contacts on pins `PA4`–`PA7`, sampled with `GPIO_PULLUP`.
- TIM2 runs at 500 Hz in interrupt mode. Its period callback reads all four contacts, detects changes, and records a change mask per switch via `AppLimitSwitch_Poll()`.
- For valid combinations (`NC=0, NO=1` = released, `NC=1, NO=0` = pressed) the firmware derives a binary state (`0` open, `1` closed) and clears the fault flag.
- Each switch publishes its status frame on its own CAN identifier immediately on any state or fault change, and otherwise every 200 ms (5 Hz) while the contacts remain valid.
- When an invalid combination is sampled (`NC == NO`), the frame's fault flag (`byte 1 bit 0`) is asserted and transmissions are rate-limited to once every 500 ms until the contacts return to a valid state (after which the cadence returns to 200 ms).
- The timer ISR only prepares the CAN payload; the main loop attempts the transmit in `AppLimitSwitch_Service()` when the TX FIFO has room.

### Scheduler (TIM1)
- With the configured prescaler and period, TIM1 Update fires every 100 ms.
- The callback defers to `AppEncoder_OnSchedulerTick()`, allowing the main loop to service encoder polling at ~10 Hz via `AppEncoder_Service()`.
- Startup logic requests and processes one encoder sample before the first timer tick so the CAN bus is primed immediately after boot.

### Debug Output (UART)
- `printf` is retargeted to `USART2` at 115 200 baud via `retarget.c`; all logs go out over that UART.

## CAN Interface
Classic CAN (FDCAN1), 11-bit standard IDs, FD features disabled, no CRC inside the payloads (CAN handles CRC).

### 0x180 — Encoder 1 Status (sent every encoder poll, 4-byte DLC)
| Bytes | Description |
|-------|-------------|
| 0-2   | Encoder 1 position, signed Q24 big-endian (`PackS24BE`) |
| 3     | Encoder 1 status flags (`bit0 index_seen`, `bit1 error_latched`, `bit2 sensor_fault`, `bit3 over_speed`, others reserved) |

### 0x182 — Encoder 2 Status (sent every encoder poll, 4-byte DLC)
| Bytes | Description |
|-------|-------------|
| 0-2   | Encoder 2 position, signed Q24 big-endian (`PackS24BE`) |
| 3     | Encoder 2 status flags (`bit0 index_seen`, `bit1 error_latched`, `bit2 sensor_fault`, `bit3 over_speed`, others reserved) |

Notes (apply to both encoder status frames):
- Positions wrap naturally with the signed Q24 representation; receivers should apply their own counts-to-angle scaling using 4 096 counts per revolution. With the reduced DLC, only bytes 0–3 are emitted (no padding).
- Fault bits remain latched until firmware reset; `sensor_fault` clears automatically on a good read.
- Although the code currently polls at 10 Hz, adjust `TIM1` prescaler/period if you require a 100 Hz cadence (update the README accordingly if you change it).

### 0x181 — Limit Switch 1 Status (SW1, immediate on change; 5 Hz valid / 0.5 s fault cadence, 2-byte DLC)
| Bytes | Description |
|-------|-------------|
| 0     | Switch state (`0` open, `1` closed). When a fault is active the last known state is retained. |
| 1     | Fault bits (`bit0`=1 when NC/NO inputs disagree; other bits reserved 0). |

### 0x183 — Limit Switch 2 Status (SW2, immediate on change; 5 Hz valid / 0.5 s fault cadence, 2-byte DLC)
| Bytes | Description |
|-------|-------------|
| 0     | Switch state (`0` open, `1` closed). When a fault is active the last known state is retained. |
| 1     | Fault bits (`bit0`=1 when NC/NO inputs disagree; other bits reserved 0). |

Notes:
- TIM2-driven sampling ensures new levels are observed within 2 ms. The firmware queues an out-of-band frame as soon as a state or fault change is detected.

- While the contacts remain valid, status frames repeat every 200 ms. When a fault persists, transmissions are throttled to once every 500 ms until the inputs recover.
- Frames are queued from the timer callback and emitted in the main loop; if the TX FIFO is full the latest payload is retried on the next pass.

## Extending the Firmware
- To add more diagnostics to CAN, reuse the helper `AppCAN_SendFrame()` and extend the payload tables above.
- Modify the encoder module (`app_encoder.c`) to reflect any additional alarm conditions (e.g., overspeed detection) and document the bits here.
- If you retune the scheduler or switch to asynchronous encoder reads, ensure `AppLimitSwitch_Service()` still runs frequently enough to flush pending frames.

For detailed implementation, refer to the module sources listed above. Update this README whenever payload definitions, scheduling behavior, or module responsibilities change.
