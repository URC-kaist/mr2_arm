# MR2 Arm Firmware

Firmware targeting the STM32H523 MCU that supervises two AS5600 magnetic encoders, a limit switch on EXTI1, and broadcasts system state over classic CAN (FDCAN1). The code lives under `Core/Src/main.c` and is generated from STM32CubeMX, with custom logic in the `USER CODE` sections.

## High-Level Flow
- System clock configured to run from the CSI oscillator via PLL1 (SYSCLK ≈ 250 MHz).
- GPIO, I2C1/I2C2, ICACHE, FDCAN1, USART2, and TIM1 peripherals are initialized.
- FDCAN1 is started in normal classic-CAN mode and ready to transmit.
- TIM1 runs as a 10 Hz scheduler (prescaler 24 999, period 999). Its update interrupt raises `angle_check_pending` to request an encoder refresh.
- The main loop services queued limit-switch CAN events, executes encoder refresh work when requested, and idles via `__WFI()` when nothing is pending.

## Sensors and Peripherals

### AS5600 Encoders (I2C1 / I2C2)
- Each bus targets an AS5600 at 0x36 (7-bit address). Reads use `HAL_I2C_Mem_Read` from register `0x0E` (`ANGLE`).
- `AngleCheck_Run()` pulls the 12-bit angle from both sensors. Conversion to degrees is purely for UART logging; the broadcast value is kept in counts.
- Data is shifted to signed Q24 centered around mid-scale (`(angle - 2048) << 12`) and saturated to the ±24-bit range before being queued for CAN transmission.
- Each encoder publishes its own CAN frame (IDs `0x180` and `0x182` respectively); payload layout is detailed below.
- Encoder status flags track:
  - `bit0` (`ENC_FLAG_INDEX_SEEN`): at least one valid read has occurred.
  - `bit1` (`ENC_FLAG_ERROR_LATCHED`): a fault has been observed and latched.
  - `bit2` (`ENC_FLAG_SENSOR_FAULT`): latest read failed (cleared on the next good sample).
  - `bit3` (`ENC_FLAG_OVER_SPEED`): reserved for future use; never set in current code.
- I2C errors are logged to USART2 at 115 200 baud for diagnostics.

### Limit Switch (GPIOA Pin 1 / EXTI1)
- Configured for rising and falling edge interrupts with no pull resistors.
- ISR (`HAL_GPIO_EXTI_Callback`) snapshots the input level, generates a rising/falling edge flag, increments a 16-bit event counter, and timestamps the event with `HAL_GetTick()` (ms).
- Event payload is buffered atomically; the main loop pushes the CAN frame when FDCAN TX FIFO space is available.

### Scheduler (TIM1)
- With the configured prescaler and period, TIM1 Update fires every 100 ms.
- The callback increments `angle_check_pending`, allowing the main loop to run encoder polling at ~10 Hz.
- The initial call to `AngleCheck_Run()` ensures the encoders are read immediately after boot before the first timer tick.

### Serial Debug (USART2)
- Simple TX-only logging via `UART_SendLine()` for probe results and angle readouts.

## CAN Interface
Classic CAN (FDCAN1), 11-bit standard IDs, FD features disabled, no CRC inside the payloads (CAN handles CRC).

### 0x180 — Encoder 1 Status (sent every encoder poll)
| Bytes | Description |
|-------|-------------|
| 0-2   | Encoder 1 position, signed Q24 big-endian (`PackS24BE`) |
| 3     | Encoder 1 status flags (`bit0 index_seen`, `bit1 error_latched`, `bit2 sensor_fault`, `bit3 over_speed`, others reserved) |
| 4-7   | Reserved (set to 0) |

### 0x182 — Encoder 2 Status (sent every encoder poll)
| Bytes | Description |
|-------|-------------|
| 0-2   | Encoder 2 position, signed Q24 big-endian (`PackS24BE`) |
| 3     | Encoder 2 status flags (`bit0 index_seen`, `bit1 error_latched`, `bit2 sensor_fault`, `bit3 over_speed`, others reserved) |
| 4-7   | Reserved (set to 0) |

Notes (apply to both encoder status frames):
- Positions wrap naturally with the signed Q24 representation; receivers should apply their own counts-to-angle scaling using 4 096 counts per revolution.
- Fault bits remain latched until firmware reset; `sensor_fault` clears automatically on a good read.
- Although the code currently polls at 10 Hz, adjust `TIM1` prescaler/period if you require a 100 Hz cadence (update the README accordingly if you change it).

### 0x181 — Limit Switch Event (sent on every EXTI1 edge)
| Bytes | Description |
|-------|-------------|
| 0     | Switch state (`0` released, `1` pressed) |
| 1     | Edge flags (`bit0` rising edge, `bit1` falling edge; only one set) |
| 2-3   | Event counter (`uint16`, big-endian) |
| 4-7   | Timestamp (system tick in ms, big-endian, wrap-around permitted) |

Notes:
- Frames are generated in the ISR but emitted from the main loop to keep interrupt latency low. If the TX FIFO is full, the frame is re-queued for the next iteration.
- Consumers can detect missed edges by observing jumps in the event counter.

## Extending the Firmware
- To add more diagnostics to CAN, reuse the helper `CAN_SendFrame()` and extend the payload tables above.
- Modify the `encoder_status_flags` logic to reflect any additional alarm conditions (e.g., overspeed detection) and document the bits here.
- If you retune the scheduler or switch to asynchronous encoder reads, ensure `LimitSwitch_Service()` still runs frequently enough to flush pending frames.

For detailed implementation, consult `Core/Src/main.c` (line numbers referenced above). Update this README whenever payload definitions or scheduling behavior change.
