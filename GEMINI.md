# Project Piano: AKAI MPK mini MK3 x STM32F767

## 🎹 Project Overview
Replacement of the original AKAI controller with an **STM32F767ZIT6** (NUCLEO-F767ZI).
Goal: Low-latency MIDI controller + Standalone Polyphonic Piano (PCM5102 DAC).

---

## 🛠 Hardware Specifications

### 1. Keyboard Matrix (AKAI Custom)
- **Matrix:** 8 Rows x 8 Columns (Dual-contact).
- **Wiring:**
  - **Even Rows (0, 2, 4, 6):** S1 Contact (First touch).
  - **Odd Rows (1, 3, 5, 7):** S2 Contact (Bottom touch - Triggers Note On).
- **Logic:** Note On is triggered ONLY on S2 (Odd Rows) to prevent double triggers.

### 2. Peripheral Pinout (Active)

| Function | Pin(s) | Peripheral | Note |
| :--- | :--- | :--- | :--- |
| **I2S BCK** | **PB10** | I2S2_CK | Bit Clock to PCM5102 |
| **I2S WS** | **PB12** | I2S2_WS | Word Select (LRCK) |
| **I2S SD** | **PC3** | I2S2_SD | Serial Data (DIN) |
| **Matrix Rows** | PE0 - PE7 | GPIOE | Output Scan (Very High Speed) |
| **Matrix Cols** | PF0 - PF7 | GPIOF | Input (Pull-down) |
| **Debug UART** | PD8, PD9 | USART3 | 115200 bps |

---

## 🆘 Troubleshooting & Development Log (Lessons Learned)

### 1. Audio/DMA Issues (STM32F7 Specific)
- **Problem:** `DMA Callbacks` remained 0 despite correct initialization.
- **Cause:** **DTCM RAM vs DMA1.** On STM32F7, DMA1 cannot access the DTCM RAM region (starts at `0x20000000`). Global variables are placed there by default.
- **Solution:** Use a "Pusher" array (128KB) or `__attribute__((section(".data")))` to force the `audio_buffer` into **SRAM1** (starts at `0x20020000`).

### 2. I2S Clock Issue
- **Problem:** I2S peripheral stuck in Busy/Ready state without requesting DMA.
- **Cause:** PLLI2S was configured but not assigned to the I2S peripheral.
- **Solution:** Explicitly set `PeriphClockSelection = RCC_PERIPHCLK_I2S` in `stm32f7xx_hal_msp.c`.

### 3. PCM5102 Hardware Gotchas
- **XSMT (Soft Mute):** Must be tied to **3.3V**. If left floating or GND, DAC is silent.
- **SCK:** Must be tied to **GND** to enable Internal PLL mode (since we don't provide MCLK).
- **GPIO Speed:** I2S pins MUST be set to **Very High Speed** for stable 48kHz audio.

### 4. Matrix Mapping Disorder
- **Problem:** Keys were out of order and some didn't trigger.
- **Solution:** Implemented a `note_map[8][8]` lookup table. AKAI's PCB layout is non-linear. The mapping now follows a standard Chromatic Scale starting from C2.

---

## 🔊 Audio Engine Specs
- **Format:** 16-bit Signed Stereo, **48,000 Hz**.
- **Synthesis:** Additive (Fundamental + 3 Harmonics).
- **Envelope:** Exponential Decay (Piano release feel).
- **Polyphony:** 8 Simultaneous Voices.

---

## 📅 Roadmap & Progress
- [x] Phase 1: Matrix Discovery.
- [x] Phase 2: Correct Note Mapping (LUT implemented).
- [x] Phase 6: Polyphonic Audio Engine (I2S + DMA + Mixer).
- [x] Phase 7: **Velocity Sensitivity Logic** (Timing S1 to S2).
- [ ] Phase 8: **GUI Implementation** (ILI9341).

---

## 🔗 References
- [AKAI MPK mini MK3 Matrix Info](https://wave.hatenablog.com/entry/2023/04/16/212100)
