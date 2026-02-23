# Project Piano: AKAI MPK mini MK3 x STM32F767

## 🎹 Project Overview
This project aims to replace the original controller of the **AKAI MPK mini MK3** with an **STM32F767ZIT6** (NUCLEO-F767ZI) microcontroller. The goal is to create a high-performance, low-latency MIDI controller with velocity-sensitive keys, standalone polyphonic sound, and a visual interface.

---

## 🛠 Hardware Specifications

### 1. Keyboard PCB (Keybed)
- **Model:** PC19A012 (AKAI MPK mini MK3)
- **Interface:** 16-pin ribbon cable (Direct GPIO).
- **Matrix:** 8 Rows x 8 Columns (Velocity sensitive).

### 2. Microcontroller (MCU)
- **Model:** STM32F767ZIT6 (ARM Cortex-M7 @ 216MHz)
- **Board:** NUCLEO-F767ZI

### 3. Peripherals & Pin Planning
- **Matrix Rows:** GPIOE (PE0 - PE7) - [Output]
- **Matrix Columns:** GPIOF (PF0 - PF7) - [Input + Pull-down]
- **Debug:** USART3 (PD8, PD9) - [115200 bps]
- **USB MIDI:** USB_OTG_FS (PA8 - PA12)
- **Precision Timing:** TIM2 (32-bit) for Velocity.
- **Audio Output:** I2S (Planned for External DAC like PCM5102A).
- **Visual Display:** **ILI9341 LCD** (SPI Interface - Planned).
- **Volume Control:** **Potentiometer** (ADC1 - Planned).

---

## 🔍 Implementation Strategy

### 1. Visual Interface (GUI)
- **Hardware:** ILI9341 2.4"/2.8" TFT LCD (240x320).
- **Features:** Display current MIDI status, Volume levels, Sound presets, and Real-time waveform.
- **Library:** Optimized SPI driver for STM32F7.

### 2. Audio & Volume Control
- **Analog Volume:** Use ADC to read the physical potentiometer and scale the digital audio samples in real-time.
- **Polyphony:** Software mixing of multiple oscillators for chord support.

---

## 📅 Roadmap & Progress

- [x] **Phase 0:** Project Initialization & Peripheral Config (IOC).
- [x] **Phase 1:** Matrix Discovery Script (Scanning with Timestamps).
- [ ] **Phase 2:** Key Mapping Table & Velocity Logic.
- [ ] **Phase 3:** USB MIDI Integration (TinyUSB).
- [ ] **Phase 4:** **GUI Implementation (ILI9341 SPI Driver)**.
- [ ] **Phase 5:** **Analog Volume Integration (ADC)**.
- [ ] **Phase 6:** Standalone Polyphonic Audio Engine (I2S + Mixer).

---

## 🔗 References
- [AKAI MPK mini MK3 Disassembly & Matrix Info](https://wave.hatenablog.com/entry/2023/04/16/212100)
- [ILI9341 Datasheet](https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf)
- [TinyUSB GitHub](https://github.com/hathach/tinyusb)
