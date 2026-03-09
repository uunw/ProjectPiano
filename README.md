# Project Piano: AKAI MPK mini MK3 x STM32F767 (Standalone)

Standalone Music Engine and Control System built on **STM32F767ZIT6**. This project transforms the AKAI MPK mini MK3 hardware into a self-contained synthesizer and sequencer without the need for an external computer.

## 🎹 Features
- **All-in-One Engine:** High-performance audio generation and GUI orchestration on a single MCU.
- **Polyphonic Audio:** 16-voice polyphony, 48kHz/16-bit I2S DMA output (PCM5102A).
- **Multi-Mode Engine:** Toggle between GRAND PIANO, E-PIANO, and STRINGS via touchscreen.
- **8x8 Matrix Scanning:** Direct piano key scanning with velocity sensitivity support.
- **Sequencer & Storage:** 4 recording slots (up to 2,000 events each) with persistent storage in internal Flash.
- **Audio Effects:** Integrated cubic soft clipping and dynamic headroom management.
- **Interactive GUI:** 320x240 TFT LCD with touch-based control for mode switching and volume.

## 🏗 System Architecture
- **MCU:** STM32F767ZIT6 (Cortex-M7, 216MHz)
- **Audio DAC:** PCM5102A (via I2S)
- **Display:** ILI9341 3.2" TFT (via SPI)
- **Input:** 8x8 Matrix Scanning (Keyboard) + ADC (Volume Potentiometer)

## 📁 Documentation
Detailed project details can be found in the following documents:
- [GEMINI.md](GEMINI.md) - Project overview and core mandates.
- [PINOUT.md](PINOUT.md) - Hardware pin mappings.
- [WIRING.md](WIRING.md) - Wiring diagrams and connections.
- [INTERRUPTS.md](INTERRUPTS.md) - Interrupt and peripheral module details.

## 🚀 Getting Started
1. **Hardware:** Connect the STM32 Nucleo-144 board to the AKAI MPK mini MK3 hardware according to `PINOUT.md`.
2. **Build:** Open the project in STM32CubeIDE.
3. **Flash:** Build and upload to the target STM32F767 board.

## ⚠️ Important Note
This project uses **Hardware SPI1** for the display and **SPI2 (I2S)** for audio. Do not exceed 13MHz SPI speed for the display to ensure touchscreen stability.
