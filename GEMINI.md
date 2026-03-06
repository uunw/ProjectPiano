# Project Piano: AKAI MPK mini MK3 x STM32F767 + ESP32

## 🎹 Project Overview
การเปลี่ยนระบบควบคุมของ AKAI MPK mini MK3 โดยใช้ **STM32F767ZIT6** เป็น Audio Engine (Backend) และ **ESP32** เป็น Input/GUI Controller (Frontend) เชื่อมต่อกันด้วยโปรโตคอล MIDI-over-UART

---

## 🏗 Modular Architecture (Updated V5.8 - Arctic Edition)

### 1. ESP32 (Frontend - Orchestrator)
- **Dual-Core FreeRTOS:** 
    - **Core 0 (TaskMIDI):** สแกนคีย์ความเร็วสูง, ประมวลผล MIDI, ระบบ Recording/Playback, และ Bidirectional Heartbeat
    - **Core 1 (TaskGUI):** จัดการจอภาพ TFT และระบบสัมผัส (Touch) เพื่อไม่ให้ขัดจังหวะการเล่นดนตรี
- **UI Engine (TFT_eSPI):** 
    - **Arctic Light Theme:** UI สีขาวมินิมอล พร้อมปุ่มกด 5 ปุ่ม (PNO, ORG, SYN, REC, LIB)
    - **Smart Overlays:** หน้าจอ Master Volume ขนาดใหญ่ที่จะปรากฏขึ้นเฉพาะตอนปรับค่า และหรี่ไฟ (Dimming) อัตโนมัติเมื่อไม่ได้ใช้งาน
- **State Management (ArduinoJson):** บันทึกค่าระดับเสียง, เครื่องดนตรีล่าสุด และสถานะระบบลงใน Flash Memory (LittleFS)
- **File System (StorageManager):** ระบบจัดการคลังเพลง (Library) รองรับการบันทึกหลายไฟล์, การลบไฟล์ และ Media Player ในตัว

### 2. STM32F767 (Backend - Audio Engine)
- **Multi-Engine Synth:** 16-voice Polyphonic รองรับ 3 โหมดเสียง (Piano, Organ, Synth) เปลี่ยนผ่าน MIDI Program Change
- **DSP Effects:** ระบบ ADSR Envelope ที่นุ่มนวล และเอฟเฟกต์ **Echo/Delay** เพื่อเพิ่มมิติเสียง
- **Resilient Comm:** สื่อสารผ่าน **USART2 (PD5/PD6)** พร้อมระบบ **Handshake & Heartbeat** (Auto-Silence เมื่อสายหลุด)
- **Debug Port:** แยก **USART3 (PD8/PD9)** สำหรับส่ง Log เข้าคอมพิวเตอร์และ Forward ไปยัง ESP32

---

## 🛠 Hardware Specifications
*ดูรายละเอียดฉบับเต็มได้ที่ [PINOUT.md](PINOUT.md)*

| Interface | ESP32 Pins | STM32 Pins | Note |
| :--- | :--- | :--- | :--- |
| **UART (MIDI)** | TX2(17), RX2(16) | RX2(PD6), TX2(PD5)| Handshake + Heartbeat |
| **TFT (SPI)** | CS(5), DC(2), RST(4), MOSI(23), SCK(18), MISO(19) | - | Arctic Light UI |
| **Touch** | T_CS(15), T_CLK(18), T_DIN(23), T_DO(19) | - | 1MHz SPI for stability |
| **Audio I2S** | - | BCK(PB10), WS(PB12), DIN(PC3) | PCM5102A DAC |

---

## 📅 Roadmap & Progress
- [x] Phase 1-7: Matrix Discovery & Basic Audio - *Completed*
- [x] Phase 8: **Modular Migration & Multi-core Integration**
  - [x] Arctic Light UI with High-performance TFT_eSPI.
  - [x] Logarithmic Velocity Curve & 16-voice Polyphony.
  - [x] Multi-file Recording & Library Manager (Delete/Play).
  - [x] Persistent Settings via ArduinoJson.
  - [x] Bidirectional Heartbeat & Safety Auto-Silence.
  - [x] Modular Code Structure (StorageManager, UI Modules).
- [ ] Phase 9: **Advanced Sound Design**
  - [ ] Sound Effects Tuning (Reverb parameters) via UI.
  - [ ] Support for more MIDI CC commands (Sustain Pedal).
- [ ] Phase 10: **Standalone Hardware Design**
  - [ ] 3D Printed Case for Screen and Controls.
  - [ ] Integrated Power Circuitry.

---

## 🆘 Lessons Learned (Troubleshooting)
1. **UART Conflict:** ขา PD8/PD9 บน Nucleo ต่อกับ ST-Link ต้องย้ายไป USART2 (PD5/PD6) เพื่อเลี่ยงสัญญาณตีกัน
2. **SPI Interference:** เมื่อใช้ Touch และ Screen ร่วมกัน ควรต่อสาย MISO และลดความเร็ว Touch Clock (1MHz)
3. **God Object:** การแยกไฟล์เป็น UI, Screens, และ Storage ช่วยลดความซับซ้อนและแก้ปัญหา Compile Error ได้ดีที่สุด
4. **Heartbeat:** การส่งสัญญาณชีพ 2 ทางช่วยป้องกันเสียงค้าง (Note Hang) เมื่อสายสัญญาณหลุดได้ดีเยี่ยม
