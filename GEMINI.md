# Project Piano: AKAI MPK mini MK3 x STM32F767 (Standalone)

## 🎹 Project Overview
การพัฒนาระบบควบคุมและ Audio Engine สำหรับ AKAI MPK mini MK3 โดยใช้ **STM32F767ZIT6** เพียงตัวเดียวเป็นทั้ง Input Controller, GUI Orchestrator และ Audio Engine เชื่อมต่อกับจอ ILI9341 ผ่าน Hardware SPI

---

## 🏗 Modular Architecture (Updated V6.0 - Standalone Edition)

### 1. STM32F767 (The All-in-One Engine)
- **High-Performance Audio:** 16-voice Polyphonic, 48kHz/16-bit I2S DMA Output (PCM5102A).
- **Hardware Graphics Acceleration:** 
    - ใช้งาน **Hardware SPI1** วิ่งที่ความเร็วสูงสุด 27MHz.
    - ใช้ **Direct Register Access (BSRR)** สำหรับการคุมขา CS/DC เพื่อลด Overhead.
    - ระบบ Font รองรับตัวพิมพ์ใหญ่ (A-Z) และตัวเลข.
- **Real-time Scanning:** สแกนปุ่มกดเปียโนโดยตรงผ่านขา PE (GPIO Input Pull-up) พร้อมระบบหลบหลีกขา Ethernet.

### 2. UI System (Arctic Light Theme)
- **Landscape Mode:** แสดงผลแนวนอน 320x240 (Rotation 0x28).
- **Gamma Correction:** ใช้ค่าเฉพาะเจาะจงเพื่อให้สีสันคมชัดและไม่มีเส้นกวนบนจอ ILI9341.
- **Visual Feedback:** แสดงชื่อโน้ตที่กดล่าสุดและระดับเสียง (Master Volume) แบบเรียลไทม์.

---

## ⚠️ Strict Mandates (กฎเหล็กในการพัฒนา)

1. **Pin Configuration:**
    - **ห้ามเอเยนต์แก้ไข** Pin Software Defines, Macros ควบคุมขา หรือรายการใน `PIANO_PINS` โดยพลการ
    - การกำหนดขา (Hardware Mapping) ทั้งหมดต้องทำผ่าน `.ioc` หรือ User เป็นคนสั่งเท่านั้น
2. **STM32F7 SPI Standards:**
    - ต้องใช้ **Byte-access** (`*(__IO uint8_t *)&SPIx->DR`) สำหรับการส่งข้อมูล 8 บิตเสมอ
    - ต้องตั้งค่า **FIFO Threshold (FRXTH)** เป็นระดับ Byte เพื่อป้องกันข้อมูลผิดพลาด
3. **UI Strings:**
    - ข้อความที่แสดงบนจอต้องเป็น **ตัวพิมพ์ใหญ่ (UPPERCASE)** เท่านั้น เนื่องจากข้อจำกัดของ Font Engine ปัจจุบัน

---

## 📅 Roadmap & Progress
- [x] Phase 1-8: Matrix Discovery & Multi-core (Legacy).
- [x] Phase 9: **Standalone STM32 Migration**
    - [x] ย้าย Logic ทั้งหมดจาก ESP32 มาลง STM32F7.
    - [x] ตั้งค่า Hardware SPI1 พร้อมความเร็ว 27MHz.
    - [x] แก้ไขปัญหา "จอขาว" และ "เส้นกวน" ด้วย Gamma Tuning.
- [ ] Phase 10: **Advanced Sound & Library (In Progress)**
    - [x] [dq6] **Touchscreen Integration (XPT2046):** รองรับการกดปุ่มบนหน้าจอ (4-Point Calibrated).
    - [x] [cxs] **SPI Speed Optimization:** ปรับจูนความเร็วที่ 6.75MHz เพื่อความเสถียรของ Touch.
    - [x] **Master Volume Control:** ปรับเสียงได้ทั้งผ่าน Touch (Slide) และ VR (ADC PA3).
    - [ ] [dcr] **Advanced Audio Effects:** เพิ่ม Reverb และ Echo.
    - [ ] [714] **Library Management:** ระบบบันทึกเพลง (MIDI) ลง Flash/SD.

---

## 🆘 Troubleshooting (Technical Lessons)
1. **White Screen:** เกิดจาก SPI FIFO ไม่ได้ถูกตั้งเป็น Byte-level (FRXTH) หรือการส่งข้อมูลเป็น 16-bit โดยไม่ตั้งใจ
2. **Horizontal Lines:** แก้ไขได้ด้วยการตั้งค่า Gamma Correction (E0, E1) และ Power Control ให้ตรงตามสเปกชิป
3. **MISO Failure:** หากอ่าน ID ได้ 0x00 หรือ 0xFF ให้ตรวจสอบสาย SDO และการตั้งค่า MspInit ของพอร์ต GPIO
