# 🎹 Project Piano: Pinout & Wiring Diagram
เอกสารสรุปการเชื่อมต่อขา (Pinout) ทั้งหมดของระบบ AKAI MPK mini MK3 (Custom Engine)

---

## 1. 📟 ESP32 Controller (Frontend)
บอร์ดทำหน้าที่สแกน Keyboard Matrix, รับค่า Volume และควบคุมหน้าจอ GUI

### 📺 TFT Display (ILI9341 SPI)
| Function | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **VCC** | 3.3V | |
| **GND** | GND | **Common Ground** |
| **CS** | GPIO 5 | Chip Select |
| **RESET** | GPIO 4 | |
| **DC/RS** | GPIO 2 | Data/Command |
| **MOSI** | GPIO 23 | Standard SPI MOSI |
| **SCK** | GPIO 18 | Standard SPI SCK |
| **LED** | **GPIO 32** | PWM Brightness/Fade Control |
| **MISO** | GPIO 19 | Shared SPI MISO (for Touch) |

### 🖱️ Touch Screen (Resistive)
| Function | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **T_CS** | **GPIO 15** | Touch Chip Select |
| **T_CLK** | GPIO 18 | Shared with TFT SCK |
| **T_DIN** (MOSI) | GPIO 23 | Shared with TFT MOSI |
| **T_OUT** (MISO) | **GPIO 19** | ต้องต่อเข้า GPIO 19 เพื่ออ่านค่า |

### 🎹 Keyboard Matrix (8x8 Dual-Contact)
การสแกน Matrix ใช้ **MC14051 (Analog Mux/DeMux)** จำนวน 2 ตัว แบ่งเป็นฝั่ง Row และ Column

#### 1. Row Mux (MC14051 ตัวที่ 1) - จ่ายไฟ (Source)
| Physical IC Pin | Function | Connect to | Note |
| :--- | :--- | :--- | :--- |
| **Pin 16** | VDD | **3.3V** | Power |
| **Pin 8** | VSS | **GND** | |
| **Pin 3** | COM | **3.3V** | Source Supply |
| **Pin 11** | Addr A | **ESP32 GPIO 21** | LSB |
| **Pin 10** | Addr B | **ESP32 GPIO 13** | |
| **Pin 9** | Addr C | **ESP32 GPIO 14** | MSB |
| **Pin 13** | X0 | **Matrix Row 0** | |
| **Pin 14** | X1 | **Matrix Row 1** | |
| **Pin 15** | X2 | **Matrix Row 2** | |
| **Pin 12** | X3 | **Matrix Row 3** | |
| **Pin 1** | X4 | **Matrix Row 4** | |
| **Pin 5** | X5 | **Matrix Row 5** | |
| **Pin 2** | X6 | **Matrix Row 6** | |
| **Pin 4** | X7 | **Matrix Row 7** | |

#### 2. Column Mux (MC14051 ตัวที่ 2) - อ่านค่า (Read)
| Physical IC Pin | Function | Connect to | Note |
| :--- | :--- | :--- | :--- |
| **Pin 3** | COM | **ESP32 GPIO 33** | Matrix Input Signal |
| **Pin 11** | Addr A | **ESP32 GPIO 25** | LSB |
| **Pin 10** | Addr B | **ESP32 GPIO 26** | |
| **Pin 9** | Addr C | **ESP32 GPIO 27** | MSB |
| **Pin 13** | X0 | **Matrix Col 0** | |
| **Pin 14** | X1 | **Matrix Col 1** | |
| **Pin 15** | X2 | **Matrix Col 2** | |
| **Pin 12** | X3 | **Matrix Col 3** | |
| **Pin 1** | X4 | **Matrix Col 4** | |
| **Pin 5** | X5 | **Matrix Col 5** | |
| **Pin 2** | X6 | **Matrix Col 6** | |
| **Pin 4** | X7 | **Matrix Col 7** | |

---

## 2. 🎼 STM32F767ZI (Backend Audio Engine)
บอร์ดทำหน้าที่ประมวลผลเสียง (Polyphonic Synthesis) และส่งออก DAC

### 🔊 I2S Audio Output (PCM5102A DAC Module)
| PCM5102 Pin | STM32 Pin | Function | Note |
| :--- | :--- | :--- | :--- |
| **BCK** | **PB10** | I2S Bit Clock | Serial Clock |
| **LRCK (LCK)** | **PB12** | I2S Word Select | Left/Right Clock (WS) |
| **DIN** | **PC3** | I2S Data Out | Serial Audio Data |
| **GND** | GND | Ground | **Common GND** |
| **VIN** | 5V | Power | จ่ายไฟ 5V จากบอร์ด Nucleo |

#### ⚙️ Hardware Config (บนโมดูล DAC)
| PCM5102 Pin | Connect to | Purpose | Note |
| :--- | :--- | :--- | :--- |
| **SCK (SCL)** | **GND** | Internal PLL | **สำคัญมาก:** ต้องต่อลง GND เพื่อให้ DAC สร้าง Clock เอง |
| **XMT (Mute)** | **3.3V (High)** | Soft Mute | **สำคัญ:** ต้องเป็น High ถึงจะมีเสียง |
| **FMT / DMP / FLT**| **GND** | Config Pins | ตั้งค่าเป็น Standard I2S Mode |

### 🚨 Onboard Peripherals (Debug)
| Function | STM32 Pin | Color | Note |
| :--- | :--- | :--- | :--- |
| **LD1** | PB0 | Green | Power / Status |
| **LD2** | PB7 | Blue | MIDI Activity |
| **LD3** | PB14 | Red | Error / Overflow |
| **User Button**| PC13 | Blue | Reset / Calibrate |

---

## 3. 📡 Inter-Board Communication (UART)
| Connection | ESP32 Pin | STM32 Pin | Protocol | Note |
| :--- | :--- | :--- | :--- | :--- |
| **MIDI Out** | **GPIO 17 (TX2)** | **PD6 (RX2)** | USART2 RX | MIDI จาก ESP เข้า STM |
| **Status In** | **GPIO 16 (RX2)** | **PD5 (TX2)** | USART2 TX | Feedback จาก STM เข้า ESP |
| **Volume Pot** | **GPIO 34 (Analog)**| - | - | ต่อเข้ากับ ESP32 เท่านั้น |
| **Common GND**| **GND** | **GND** | **CRITICAL!** | ต้องเชื่อมกราวด์ถึงกันเสมอ |

**Baudrate:** 115200 bps (8N1)
