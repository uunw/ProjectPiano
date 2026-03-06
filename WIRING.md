# 🔌 Project Piano: Comprehensive Wiring Diagram

เอกสารฉบับนี้รวบรวมการเชื่อมต่อทางฮาร์ดแวร์ทั้งหมดของระบบ โดยอ้างอิงตามโค้ดปัจจุบัน (V5.8 Arctic Edition)

---

## 📊 System Architecture (Mermaid)

```mermaid
graph TD
    subgraph "Power Supply (7V-12V DC)"
        PS[Switching Supply]
    end

    subgraph "ESP32 (Frontend Controller)"
        E_VIN[VIN]
        E_GND[GND]
        E_3V3[3.3V]
        E_UART_TX[GPIO 17]
        E_UART_RX[GPIO 16]
        E_MAT_IN[GPIO 33]
        E_POT[GPIO 34]
    end

    subgraph "STM32F767ZI (Audio Engine)"
        S_VIN[VIN / JP3 to VIN]
        S_GND[GND]
        S_5V[5V]
        S_3V3[3.3V]
        S_UART_RX[PD6]
        S_UART_TX[PD5]
        S_I2S_BCK[PB10]
        S_I2S_WS[PB12]
        S_I2S_SD[PC3]
    end

    subgraph "PCM5102A DAC (Audio Output)"
        D_VCC[VCC/VIN]
        D_GND[GND]
        D_BCK[BCK]
        D_LRCK[LCK/LRCK]
        D_DIN[DIN]
        D_SCL[SCL/SCK]
        D_XMT[XMT/MUTE]
    end

    %% Power Connections
    PS -->|Positive| E_VIN
    PS -->|Positive| S_VIN
    PS -->|Negative| E_GND
    PS -->|Negative| S_GND
    E_GND --- S_GND
    S_GND --- D_GND

    %% Communication
    E_UART_TX -->|MIDI Data| S_UART_RX
    S_UART_TX -->|Heartbeat| E_UART_RX

    %% Audio Path
    S_5V --> D_VCC
    S_I2S_BCK --> D_BCK
    S_I2S_WS --> D_LRCK
    S_I2S_SD --> D_DIN
    S_GND --> D_SCL
    S_3V3 --> D_XMT
```

---

## 📋 Detailed Pin Mapping

### 1. 🎹 Keyboard Matrix (MC14051 Mux x2)
| Function | Mux Pin | Connect to | Note |
| :--- | :--- | :--- | :--- |
| **Row Addr A** | 11 | **ESP32 GPIO 21** | LSB |
| **Row Addr B** | 10 | **ESP32 GPIO 13** | |
| **Row Addr C** | 9 | **ESP32 GPIO 14** | MSB |
| **Row COM** | 3 | **ESP32 3.3V** | Source Supply |
| **Col Addr A** | 11 | **ESP32 GPIO 25** | LSB |
| **Col Addr B** | 10 | **ESP32 GPIO 26** | |
| **Col Addr C** | 9 | **ESP32 GPIO 27** | MSB |
| **Col COM** | 3 | **ESP32 GPIO 33** | **Read Signal** |
| **GND Common** | 6, 7, 8 | **GND** | INH, VEE, VSS |

### 2. 📺 TFT Display & Touch (SPI)
| TFT Pin | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **VCC** | **3.3V** | **CRITICAL: DO NOT USE 5V** |
| **GND** | **GND** | |
| **CS** | GPIO 5 | |
| **RESET** | GPIO 4 | |
| **DC/RS** | GPIO 2 | |
| **MOSI** | GPIO 23 | |
| **SCK** | GPIO 18 | |
| **LED** | GPIO 32 | PWM Dimming |
| **T_CS** | GPIO 15 | Touch CS |
| **T_DO** | GPIO 19 | MISO (For Watchdog) |

### 3. 📡 Inter-Board Comms (UART)
| ESP32 Pin | STM32 Pin | Function |
| :--- | :--- | :--- |
| **GPIO 17 (TX2)** | **PD6 (RX2)** | MIDI Out |
| **GPIO 16 (RX2)** | **PD5 (TX2)** | Status In |
| **GND** | **GND** | **MUST CONNECT** |

### 4. 🔊 Audio Path (I2S)
| STM32 Pin | DAC Pin | Function |
| :--- | :--- | :--- |
| **5V (Morpho)** | **VIN** | Power DAC |
| **PB10** | **BCK** | Bit Clock |
| **PB12** | **LRCK** | Word Select |
| **PC3** | **DIN** | Data |
| **GND** | **SCL (SCK)** | **Internal PLL Enable** |
| **3.3V** | **XMT (MUTE)**| **Unmute (High)** |

---

## ⚠️ Important Safety & Stability Notes

1. **Common Ground:** GND ทุกจุดต้องเชื่อมต่อถึงกัน สัญญาณถึงจะนิ่ง
2. **Matrix Noise:** หากพบคอลัมน์เลขคี่ (1,3,5,7) ติดเอง ให้ต่อตัวต้านทาน **10k Ohm** ระหว่าง **GPIO 33 และ GND**
3. **Display Power:** ขา VCC ของจอต้องต่อกับ **3.3V** เท่านั้น การต่อกับ VIN (7V-12V) จะทำให้จอพังทันที
4. **DAC Setup:** ขา **SCL** ต้องลงดิน และขา **XMT** ต้องไป 3.3V เท่านั้นเสียงถึงจะออก
