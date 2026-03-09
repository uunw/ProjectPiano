#include "tft_driver.h"

extern SPI_HandleTypeDef hspi1;

// ฟังก์ชันส่ง/รับ 1 ไบต์ แบบสมบูรณ์ (ล้าง FIFO ในตัว)
uint8_t FastSPI_Transfer(uint8_t data) {
    uint32_t timeout = 10000;
    // รอจนกว่าจะพร้อมส่ง
    while (!(SPI1->SR & SPI_SR_TXE)) if (--timeout == 0) return 0;
    *(__IO uint8_t *)&SPI1->DR = data;
    
    // รอจนกว่าจะได้รับข้อมูลกลับ (ยืนยันว่าส่งเสร็จ)
    timeout = 10000;
    while (!(SPI1->SR & SPI_SR_RXNE)) if (--timeout == 0) return 0xFF;
    return *(__IO uint8_t *)&SPI1->DR;
}

// ใช้สำหรับวาดสีความเร็วสูง (ต้องล้าง RX FIFO ด้วย)
void FastSPI_SendByte(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));
    *(__IO uint8_t *)&SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE));
    (void)*(__IO uint8_t *)&SPI1->DR; // อ่านทิ้งเพื่อล้าง FIFO
}

void TFT_WriteCommand(uint8_t cmd) {
    CS_LOW(); DC_LOW();
    FastSPI_Transfer(cmd);
    CS_HIGH();
}

void TFT_WriteData(uint8_t data) {
    CS_LOW(); DC_HIGH();
    FastSPI_Transfer(data);
    CS_HIGH();
}

void TFT_Init(void) {
    // Hardware Reset
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(100);

    TFT_WriteCommand(0x01); // Software Reset
    HAL_Delay(500);

    // Initial sequence จากโปรเจกต์ที่ใช้งานได้
    TFT_WriteCommand(0xEF); TFT_WriteData(0x03); TFT_WriteData(0x80); TFT_WriteData(0x02);
    TFT_WriteCommand(0xCF); TFT_WriteData(0x00); TFT_WriteData(0XC1); TFT_WriteData(0X30);
    TFT_WriteCommand(0xED); TFT_WriteData(0x64); TFT_WriteData(0x03); TFT_WriteData(0X12); TFT_WriteData(0x81);
    TFT_WriteCommand(0xE8); TFT_WriteData(0x85); TFT_WriteData(0x00); TFT_WriteData(0x78);
    TFT_WriteCommand(0xCB); TFT_WriteData(0x39); TFT_WriteData(0x2C); TFT_WriteData(0x00); TFT_WriteData(0x34); TFT_WriteData(0x02);
    TFT_WriteCommand(0xF7); TFT_WriteData(0x20);
    TFT_WriteCommand(0xEA); TFT_WriteData(0x00); TFT_WriteData(0x00);

    TFT_WriteCommand(0xC0); TFT_WriteData(0x23); 
    TFT_WriteCommand(0xC1); TFT_WriteData(0x10); 
    TFT_WriteCommand(0xC5); TFT_WriteData(0x3e); TFT_WriteData(0x28); 
    TFT_WriteCommand(0xC7); TFT_WriteData(0x86); 

    TFT_WriteCommand(0x36); TFT_WriteData(0x28); // Landscape
    TFT_WriteCommand(0x3A); TFT_WriteData(0x55); 

    TFT_WriteCommand(0xB1); TFT_WriteData(0x00); TFT_WriteData(0x18); 
    TFT_WriteCommand(0xB6); TFT_WriteData(0x08); TFT_WriteData(0x82); TFT_WriteData(0x27); 

    TFT_WriteCommand(0x11); // Sleep Out
    HAL_Delay(120);
    TFT_WriteCommand(0x29); // Display ON
    HAL_Delay(20);
}

uint32_t TFT_ReadID(void) {
    uint32_t id = 0;
    CS_LOW();
    DC_LOW(); FastSPI_Transfer(0xD3);
    DC_HIGH();
    FastSPI_Transfer(0x00); // Dummy
    id |= FastSPI_Transfer(0x00) << 16;
    id |= FastSPI_Transfer(0x00) << 8;
    id |= FastSPI_Transfer(0x00);
    CS_HIGH();
    return id;
}
