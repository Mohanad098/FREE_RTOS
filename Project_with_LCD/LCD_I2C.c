#include "TM4C123.h"
#include "LCD_I2C.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

// I2C address of LCD (change to 0x3F if needed)
#define LCD_ADDR 0x27
#define LCD_RS 0x01
#define LCD_EN 0x04
#define LCD_BACKLIGHT 0x08

static void I2C0_Init(void);
static void I2C0_WriteByte(uint8_t addr, uint8_t data);
static void LCD_Write4Bits(uint8_t data);
static void LCD_Cmd(uint8_t cmd);
static void LCD_Data(uint8_t data);
static void LCD_WriteNibble(uint8_t nibble, bool is_data);

void LCD_Init(void) {
    I2C0_Init();

    vTaskDelay(pdMS_TO_TICKS(50));
    LCD_WriteNibble(0x03 << 4, false);
    vTaskDelay(pdMS_TO_TICKS(5));
    LCD_WriteNibble(0x03 << 4, false);
    vTaskDelay(pdMS_TO_TICKS(5));
    LCD_WriteNibble(0x03 << 4, false);
    vTaskDelay(pdMS_TO_TICKS(1));
    LCD_WriteNibble(0x02 << 4, false);

    LCD_Cmd(0x28);
    LCD_Cmd(0x0C);
    LCD_Cmd(0x06);
    LCD_Cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void LCD_Clear(void) {
    LCD_Cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40};
    LCD_Cmd(0x80 | (col + row_offsets[row]));
}

void LCD_Print(const char *str) {
    while (*str) {
        LCD_Data(*str++);
    }
}

void LCD_PrintNum(uint32_t num) {
    char buf[12];
    sprintf(buf, "%lu", num);
    LCD_Print(buf);
}

// Low-level I2C + LCD
// SCL PB2, SDA PB3
static void I2C0_Init(void) {
    SYSCTL->RCGCI2C |= 1;
    SYSCTL->RCGCGPIO |= (1U << 1);
    while (!(SYSCTL->PRGPIO & (1U << 1)));

    GPIOB->AFSEL |= (1U << 2) | (1U << 3);
    GPIOB->ODR |= (1U << 3);
    GPIOB->DEN |= (1U << 2) | (1U << 3);
    GPIOB->PCTL |= (3U << 8) | (3U << 12);

    I2C0->MCR = 0x10;
    I2C0->MTPR = 39; // 50kHz
}

static void I2C0_WriteByte(uint8_t addr, uint8_t data) {
    I2C0->MSA = (addr << 1);
    I2C0->MDR = data;
    I2C0->MCS = 3;
    while (I2C0->MCS & 1);
}

static void LCD_PulseEnable(uint8_t data) {
    I2C0_WriteByte(LCD_ADDR, data | LCD_EN | LCD_BACKLIGHT);
    I2C0_WriteByte(LCD_ADDR, data & ~LCD_EN | LCD_BACKLIGHT);
}

static void LCD_Write4Bits(uint8_t data) {
    I2C0_WriteByte(LCD_ADDR, data | LCD_BACKLIGHT);
    LCD_PulseEnable(data);
}

static void LCD_WriteNibble(uint8_t nibble, bool is_data) {
    uint8_t data = (is_data ? LCD_RS : 0) | (nibble & 0xF0);
    LCD_Write4Bits(data);
}

static void LCD_Cmd(uint8_t cmd) {
    LCD_WriteNibble(cmd & 0xF0, false);
    LCD_WriteNibble((cmd << 4) & 0xF0, false);
}

static void LCD_Data(uint8_t data) {
    LCD_WriteNibble(data & 0xF0, true);
    LCD_WriteNibble((data << 4) & 0xF0, true);
}
