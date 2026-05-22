#ifndef __OLED_SSD1306_H
#define __OLED_SSD1306_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* SSD1306 常见 I2C 地址 (7-bit: 0x3C/0x3D, 8-bit: 0x78/0x7A) */
#define SSD1306_I2C_ADDR_PRIMARY    0x78U
#define SSD1306_I2C_ADDR_SECONDARY  0x7AU

/* 显示尺寸 */
#define SSD1306_WIDTH     128U
#define SSD1306_HEIGHT    64U
#define SSD1306_PAGES     8U    /* 64 / 8 = 8页 */

/* 每行字符数 (5x7字体 + 1像素间距) */
#define SSD1306_CHARS_PER_LINE  21U
#define SSD1306_MAX_LINES       8U

bool SSD1306_Init(I2C_HandleTypeDef *hi2c);
void SSD1306_SetI2CLockCallbacks(void (*lock_fn)(void), void (*unlock_fn)(void));
bool SSD1306_IsReady(void);
uint8_t SSD1306_GetI2CAddress(void);
void SSD1306_Clear(void);
void SSD1306_SetCursor(uint8_t line, uint8_t col);
void SSD1306_PutChar(char ch);
void SSD1306_Print(uint8_t line, uint8_t col, const char *str);
void SSD1306_printf(uint8_t line, uint8_t col, const char *fmt, ...);
void SSD1306_DrawBitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t *bmp);
void SSD1306_SetPixel(uint8_t x, uint8_t y, bool on);
void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on);
void SSD1306_Update(void);

#ifdef __cplusplus
}
#endif

#endif
