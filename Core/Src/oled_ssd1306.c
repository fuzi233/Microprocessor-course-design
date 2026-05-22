#include "oled_ssd1306.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* 5x7 字体表 (ASCII 32-127) */
static const uint8_t font_5x7[96][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* 空格 */
    {0x00,0x00,0x5F,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* $ */
    {0x23,0x13,0x08,0x64,0x62}, /* % */
    {0x36,0x49,0x55,0x22,0x50}, /* & */
    {0x00,0x05,0x03,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00}, /* ) */
    {0x08,0x2A,0x1C,0x2A,0x08}, /* * */
    {0x08,0x08,0x3E,0x08,0x08}, /* + */
    {0x00,0x50,0x30,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08}, /* - */
    {0x00,0x60,0x60,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00}, /* ; */
    {0x00,0x08,0x14,0x22,0x41}, /* < */
    {0x14,0x14,0x14,0x14,0x14}, /* = */
    {0x41,0x22,0x14,0x08,0x00}, /* > */
    {0x02,0x01,0x51,0x09,0x06}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x01,0x01}, /* F */
    {0x3E,0x41,0x41,0x51,0x32}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x04,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x7F,0x20,0x18,0x20,0x7F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x03,0x04,0x78,0x04,0x03}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}, /* Z */
    {0x00,0x00,0x7F,0x41,0x41}, /* [ */
    {0x02,0x04,0x08,0x10,0x20}, /* \ */
    {0x41,0x41,0x7F,0x00,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04}, /* ^ */
    {0x40,0x40,0x40,0x40,0x40}, /* _ */
    {0x00,0x01,0x02,0x04,0x00}, /* ` */
    {0x20,0x54,0x54,0x54,0x78}, /* a */
    {0x7F,0x48,0x44,0x44,0x38}, /* b */
    {0x38,0x44,0x44,0x44,0x20}, /* c */
    {0x38,0x44,0x44,0x48,0x7F}, /* d */
    {0x38,0x54,0x54,0x54,0x18}, /* e */
    {0x08,0x7E,0x09,0x01,0x02}, /* f */
    {0x08,0x14,0x54,0x54,0x3C}, /* g */
    {0x7F,0x08,0x04,0x04,0x78}, /* h */
    {0x00,0x44,0x7D,0x40,0x00}, /* i */
    {0x20,0x40,0x44,0x3D,0x00}, /* j */
    {0x00,0x7F,0x10,0x28,0x44}, /* k */
    {0x00,0x41,0x7F,0x40,0x00}, /* l */
    {0x7C,0x04,0x18,0x04,0x78}, /* m */
    {0x7C,0x08,0x04,0x04,0x78}, /* n */
    {0x38,0x44,0x44,0x44,0x38}, /* o */
    {0x7C,0x14,0x14,0x14,0x08}, /* p */
    {0x08,0x14,0x14,0x18,0x7C}, /* q */
    {0x7C,0x08,0x04,0x04,0x08}, /* r */
    {0x48,0x54,0x54,0x54,0x20}, /* s */
    {0x04,0x3F,0x44,0x40,0x20}, /* t */
    {0x3C,0x40,0x40,0x20,0x7C}, /* u */
    {0x1C,0x20,0x40,0x20,0x1C}, /* v */
    {0x3C,0x40,0x30,0x40,0x3C}, /* w */
    {0x44,0x28,0x10,0x28,0x44}, /* x */
    {0x0C,0x50,0x50,0x50,0x3C}, /* y */
    {0x44,0x64,0x54,0x4C,0x44}, /* z */
    {0x00,0x08,0x36,0x41,0x00}, /* { */
    {0x00,0x00,0x7F,0x00,0x00}, /* | */
    {0x00,0x41,0x36,0x08,0x00}, /* } */
    {0x08,0x04,0x08,0x10,0x08}, /* ~ */
    {0x00,0x00,0x00,0x00,0x00}, /* DEL */
};

static I2C_HandleTypeDef *s_hi2c;
static uint8_t s_i2c_addr;
static bool s_ready;
static uint8_t s_buffer[SSD1306_PAGES][SSD1306_WIDTH];
static uint8_t s_cursor_x;
static uint8_t s_cursor_y;

/* I2C 总线锁回调（与 MPU6500 共用 I2C1 时使用） */
static void (*s_i2c_lock)(void) = NULL;
static void (*s_i2c_unlock)(void) = NULL;

void SSD1306_SetI2CLockCallbacks(void (*lock_fn)(void), void (*unlock_fn)(void))
{
    s_i2c_lock = lock_fn;
    s_i2c_unlock = unlock_fn;
}

/*
 * 常见 0.96" SSD1306 模块通常用 A1 + C8 更容易正常阅读。
 * 如果后续仍然倒置，只需要在这里改这两个常量。
 */
#define SSD1306_SEG_REMAP_CMD  0xA0U
#define SSD1306_COM_SCAN_CMD   0xC8U

static HAL_StatusTypeDef SSD1306_WriteCmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};  /* 0x00 = 命令模式 */
    if ((s_hi2c == NULL) || !s_ready)
    {
        return HAL_ERROR;
    }

    if (s_i2c_lock != NULL) s_i2c_lock();
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(s_hi2c, s_i2c_addr, buf, 2, 100);
    if (s_i2c_unlock != NULL) s_i2c_unlock();
    return ret;
}

static HAL_StatusTypeDef SSD1306_WriteData(uint8_t data)
{
    uint8_t buf[2] = {0x40, data};  /* 0x40 = 数据模式 */
    if ((s_hi2c == NULL) || !s_ready)
    {
        return HAL_ERROR;
    }

    if (s_i2c_lock != NULL) s_i2c_lock();
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(s_hi2c, s_i2c_addr, buf, 2, 100);
    if (s_i2c_unlock != NULL) s_i2c_unlock();
    return ret;
}

bool SSD1306_Init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    s_i2c_addr = 0U;
    s_ready = false;
    s_cursor_x = 0;
    s_cursor_y = 0;
    memset(s_buffer, 0, sizeof(s_buffer));

    if (s_hi2c == NULL)
    {
        return false;
    }

    if (HAL_I2C_IsDeviceReady(s_hi2c, SSD1306_I2C_ADDR_PRIMARY, 2U, 50U) == HAL_OK)
    {
        s_i2c_addr = SSD1306_I2C_ADDR_PRIMARY;
    }
    else if (HAL_I2C_IsDeviceReady(s_hi2c, SSD1306_I2C_ADDR_SECONDARY, 2U, 50U) == HAL_OK)
    {
        s_i2c_addr = SSD1306_I2C_ADDR_SECONDARY;
    }
    else
    {
        return false;
    }

    s_ready = true;

    /* SSD1306 初始化序列 */
    SSD1306_WriteCmd(0xAE);  /* 关闭显示 */

    SSD1306_WriteCmd(0xD5);  /* 设置显示时钟分频 */
    SSD1306_WriteCmd(0x80);

    SSD1306_WriteCmd(0xA8);  /* 设置多路复用比 */
    SSD1306_WriteCmd(0x3F);  /* 64行 */

    SSD1306_WriteCmd(0xD3);  /* 设置显示偏移 */
    SSD1306_WriteCmd(0x00);

    SSD1306_WriteCmd(0x40);  /* 设置起始行 */

    SSD1306_WriteCmd(0x8D);  /* 启用电荷泵 */
    SSD1306_WriteCmd(0x14);

    SSD1306_WriteCmd(0x20);  /* 内存寻址模式 */
    SSD1306_WriteCmd(0x00);  /* 水平寻址 */

    SSD1306_WriteCmd(SSD1306_SEG_REMAP_CMD);
    SSD1306_WriteCmd(SSD1306_COM_SCAN_CMD);

    SSD1306_WriteCmd(0xDA);  /* 设置COM引脚配置 */
    SSD1306_WriteCmd(0x12);

    SSD1306_WriteCmd(0x81);  /* 设置对比度 */
    SSD1306_WriteCmd(0xCF);

    SSD1306_WriteCmd(0xD9);  /* 设置预充电周期 */
    SSD1306_WriteCmd(0xF1);

    SSD1306_WriteCmd(0xDB);  /* 设置VCOMH */
    SSD1306_WriteCmd(0x40);

    SSD1306_WriteCmd(0xA4);  /* 输出跟随RAM */
    SSD1306_WriteCmd(0xA6);  /* 正常显示(非反色) */

    SSD1306_WriteCmd(0x2E);  /* 停止滚动 */

    SSD1306_WriteCmd(0xAF);  /* 开启显示 */

    SSD1306_Clear();
    SSD1306_Update();

    return true;
}

bool SSD1306_IsReady(void)
{
    return s_ready;
}

uint8_t SSD1306_GetI2CAddress(void)
{
    return s_i2c_addr;
}

void SSD1306_Clear(void)
{
    memset(s_buffer, 0, sizeof(s_buffer));
    s_cursor_x = 0;
    s_cursor_y = 0;
}

void SSD1306_SetCursor(uint8_t line, uint8_t col)
{
    if (line >= SSD1306_MAX_LINES) line = SSD1306_MAX_LINES - 1;
    if (col >= SSD1306_CHARS_PER_LINE) col = SSD1306_CHARS_PER_LINE - 1;
    s_cursor_y = line;
    s_cursor_x = col * 6;  /* 每字符6像素宽 (5+1间距) */
}

static void SSD1306_DrawChar(char ch)
{
    uint8_t idx;
    uint8_t page;
    uint8_t bit;
    uint8_t i;

    if (ch < 32 || ch > 127) ch = ' ';
    idx = (uint8_t)(ch - 32);

    page = s_cursor_y;

    for (i = 0; i < 5; i++)
    {
        if (s_cursor_x + i >= SSD1306_WIDTH) break;
        for (bit = 0; bit < 8; bit++)
        {
            if (font_5x7[idx][i] & (1 << bit))
                s_buffer[page][s_cursor_x + i] |= (1 << bit);
            else
                s_buffer[page][s_cursor_x + i] &= ~(1 << bit);
        }
    }

    /* 列间距 */
    if (s_cursor_x + 5 < SSD1306_WIDTH)
    {
        s_buffer[page][s_cursor_x + 5] = 0x00;
    }

    s_cursor_x += 6;
    if (s_cursor_x >= SSD1306_WIDTH)
    {
        s_cursor_x = 0;
        s_cursor_y++;
    }
}

void SSD1306_PutChar(char ch)
{
    SSD1306_DrawChar(ch);
}

void SSD1306_Print(uint8_t line, uint8_t col, const char *str)
{
    SSD1306_SetCursor(line, col);
    while (*str)
    {
        SSD1306_DrawChar(*str++);
    }
}

void SSD1306_printf(uint8_t line, uint8_t col, const char *fmt, ...)
{
    char buf[SSD1306_CHARS_PER_LINE + 1];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    SSD1306_Print(line, col, buf);
}

void SSD1306_DrawBitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t *bmp)
{
    uint8_t page_start, page_end;
    uint8_t col;
    uint8_t byte_idx;
    uint8_t bit_in_byte;

    page_start = y / 8;
    page_end = (y + h + 7) / 8;
    if (page_end > SSD1306_PAGES) page_end = SSD1306_PAGES;

    for (uint8_t page = page_start; page < page_end; page++)
    {
        for (col = 0; col < w && (x + col) < SSD1306_WIDTH; col++)
        {
            uint8_t pixel_data = 0;
            for (bit_in_byte = 0; bit_in_byte < 8; bit_in_byte++)
            {
                uint16_t py = page * 8 + bit_in_byte;
                if (py >= y && py < y + h)
                {
                    byte_idx = (py - y) * ((w + 7) / 8) + col / 8;
                    if (bmp[byte_idx] & (1 << (7 - (col % 8))))
                        pixel_data |= (1 << bit_in_byte);
                }
            }
            s_buffer[page][x + col] = pixel_data;
        }
    }
}

void SSD1306_SetPixel(uint8_t x, uint8_t y, bool on)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    uint8_t page = y / 8;
    if (on)
        s_buffer[page][x] |= (1 << (y % 8));
    else
        s_buffer[page][x] &= ~(1 << (y % 8));
}

void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on)
{
    for (uint8_t dy = 0; dy < h; dy++)
        for (uint8_t dx = 0; dx < w; dx++)
            SSD1306_SetPixel(x + dx, y + dy, on);
}

void SSD1306_Update(void)
{
    /* 一页 = 129字节 (1控制字节 + 128像素数据) */
    uint8_t buf[129];

    if ((s_hi2c == NULL) || !s_ready)
    {
        return;
    }

    /* 获取 I2C 锁，整个更新过程持有锁 */
    if (s_i2c_lock != NULL) s_i2c_lock();

    buf[0] = 0x40;  /* 数据模式 */

    for (uint8_t page = 0; page < SSD1306_PAGES; page++)
    {
        /* 设置页地址和列地址 */
        uint8_t cmds[] = {
            0x00, 0xB0 | page,   /* 设置页号 0-7 */
            0x00, 0x00,          /* 列地址低4位 = 0 */
            0x00, 0x10           /* 列地址高4位 = 0 */
        };
        if (HAL_I2C_Master_Transmit(s_hi2c, s_i2c_addr, cmds, 6, 100) != HAL_OK)
        {
            s_ready = false;
            if (s_i2c_unlock != NULL) s_i2c_unlock();
            return;
        }

        /* 写入整页 128 字节，列地址自动递增 */
        memcpy(&buf[1], s_buffer[page], SSD1306_WIDTH);
        if (HAL_I2C_Master_Transmit(s_hi2c, s_i2c_addr, buf, 129, 100) != HAL_OK)
        {
            s_ready = false;
            if (s_i2c_unlock != NULL) s_i2c_unlock();
            return;
        }
    }

    if (s_i2c_unlock != NULL) s_i2c_unlock();
}
