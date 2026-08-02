#include "oled.h"
#include "oledfont.h"
#include "delay.h"

/* 软件 SPI 写一个字节 */
static void SPI_WriteByte(uint8_t dat)
{
    for (uint8_t i = 0; i < 8; i++) {
        OLED_SCLK_Clr();
        if (dat & 0x80)
            OLED_MOSI_Set();
        else
            OLED_MOSI_Clr();
        dat <<= 1;
        OLED_SCLK_Set();
    }
    OLED_SCLK_Clr();
}

void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    OLED_CS_Clr();
    if (cmd) OLED_DC_Set();
    else     OLED_DC_Clr();
    SPI_WriteByte(dat);
    OLED_CS_Set();
}

void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte(x & 0x0f, OLED_CMD);
}

void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++) {
        OLED_WR_Byte(0xb0 + i, OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(0, OLED_DATA);
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey)
{
    uint8_t c = chr - ' ';
    uint8_t sizex = sizey / 2;
    uint16_t i, size1;
    if (sizey == 8) size1 = 6;
    else size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * (sizey / 2);

    OLED_Set_Pos(x, y);
    for (i = 0; i < size1; i++) {
        if (i % sizex == 0 && sizey != 8) OLED_Set_Pos(x, y++);
        if (sizey == 8) OLED_WR_Byte(asc2_0806[c][i], OLED_DATA);
        else if (sizey == 16) OLED_WR_Byte(asc2_1608[c][i], OLED_DATA);
    }
}

uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey)
{
    uint8_t t, temp, m = 0;
    uint8_t enshow = 0;
    if (sizey == 8) m = 2;
    for (t = 0; t < len; t++) {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                OLED_ShowChar(x + (sizey / 2 + m) * t, y, ' ', sizey);
                continue;
            } else enshow = 1;
        }
        OLED_ShowChar(x + (sizey / 2 + m) * t, y, temp + '0', sizey);
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t sizey)
{
    uint8_t j = 0;
    while (chr[j] != '\0') {
        OLED_ShowChar(x, y, chr[j++], sizey);
        if (sizey == 8) x += 6;
        else x += sizey / 2;
    }
}

void OLED_Init(void)
{
    /* SCLK(PB9)/MOSI(PB8) 从 SPI 外设切回 GPIO 输出 */
    DL_GPIO_initDigitalOutput(OLED_SCK_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_SDA_IOMUX);
    DL_GPIO_enableOutput(GPIOB, OLED_SCK_PIN | OLED_SDA_PIN);

    /* 初始电平 */
    OLED_CS_Set();
    OLED_SCLK_Set();
    OLED_MOSI_Set();

    /* 硬件复位 */
    OLED_RES_Clr();
    delay_ms(200);
    OLED_RES_Set();

    /* SSD1306 初始化序列 */
    OLED_WR_Byte(0xAE, OLED_CMD); /* display off */
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD);
    OLED_WR_Byte(0xCF, OLED_CMD);
    OLED_WR_Byte(0xA1, OLED_CMD);
    OLED_WR_Byte(0xC8, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD);
    OLED_WR_Byte(0x3F, OLED_CMD);
    OLED_WR_Byte(0xD3, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0xD5, OLED_CMD);
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD);
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDA, OLED_CMD);
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD);
    OLED_WR_Byte(0x02, OLED_CMD);
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xA4, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD); /* display on */
}
