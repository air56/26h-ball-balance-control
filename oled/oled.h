#ifndef __OLED_SPI_H
#define __OLED_SPI_H

#include "ti_msp_dl_config.h"

#define OLED_CMD  0
#define OLED_DATA 1

/* 引脚宏 */
#define OLED_SCLK_Set()  DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_9)    /* PB9 */
#define OLED_SCLK_Clr()  DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_9)
#define OLED_MOSI_Set()  DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_8)    /* PB8 */
#define OLED_MOSI_Clr()  DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_8)
#define OLED_RES_Set()   DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_10)   /* PB10 */
#define OLED_RES_Clr()   DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_10)
#define OLED_DC_Set()    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_11)   /* PB11 */
#define OLED_DC_Clr()    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_11)
#define OLED_CS_Set()    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14)   /* PB14 */
#define OLED_CS_Clr()    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_14)

void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey);
uint32_t oled_pow(uint8_t m, uint8_t n);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t sizey);
void OLED_Init(void);

#endif
