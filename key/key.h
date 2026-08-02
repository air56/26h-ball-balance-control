#ifndef KEY_H
#define KEY_H

#include "ti_msp_dl_config.h"

/* 按键初始化：使能 GPIOA NVIC 中断（在 SYSCFG_DL_init() 之后调用）*/
void Key_init(void);

/* 按键扫描（主循环每 20ms 调用）：软件消抖，稳定后触发 Task_key1/2 */
void Key_scan(uint32_t now_ms);

uint8_t get_key_value(uint32_t  key);


#endif
