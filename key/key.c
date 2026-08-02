#include "key.h"
#include "../task.h"   /* g_task 的 extern 声明收敛在此（定义在 task.c）*/

/* 按键防抖：中断只置"待处理标志"，主循环 Key_scan() 做电平稳定确认后再触发。
   按键按下接 GND（低有效），FALL 沿中断。

   防抖流程（Key_scan 每 20ms 调用一次）：
   1. 中断置 pending_keyN 标志（表示该键有按下事件待确认）
   2. Key_scan 读电平：若连续 KEY_DEBOUNCE_MS 内电平保持低（按下），确认触发
   3. 触发后清除 pending，等待下一次中断

   这样抖动（按下瞬间电平反复跳变）不会立刻改任务状态，
   只有电平稳定了才触发 Task_key1/2。 */

#define KEY_DEBOUNCE_MS      40U   /* 消抖窗口：连续 40ms 电平稳定才确认 */
#define KEY_SCAN_PERIOD_MS   20U   /* 主循环扫描周期 */

static volatile uint8_t key1_pending;   /* key1 有待确认的按下事件 */
static volatile uint8_t key2_pending;
static uint32_t key1_debounce_start;    /* key1 电平开始稳定的时刻 */
static uint32_t key2_debounce_start;
static uint8_t key1_level_ok;           /* key1 当前电平已确认低(按下) */
static uint8_t key2_level_ok;

/* 按键初始化：使能 GPIOA NVIC 中断（SysConfig 只配外设，不自动使能 NVIC）*/
void Key_init(void)
{
    key1_pending = 0U;
    key2_pending = 0U;
    key1_debounce_start = 0U;
    key2_debounce_start = 0U;
    key1_level_ok = 0U;
    key2_level_ok = 0U;
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

uint8_t get_key_value(uint32_t key)
{
    uint8_t value = DL_GPIO_readPins(KEY_PORT, key);
    if ((value & key) != 0) return 1; //按位与，保证返回值是0或1
    else return 0;
}

/* 单键消抖：读 key 引脚电平（KEY_*_PIN），稳定 KEY_DEBOUNCE_MS 后返回 1(按下) */
static uint8_t Key_debounce(uint32_t key_pin, volatile uint8_t *pending,
                            uint32_t *start_ms, uint8_t *level_ok,
                            uint32_t now_ms)
{
    uint8_t pressed;

    /* 无待确认事件则直接返回 */
    if (*pending == 0U) {
        return 0U;
    }

    /* 按键按到 GND，按下时引脚为低（读回 0） */
    pressed = (DL_GPIO_readPins(KEY_PORT, key_pin) & key_pin) ? 0U : 1U;

    if (pressed) {
        if (*level_ok == 0U) {
            *start_ms = now_ms;   /* 电平开始变低，记录起点 */
            *level_ok = 1U;
        } else if ((uint32_t)(now_ms - *start_ms) >= KEY_DEBOUNCE_MS) {
            /* 电平已连续稳定 KEY_DEBOUNCE_MS，确认按下 */
            *pending = 0U;
            *level_ok = 0U;
            return 1U;
        }
    } else {
        /* 电平回高（抖动或松手），重新计时 */
        *level_ok = 0U;
    }
    return 0U;
}

void Key_scan(uint32_t now_ms)
{
    /* key1 = 题目1（+5cm→-5cm），key2 = 题目2（0cm 稳定），分开启停 */
    if (Key_debounce(KEY_key1_PIN, &key1_pending, &key1_debounce_start,
                     &key1_level_ok, now_ms)) {
        Task_key2(&g_task);   /* 题目1：先到 +5cm 再到 -5cm 保持 */
    }
    if (Key_debounce(KEY_key2_PIN, &key2_pending, &key2_debounce_start,
                     &key2_level_ok, now_ms)) {
        Task_key1(&g_task);   /* 题目2：0cm 稳定 */
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(KEY_PORT))
    {
        case KEY_key1_IIDX:
            key1_pending = 1U;   /* 只置待确认标志，不在中断里改状态 */
            break;
        case KEY_key2_IIDX:
            key2_pending = 1U;
            break;
        default:
            break;
    }
}
