#ifndef TASK_H
#define TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TASK_OFF = 0,
    TASK_MAIN,    /* 题目2：0cm 稳定 */
    TASK_GO_P5,   /* 题目1阶段1：到 +5cm */
    TASK_GO_N5,   /* 题目1阶段2：到 -5cm 并保持 */
} Task_State;

typedef struct {
    /* 下列字段被 key.c 的 GPIO 中断（GROUP1_IRQHandler → Task_key1/2）写入、
       被 empty.c 主循环读取，故标 volatile，避免单帧多处读取读到陈旧/撕裂值。 */
    volatile Task_State state;
    volatile uint16_t target_x;        /* 当前任务目标（px）*/
    volatile uint8_t reach_count;      /* 连续到达帧计数 */
    volatile uint32_t phase_start_ms;  /* 当前阶段计时起点（惰性启动，见 phase_started）*/
    volatile bool phase_started;       /* 当前阶段计时是否已启动（区分“未启动”与 now_ms==0）*/
} Task_Context;

/* 全局任务实例：在 task.c 定义；key.c（GPIO 中断）与 empty.c（主循环）共享，
   勿在其它文件重复 extern，统一引用本声明。 */
extern Task_Context g_task;

/* 坐标标定 */
#define TASK_X_0CM    92U
#define TASK_X_P5CM   142U
#define TASK_X_N5CM   42U

void Task_init(Task_Context *ctx);
void Task_key1(Task_Context *ctx);          /* 启停 0cm 闭环 */
void Task_key2(Task_Context *ctx);          /* 启动/退出题目1流程 */
void Task_update(Task_Context *ctx, bool has_valid_x, uint16_t ball_x,
                 uint32_t now_ms);          /* 每帧调用，处理阶段切换 */

#endif
