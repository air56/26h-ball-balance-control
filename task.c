#include "task.h"

/* 全局任务状态实例：key.c（GPIO 中断分发）与 empty.c（主循环）共用。
   extern 声明收敛在 task.h，勿在其它文件重复 extern。 */
Task_Context g_task;

#define TASK_REACH_WINDOW_PX    5U   /* ±5px 到达判定 */
#define TASK_REACH_FRAMES       3U   /* 连续 3 帧到达 */
#define TASK_PHASE_TIMEOUT_MS 5000U   /* 阶段超时兜底 */

/* 阶段计时统一走惰性启动：进入新阶段时不记录具体时刻，
   而是标记“未启动”，等首次 update 才写 phase_start_ms。
   避免用按键瞬间的 now_ms 或 0 时刻参与 32 位回绕比较，
   也避免“未启动”与 now_ms==0 混淆（见 phase_started）。 */
static void Task_resetPhaseTimer(Task_Context *ctx)
{
    ctx->phase_start_ms = 0U;
    ctx->phase_started = false;
}

void Task_init(Task_Context *ctx)
{
    ctx->state = TASK_OFF;
    ctx->target_x = TASK_X_0CM;
    ctx->reach_count = 0U;
    Task_resetPhaseTimer(ctx);
}

void Task_key1(Task_Context *ctx)
{
    if (ctx->state == TASK_OFF) {
        ctx->state = TASK_MAIN;      /* 启动 0cm 闭环 */
        ctx->target_x = TASK_X_0CM;
        Task_resetPhaseTimer(ctx);
    } else if (ctx->state == TASK_MAIN) {
        ctx->state = TASK_OFF;       /* 停止闭环 */
    }
    /* 题目1运行中按 key1 也停止回 OFF */
    else {
        ctx->state = TASK_OFF;
    }
    ctx->reach_count = 0U;
}

void Task_key2(Task_Context *ctx)
{
    if (ctx->state == TASK_OFF) {
        ctx->state = TASK_GO_P5;     /* 启动题目1：先到 +5cm */
        ctx->target_x = TASK_X_P5CM;
    } else {
        ctx->state = TASK_OFF;       /* 运行中按 key2 回待机 */
    }
    ctx->reach_count = 0U;
    Task_resetPhaseTimer(ctx);
}

static void Task_onReach(Task_Context *ctx, uint32_t now_ms)
{
    ctx->reach_count = 0U;
    ctx->phase_start_ms = now_ms;
    ctx->phase_started = true;
    if (ctx->state == TASK_GO_P5) {
        ctx->state = TASK_GO_N5;     /* 到达 +5cm，立即回 -5cm */
        ctx->target_x = TASK_X_N5CM;
    }
    /* TASK_GO_N5 是终态，保持 */
}

void Task_update(Task_Context *ctx, bool has_valid_x, uint16_t ball_x,
                 uint32_t now_ms)
{
    if (ctx->state == TASK_OFF) {
        return;
    }
    /* 阶段超时兜底：惰性挂钟——按键启动阶段时仅标记“未启动”，
       首次 update 才记录起始时刻，避免用 0 时刻或按键瞬间时刻
       参与 32 位回绕比较。GO_P5 阶段球始终未到达时，5s 超时
       触发 GO_P5 -> GO_N5。 */
    if (!ctx->phase_started) {
        ctx->phase_start_ms = now_ms;
        ctx->phase_started = true;
    } else if ((now_ms - ctx->phase_start_ms) >= TASK_PHASE_TIMEOUT_MS) {
        Task_onReach(ctx, now_ms);
        return;
    }
    if (!has_valid_x) {
        ctx->reach_count = 0U;
        return;
    }
    int32_t diff = (int32_t)ball_x - (int32_t)ctx->target_x;
    if (diff < 0) diff = -diff;
    if (diff <= (int32_t)TASK_REACH_WINDOW_PX) {
        ctx->reach_count++;
        if (ctx->reach_count >= TASK_REACH_FRAMES) {
            Task_onReach(ctx, now_ms);
        }
    } else {
        ctx->reach_count = 0U;
    }
}
