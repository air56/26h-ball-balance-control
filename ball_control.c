#include "ball_control.h"

#include <stddef.h>

#define BALL_X_MIN                   23U
#define BALL_X_MAX                  212U
#define BALL_START_SAMPLE_COUNT       3U
#define BALL_CAMERA_TIMEOUT_MS     100U

static BallControl_Config BALL_CONTROL_DEFAULT_CONFIG = {
    .target_x = 92U,
    .kp_rpm_per_px = 2U,
    .kd_rpm_per_px = 5U,
    .right_kd_rpm_per_px = 0U,   /* 0 = 使用 kd_rpm_per_px */
    .left_kd_rpm_per_px = 0U,    /* 0 = 使用 kd_rpm_per_px */
    .deadband_px = 4U,
    .max_rpm = 35U,
    .right_max_rpm = 0U,    /* 0 = 使用 max_rpm */
    .left_max_rpm = 0U,     /* 0 = 使用 max_rpm */
    .rpm_step = 5U,
    .motion_jitter_px = 1U,
    /* ki_scaled = 10 即 ki = 0.01（与设计规格初始值一致）：
       持续偏差 20px 累积 160px 时贡献 (160*10+500)/1000 = 2 RPM；
       饱和 500px 时贡献 5 RPM。默认配置下积分项对输出有可观测影响。
       调参：静差消除太慢可增大；超调/摆动可减小。*/
    .ki_scaled = 10U,
    .integral_max = 500U,
    .integral_interval = 30U,
    .predict_px = 0U   /* 0 = 关闭预测 */
};

static BallControl_Config *g_config = NULL;

static BallControl_Command BallControl_stop(void)
{
    BallControl_Command command = {
        .direction = BALL_CONTROL_DIR_CW,
        .rpm = 0U
    };

    return command;
}

static bool BallControl_isValidX(uint16_t ball_x)
{
    return (ball_x >= BALL_X_MIN) && (ball_x <= BALL_X_MAX);
}

static void BallControl_clearIntegral(BallControl_State *state)
{
    state->integral = 0;
    state->integral_frame_cnt = 0U;
}

static void BallControl_integrate(BallControl_State *state, int32_t bias)
{
    /* 低频积分：每 integral_interval 帧累加一次（参考代码每 30 帧），
       以此抑制死区内积分快速推满的 windup 现象。调参事项。 */
    state->integral_frame_cnt++;
    if (state->integral_frame_cnt >= g_config->integral_interval) {
        state->integral_frame_cnt = 0;
        state->integral += bias;
        if (state->integral > (int32_t)g_config->integral_max)
            state->integral = g_config->integral_max;
        else if (state->integral < -(int32_t)g_config->integral_max)
            state->integral = -(int32_t)g_config->integral_max;
    }
}

static BallControl_Command BallControl_fromMotion(BallControl_State *state,
                                                  uint16_t ball_x,
                                                  int16_t x_delta)
{
    int32_t bias;
    int32_t predicted_bias;   /* 预测偏差：把当前偏差外推到 predict_px 帧后 */
    int64_t position_term;
    int64_t motion_term;
    int64_t integral_term;
    int64_t prediction_term;  /* 预测项：小球速度 × 预测系数 */
    int64_t control;
    uint32_t rpm;
    BallControl_Command command = BallControl_stop();

    bias = (int32_t)g_config->target_x - (int32_t)ball_x;

    BallControl_integrate(state, bias);

    if ((bias >= -(int32_t)g_config->deadband_px) &&
        (bias <= (int32_t)g_config->deadband_px)) {
        position_term = 0;
    } else {
        /* int64 中间量，避免 int32 乘积溢出（可调参数可能达 uint16 上限） */
        position_term = (int64_t)bias * (int32_t)g_config->kp_rpm_per_px;
    }

    if ((x_delta >= -(int32_t)g_config->motion_jitter_px) &&
        (x_delta <= (int32_t)g_config->motion_jitter_px)) {
        motion_term = 0;
    } else {
        /* 按小球运动方向选择速度阻尼系数：
           x_delta > 0 → 球右移，用 right_kd_rpm_per_px（0 = 用 kd_rpm_per_px）
           x_delta < 0 → 球左移，用 left_kd_rpm_per_px（0 = 用 kd_rpm_per_px） */
        int32_t kd_use = g_config->kd_rpm_per_px;
        if (x_delta > 0 && g_config->right_kd_rpm_per_px != 0U) {
            kd_use = g_config->right_kd_rpm_per_px;
        } else if (x_delta < 0 && g_config->left_kd_rpm_per_px != 0U) {
            kd_use = g_config->left_kd_rpm_per_px;
        }
        /* int64 中间量，避免 int32 乘积溢出 */
        motion_term = -((int64_t)x_delta * (int32_t)kd_use);
    }

    /* 预测项（提前调节核心）：按小球当前速度外推 predict_px 帧后的位置，
       只保留"外推带来的增量"（-速度×预测帧数），避免与 position_term 重复。
       推导：bias=target-x；球右移时 x_delta>0，未来位置 = x + xd*pp（更右），
       未来偏差 = target-(x+xd*pp) = bias - xd*pp，增量 = (bias-xd*pp) - bias
       = -xd*pp。故 prediction_term = -xd*pp*kp，是标准的"速度前馈阻尼"。
       球右移(xd>0)且已在目标右侧(bias<0)时该项为负 → 提前加强反向推回，
       与 P 项、D 项同号，避免"越动越往原方向推"的正反馈。
       predict_px=0 时此项关闭，退化为纯 PD。
       【符号修复】此前实现：
         predicted_bias = bias + xd*pp            （速度项符号反了）
         prediction_term = (predicted_bias-bias)*kp = +xd*pp*kp
       在球正向偏差一侧移动时与 P/D 反号（正反馈）：如球在右侧(bias<0)
       继续右移(xd>0)时增量 >0，输出变成往右推 —— 这是"上电右滑过快"根因。
       改为 predicted_bias = bias - xd*pp 后增量 = -xd*pp*kp 与阻尼同号。 */
    if (g_config->predict_px != 0U) {
        predicted_bias = bias - (int32_t)x_delta * (int32_t)g_config->predict_px;
        prediction_term = (int64_t)(predicted_bias - bias) *
                          (int32_t)g_config->kp_rpm_per_px;
    } else {
        prediction_term = 0;
    }

    /* 积分项：(integral * ki_scaled + 500) / 1000 四舍五入到 RPM。
       int64 中间量避免 int32 溢出；+500 保证默认小 ki 下仍有非零贡献。 */
    integral_term = ((int64_t)state->integral *
                     (int32_t)g_config->ki_scaled + 500) / 1000;

    control = position_term + motion_term + integral_term + prediction_term;
    if (control == 0) {
        return command;
    }

    if (control > 0) {
        command.direction = BALL_CONTROL_DIR_CW;
        rpm = (uint32_t)control;
    } else {
        command.direction = BALL_CONTROL_DIR_CCW;
        rpm = (uint32_t)(-control);
    }

    /* 按小球运动方向选择最大转速：
       x_delta > 0 → 球右移，用 right_max_rpm（0 = 用 max_rpm）
       x_delta < 0 → 球左移，用 left_max_rpm（0 = 用 max_rpm）
       首帧 x_delta=0 时用 max_rpm。 */
    if (x_delta > 0 && g_config->right_max_rpm != 0U) {
        if (rpm > g_config->right_max_rpm) {
            rpm = g_config->right_max_rpm;
        }
    } else if (x_delta < 0 && g_config->left_max_rpm != 0U) {
        if (rpm > g_config->left_max_rpm) {
            rpm = g_config->left_max_rpm;
        }
    } else if (rpm > g_config->max_rpm) {
        rpm = g_config->max_rpm;
    }
    command.rpm = (uint16_t)rpm;

    return command;
}

void BallControl_init(BallControl_State *state, BallControl_Config *config)
{
    if (config == NULL) {
        g_config = &BALL_CONTROL_DEFAULT_CONFIG;
    } else {
        g_config = config;
    }

    state->has_valid_sample = false;
    state->has_previous_sample = false;
    state->valid_sample_count = 0U;
    state->previous_ball_x = 0U;
    state->last_valid_sample_ms = 0U;
    state->command = BallControl_stop();
    state->integral = 0;
    state->integral_frame_cnt = 0U;
    state->last_x_delta = 0;
}

/* 设计说明（问题4/5）：
   - setTarget 忽略 state 参数，直接改写 g_config（指向调用方传入的 config，
     或未传时指向 BALL_CONTROL_DEFAULT_CONFIG）。单实例使用无危害。
   - 由于默认配置非 const，setTarget 会"改写默认配置"；当前调用方总是先
     BallControl_init(state, &ball_config) 传入自有 config，不会触达默认值。 */
void BallControl_setTarget(BallControl_State *state, uint16_t target_x)
{
    (void)state;
    if (g_config != NULL) {
        g_config->target_x = target_x;
    }
}

BallControl_Command BallControl_update(BallControl_State *state,
                                       bool has_new_sample,
                                       uint16_t ball_x,
                                       uint32_t now_ms)
{
    if (has_new_sample && BallControl_isValidX(ball_x)) {
        int16_t x_delta = 0;

        if (state->has_previous_sample) {
            x_delta = (int16_t)((int32_t)ball_x -
                                (int32_t)state->previous_ball_x);
        }
        state->previous_ball_x = ball_x;
        state->has_previous_sample = true;
        state->has_valid_sample = true;
        state->last_valid_sample_ms = now_ms;
        if (state->valid_sample_count < BALL_START_SAMPLE_COUNT) {
            state->valid_sample_count++;
        }

        if (state->valid_sample_count >= BALL_START_SAMPLE_COUNT) {
            BallControl_Command target = BallControl_fromMotion(state,
                                                                ball_x,
                                                                x_delta);

            if (target.rpm == 0U) {
                state->command = target;
            } else if (state->command.rpm > 0U &&
                       state->command.direction != target.direction) {
                state->command = BallControl_stop();
            } else if (target.rpm > state->command.rpm +
                                     (uint16_t)g_config->rpm_step) {
                state->command.direction = target.direction;
                state->command.rpm += g_config->rpm_step;
            } else {
                state->command = target;
            }
        } else {
            state->command = BallControl_stop();
        }
    } else if (has_new_sample) {
        state->valid_sample_count = 0U;
        state->has_previous_sample = false;
        state->command = BallControl_stop();
    }

    if (!state->has_valid_sample ||
        (uint32_t)(now_ms - state->last_valid_sample_ms) >=
            BALL_CAMERA_TIMEOUT_MS) {
        state->command = BallControl_stop();
        state->valid_sample_count = 0U;
        state->has_previous_sample = false;
        BallControl_clearIntegral(state);
    }

    return state->command;
}

void BallControl_resetIntegral(BallControl_State *state)
{
    BallControl_clearIntegral(state);
}
