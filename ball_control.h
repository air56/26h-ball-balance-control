#ifndef BALL_CONTROL_H
#define BALL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BALL_CONTROL_DIR_CW = 0,
    BALL_CONTROL_DIR_CCW = 1
} BallControl_Direction;

typedef struct {
    BallControl_Direction direction;
    uint16_t rpm;
} BallControl_Command;

typedef struct {
    uint16_t target_x;          /* 目标像素坐标（任务状态机设定）*/
    uint16_t kp_rpm_per_px;     /* 位置比例系数（RPM/px）*/
    uint16_t kd_rpm_per_px;     /* 速度微分系数（RPM/(px/帧)）*/
    uint16_t right_kd_rpm_per_px; /* 球右移（x 增大）时速度微分系数，0 = 用 kd_rpm_per_px */
    uint16_t left_kd_rpm_per_px;  /* 球左移（x 减小）时速度微分系数，0 = 用 kd_rpm_per_px */
    uint16_t deadband_px;       /* 死区（px）*/
    uint16_t max_rpm;           /* 最大转速（RPM）*/
    uint16_t right_max_rpm;     /* 小球右移（x 增大）时最大转速（RPM），0 = 用 max_rpm */
    uint16_t left_max_rpm;      /* 小球左移（x 减小）时最大转速（RPM），0 = 用 max_rpm */
    uint16_t rpm_step;          /* 每帧最大转速增量（RPM）*/
    uint16_t motion_jitter_px;  /* 抖动忽略阈值（px）*/
    uint16_t ki_scaled;         /* 积分系数，放大1000倍存储：ki = ki_scaled/1000。
                                   默认 10 = 0.01，饱和 500px 时贡献 5 RPM。
                                   对 RPM 输出的贡献 = (integral*ki_scaled+500)/1000 */
    uint16_t integral_max;      /* 积分限幅（px）*/
    uint16_t integral_interval; /* 积分累加周期（帧数，参考代码30）*/
    uint16_t predict_px;        /* 预测超前量（px）：小球速度 × predict_px 帧 的预测位移。
                                   相当于把"当前偏差"外推到 predict_px 帧后，提前调节。
                                   0 = 关闭预测。 */
} BallControl_Config;

typedef struct {
    bool has_valid_sample;
    bool has_previous_sample;
    uint8_t valid_sample_count;
    uint16_t previous_ball_x;
    uint32_t last_valid_sample_ms;
    BallControl_Command command;
    int32_t integral;           /* 积分累加值（px）*/
    uint16_t integral_frame_cnt; /* 距上次积分累加的帧数 */
    int32_t last_x_delta;       /* 最近一帧位移（px/帧），用于预测 */
} BallControl_State;

void BallControl_init(BallControl_State *state, BallControl_Config *config);
BallControl_Command BallControl_update(BallControl_State *state,
                                       bool has_new_sample,
                                       uint16_t ball_x,
                                       uint32_t now_ms);
void BallControl_setTarget(BallControl_State *state, uint16_t target_x);
void BallControl_resetIntegral(BallControl_State *state);

#endif
