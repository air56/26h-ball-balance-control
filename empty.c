/* 闭环主循环：摄像头 -> ball_control -> Emm_V5 电机，OLED 三行显示。
 * 按键 key1/key2 由 key.c 的 GPIOA 中断分发到 Task_key1/Task_key2（操作 g_task），
 * 主循环每 20ms 读取 g_task 目标并驱动电机。
 * 坐标标定：0cm = 92px（TASK_X_0CM），有效范围 23~212。 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "Emm_V5.h"
#include "oled.h"
#include "ball_control.h"
#include "camera_uart.h"
#include "motor_parser.h"
#include "task.h"
#include "key.h"

#define MOTOR_ADDR       1U
#define MOTOR_ACCEL     10U   /* 加速度档位（0~255，越大启动越快）*/
/* 位置模式每周期脉冲上限（限步数核心）：320 脉冲 ≈ 1/10 圈（3200=1圈）。
   球右移等大偏差时，每 100ms 电机最多转这么多脉冲就停，防止俯冲。
   调参：想更温和减小（如 160），响应更快增大（如 640）。 */
#define MOTOR_PULSES_STEP  40U

/* 注意：g_task 全局实例在 task.c 定义，extern 声明收敛在 task.h（见上方 include）*/

static BallControl_Config ball_config;
static BallControl_State ball_state;

int main(void)
{
    SYSCFG_DL_init();
    NVIC_EnableIRQ(PC_INST_INT_IRQN);      /* 电机 UART 应答 */
    NVIC_EnableIRQ(CAMERA_INST_INT_IRQN);  /* maixcam 帧 */
    Key_init();                            /* 按键 GPIOA NVIC */

    OLED_Init();
    OLED_Clear();

    delay_ms(2000U);
    Emm_V5_En_Control(PC_INST, MOTOR_ADDR, MOTOR_ENABLE, MOTOR_SYNC_NOW);
    delay_ms(1000U);

    /* 球控制配置：目标默认 0cm=92px
       调参要点（解决"上电右滑过快"）：
       - predict_px 符号修复后（见 ball_control.c）预测项变成"速度前馈阻尼"，
         与 P/D 同号，不再正反馈；从 10 降到 4 使起滑瞬间的提前修正温和。
       - right_max_rpm 300→120：球右移被严重限速，即使起滑也只在慢速下
         被阻尼压回，不再直接冲向管壁。
       - right_kd 300→400：右移阻尼加强，起滑瞬间的反向修正更强。
       - MOTOR_PULSES_STEP 90→40：每 100ms 的脉冲上限减半，物理动作更温和。 */
    ball_config.target_x = TASK_X_0CM;
    ball_config.kp_rpm_per_px = 1U;        //Kp  位置反馈（RPM/px）。注意：整数类型，1/2U 会被截断为 0 导致位置反馈失效！
    ball_config.kd_rpm_per_px = 100U;       //Kd   速度阻尼，增大时小球接近目标提前减速
    /* 按小球运动方向分别设速度阻尼：x 增大(右移)用 right_kd，x 减小(左移)用 left_kd。
       设 0 则回退到上面的 kd_rpm_per_px。 */
    ball_config.right_kd_rpm_per_px = 400U;  //球右移时Kd（0=用kd_rpm_per_px）
    ball_config.left_kd_rpm_per_px = 200U;   //球左移时Kd（0=用kd_rpm_per_px）
    ball_config.deadband_px = 2U;          //死区
    ball_config.max_rpm = 10U;             //速度最大值
    /* 按小球运动方向分别限速：x 增大(球右移)用 right_max_rpm，x 减小(球左移)用 left_max_rpm。
       设 0 则回退到上面的 max_rpm。 */
    ball_config.right_max_rpm = 60U;       //球右移时最大转速（0=用max_rpm）
    ball_config.left_max_rpm = 60U;        //球左移时最大转速（0=用max_rpm）
    ball_config.rpm_step = 1U;             //每帧转速增量
    ball_config.motion_jitter_px = 1U;
    ball_config.ki_scaled = 10U;           //积分
    ball_config.integral_max = 500U;
    ball_config.integral_interval = 20U;   //积分周期
    /* 预测超前（提前调节）：predict_px = 小球速度 × 预测帧数 的外推超前量。
       球右移/左移时，按当前速度提前预判位置并提前反向。
       0 = 关闭预测。修复符号后预测与 P 同号，量级接近 P×predict_px，故 4 较稳妥。 */
    ball_config.predict_px = 30U;
    BallControl_init(&ball_state, &ball_config);

    Task_init(&g_task);

    
    /* 合成时钟：now_ms 每帧 +20ms，仅近似真实时间（delay_ms(20) 忙等
       实际偏慢 5-10%，task 的 5000ms 超时兜底实际约 5.2-5.5s，属可接受
       的兜底性质）。如需精确计时，改用硬件定时器。 */
    uint32_t now_ms = 0U;
    uint32_t last_send_ms = 0U;
    uint32_t last_poll_ms = 0U;
    uint16_t last_rpm = 0xFFFFU;
    uint16_t last_dir = 0xFFU;
    uint16_t display_x = 0U;

    for (;;) {
        now_ms += 20U;

        /* 按键消抖扫描：中断置标志，这里做电平稳定确认 */
        Key_scan(now_ms);

        /* 消费摄像头新帧 */
        uint16_t ball_x = 0U;
        bool has_new = CameraUart_takeLatestX(&ball_x);
        /* 诊断：显示原始解析 X（不过滤范围），判断 maixcam 坐标空间。
           原逻辑：仅 23~212 内才更新 display_x，若 maixcam 发的是全图像
           坐标（如 320），display_x 永远 0。 */
        if (has_new) display_x = ball_x;

        /* 任务状态机设定目标 */
        Task_update(&g_task, has_new, ball_x, now_ms);
        BallControl_setTarget(&ball_state, g_task.target_x);

        /* 控制 */
        BallControl_Command cmd = BallControl_update(&ball_state, has_new,
                                                      ball_x, now_ms);

        /* 发送命令：位置模式限步数。cmd.rpm>0 表示需要转（PD 判定方向），
           每周期发相对位置命令，固定最多转 MOTOR_PULSES_STEP 脉冲后停住。
           rpm==0 或待机 → 停止。 */
        bool run = (g_task.state != TASK_OFF);
        if (cmd.rpm == 0U || !run) {
            if (last_rpm != 0U) {
                Emm_V5_Stop_Now(PC_INST, MOTOR_ADDR, MOTOR_SYNC_NOW);
                last_rpm = 0U;
            }
        } else if ((cmd.direction != last_dir) ||
                   (now_ms - last_send_ms) >= 100U ||
                   cmd.rpm != last_rpm) {
            /* 相对当前位置（raF=2），每周期转固定脉冲，方向由 PD 决定 */
            Emm_V5_Pos_Control(PC_INST, MOTOR_ADDR, cmd.direction,
                               cmd.rpm, MOTOR_ACCEL, MOTOR_PULSES_STEP,
                               POS_MODE_RELATIVE_CURRENT, MOTOR_SYNC_NOW);
            last_rpm = cmd.rpm;
            last_dir = cmd.direction;
            last_send_ms = now_ms;
        }

        /* 每 200ms 轮询电机位置并刷新 OLED */
        if ((now_ms - last_poll_ms) >= 200U) {
            last_poll_ms = now_ms;
            Emm_V5_QueryCurPos(PC_INST, MOTOR_ADDR);

            OLED_Clear();
            /* 行1: X 坐标 */
            OLED_ShowString(0, 0, (uint8_t *)"X:", 16);
            OLED_ShowNum(24, 0, display_x, 3, 16);  //maixcam传入偏移值
            /* 行2: 电机角度 */
            int32_t angle;
            OLED_ShowString(0, 2, (uint8_t *)"A:", 16);
            if (MotorParser_getAngleDegrees(&angle)) {
                if (angle < 0) {
                    OLED_ShowString(16, 2, (uint8_t *)"-", 16);
                    angle = -angle;
                } else {
                    OLED_ShowString(16, 2, (uint8_t *)" ", 16);
                }
                OLED_ShowNum(24, 2, (uint32_t)angle, 3, 16);
            }
            /* 行3: 指令 RPM + 方向。停机时 direction 可能残留 CW/rpm 非零，
               显示 'S' 表示实际停止，避免误读为旋转中。 */
            OLED_ShowString(0, 4, (uint8_t *)"R:", 16);
            OLED_ShowNum(24, 4, cmd.rpm, 3, 16);
            char dir_char = 'S';  //电机转速
            if (run && cmd.rpm != 0U) {
                dir_char = (cmd.direction == BALL_CONTROL_DIR_CW) ? 'C' : 'A';
            }
            OLED_ShowChar(80, 4, dir_char, 16);
            /* 行4: S=启动状态(key1/key2 任一启动闭环=1)，T=题目，H=最近原始字节(hex)。
               T:0=待机, T:1=题目1(+5cm→-5cm,key1), T:2=题目2(0cm稳定,key2)。 */
            uint8_t start_state = (g_task.state != TASK_OFF) ? 1U : 0U;
            uint8_t task_no;
            switch (g_task.state) {
            case TASK_MAIN:
                task_no = 2U;   /* 题目2：0cm 稳定 */
                break;
            case TASK_GO_P5:
            case TASK_GO_N5:
                task_no = 1U;   /* 题目1：+5cm→-5cm */
                break;
            default:
                task_no = 0U;
                break;
            }
            OLED_ShowString(0, 6, (uint8_t *)"S:", 16);
            OLED_ShowNum(24, 6, start_state, 1, 16);
            OLED_ShowString(40, 6, (uint8_t *)"T:", 16);
            OLED_ShowNum(64, 6, task_no, 1, 16);
            /* 诊断：最近收到字节的 hex 值（判断摄像头数据内容） */
            uint8_t diag_byte;
            if (CameraUart_getRxLastByte(&diag_byte)) {
                static const char hex_d[] = "0123456789ABCDEF";
                OLED_ShowString(80, 6, (uint8_t *)"H:", 16);
                OLED_ShowChar(104, 6, hex_d[(diag_byte >> 4) & 0x0F], 16);
                OLED_ShowChar(112, 6, hex_d[diag_byte & 0x0F], 16);
            }
        }

        delay_ms(20U);
    }
}
