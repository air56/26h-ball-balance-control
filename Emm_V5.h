#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/**********************************************************
*** Emm_V5.0 闭环步进电机控制
*** 移植自张大头闭环伺服示例代码
*** 适配: TI MSPM0G3507 + DriverLib
*** UART 波特率: 115200
*** 校验码: 固定 0x6B
**********************************************************/

#define ABS(x) ((x) > 0 ? (x) : -(x))

typedef enum {
    S_VER   = 0,  /* 读取固件版本和对应的硬件版本 */
    S_RL    = 1,  /* 读取相电阻和相电感 */
    S_PID   = 2,  /* 读取PID参数 */
    S_VBUS  = 3,  /* 读取总线电压 */
    S_CPHA  = 5,  /* 读取相电流 */
    S_ENCL  = 7,  /* 读取编码器值 */
    S_TPOS  = 8,  /* 读取目标位置角度 */
    S_VEL   = 9,  /* 读取实时转速 */
    S_CPOS  = 10, /* 读取实时位置角度 */
    S_PERR  = 11, /* 读取位置误差角度 */
    S_FLAG  = 13, /* 读取使能/到位/堵转状态标志位 */
    S_Conf  = 14, /* 读取驱动参数 */
    S_State = 15, /* 读取系统状态参数 */
    S_ORG   = 16, /* 读取回零状态标志位 */
} SysParams_t;

/* 方向 */
#define MOTOR_DIR_CW   0
#define MOTOR_DIR_CCW  1

/* 使能状态 */
#define MOTOR_DISABLE  0
#define MOTOR_ENABLE   1

/* 同步标志 */
#define MOTOR_SYNC_NOW    0
#define MOTOR_SYNC_CACHE  1

/* 电机地址默认值 */
#define MOTOR_ADDR_DEFAULT  1

/* 控制模式 */
#define CTRL_MODE_OFF      0
#define CTRL_MODE_OPEN     1
#define CTRL_MODE_CLOSED   2
#define CTRL_MODE_PULSE    3

/* 运动模式（位置模式） */
#define POS_MODE_RELATIVE_TARGET  0
#define POS_MODE_ABSOLUTE         1
#define POS_MODE_RELATIVE_CURRENT 2

void Emm_V5_Reset_CurPos_To_Zero(UART_Regs *uart, uint8_t addr);
void Emm_V5_Reset_Clog_Pro(UART_Regs *uart, uint8_t addr);
void Emm_V5_Modify_Ctrl_Mode(UART_Regs *uart, uint8_t addr, uint8_t svF, uint8_t ctrl_mode);
void Emm_V5_En_Control(UART_Regs *uart, uint8_t addr, uint8_t state, uint8_t snF);
void Emm_V5_Vel_Control(UART_Regs *uart, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t snF);
void Emm_V5_Pos_Control(UART_Regs *uart, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, uint8_t snF);
void Emm_V5_Stop_Now(UART_Regs *uart, uint8_t addr, uint8_t snF);
void Emm_V5_Synchronous_motion(UART_Regs *uart, uint8_t addr);
void Emm_V5_Origin_Set_O(UART_Regs *uart, uint8_t addr, uint8_t svF);
void Emm_V5_Origin_Modify_Params(UART_Regs *uart, uint8_t addr, uint8_t svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, uint8_t potF);
void Emm_V5_Origin_Trigger_Return(UART_Regs *uart, uint8_t addr, uint8_t o_mode, uint8_t snF);
void Emm_V5_Origin_Interrupt(UART_Regs *uart, uint8_t addr);

/* 读取实时位置（01 36 6B），应答由 motor_parser 解析 */
void Emm_V5_QueryCurPos(UART_Regs *uart, uint8_t addr);

#endif
