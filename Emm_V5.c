#include "Emm_V5.h"
#include "uart.h"

/**********************************************************
*** Emm_V5.0 闭环步进电机控制
*** 移植自张大头闭环伺服示例代码
*** 适配: TI MSPM0G3507 + DriverLib
**********************************************************/

/* 将当前位置清零 */
void Emm_V5_Reset_CurPos_To_Zero(UART_Regs *uart, uint8_t addr)
{
    uint8_t cmd[] = { addr, 0x0A, 0x6D, 0x6B };
    UART_send_bytes(uart, cmd, sizeof(cmd));
}

/* 解除堵转保护 */
void Emm_V5_Reset_Clog_Pro(UART_Regs *uart, uint8_t addr)
{
    uint8_t cmd[] = { addr, 0x0E, 0x52, 0x6B };
    UART_send_bytes(uart, cmd, sizeof(cmd));
}

/* 修改控制模式 */
void Emm_V5_Modify_Ctrl_Mode(UART_Regs *uart, uint8_t addr, uint8_t svF, uint8_t ctrl_mode)
{
    uint8_t cmd[] = { addr, 0x46, 0x69, svF, ctrl_mode, 0x6B };
    UART_send_bytes(uart, cmd, sizeof(cmd));
}

/* 电机使能控制 */
void Emm_V5_En_Control(UART_Regs *uart, uint8_t addr, uint8_t state, uint8_t snF)
{
    uint8_t cmd[] = { addr, 0xF3, 0xAB, state, snF, 0x6B };
    UART_send_bytes(uart, cmd, sizeof(cmd));
}

/* 速度模式控制 */
void Emm_V5_Vel_Control(UART_Regs *uart, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t snF)
{
    uint8_t cmd[] = {
        addr,
        0xF6,
        dir,
        (uint8_t)(vel >> 8), (uint8_t)(vel & 0xFF),
        acc,
        snF,
        0x6B
    };
    UART_send_bytes(uart, cmd, sizeof(cmd));
}

/* 位置模式控制 */
void Emm_V5_Pos_Control(UART_Regs *uart, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, uint8_t snF)
{
    uint8_t cmd[] = {
        addr,
        0xFD,
        dir,
        (uint8_t)(vel >> 8), (uint8_t)(vel & 0xFF),
        acc,
        (uint8_t)(clk >> 24), (uint8_t)(clk >> 16),
        (uint8_t)(clk >>  8), (uint8_t)(clk & 0xFF),
        raF,
        snF,
        0x6B
    };
    UART_send_bytes(uart, cmd, sizeof(cmd));
}

/* 立即停止 */
void Emm_V5_Stop_Now(UART_Regs *uart, uint8_t addr, uint8_t snF)
{
    uint8_t cmd[] = { addr, 0xFE, 0x98, snF, 0x6B };
    UART_send_bytes(uart, cmd, sizeof(cmd));
}

/* 触发多机同步运动 */
void Emm_V5_Synchronous_motion(UART_Regs *uart, uint8_t addr)
{
    uint8_t cmd[] = { addr, 0xFF, 0x66, 0x6B };
    UART_send_bytes(uart, cmd, sizeof(cmd));
}

/* 设置回零零点 */
void Emm_V5_Origin_Set_O(UART_Regs *uart, uint8_t addr, uint8_t svF)
{
    uint8_t cmd[] = { addr, 0x93, 0x88, svF, 0x6B };
    UART_send_bytes(uart, cmd, sizeof(cmd));
}

/* 修改回零参数 */
void Emm_V5_Origin_Modify_Params(UART_Regs *uart, uint8_t addr, uint8_t svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, uint8_t potF)
{
    uint8_t cmd[] = {
        addr,
        0x4C, 0xAE,
        svF,
        o_mode,
        o_dir,
        (uint8_t)(o_vel >> 8), (uint8_t)(o_vel & 0xFF),
        (uint8_t)(o_tm >> 24), (uint8_t)(o_tm >> 16),
        (uint8_t)(o_tm >>  8), (uint8_t)(o_tm & 0xFF),
        (uint8_t)(sl_vel >> 8), (uint8_t)(sl_vel & 0xFF),
        (uint8_t)(sl_ma >> 8), (uint8_t)(sl_ma & 0xFF),
        (uint8_t)(sl_ms >> 8), (uint8_t)(sl_ms & 0xFF),
        potF,
        0x6B
    };
    UART_send_bytes(uart, cmd, sizeof(cmd));
}

/* 触发回零 */
void Emm_V5_Origin_Trigger_Return(UART_Regs *uart, uint8_t addr, uint8_t o_mode, uint8_t snF)
{
    uint8_t cmd[] = { addr, 0x9A, o_mode, snF, 0x6B };
    UART_send_bytes(uart, cmd, sizeof(cmd));
}

/* 强制中断回零 */
void Emm_V5_Origin_Interrupt(UART_Regs *uart, uint8_t addr)
{
    uint8_t cmd[] = { addr, 0x9C, 0x48, 0x6B };
    UART_send_bytes(uart, cmd, sizeof(cmd));
}

/* 读取实时位置角度（01 36 6B），应答帧在 UART0 中断由 motor_parser 解析 */
void Emm_V5_QueryCurPos(UART_Regs *uart, uint8_t addr)
{
    uint8_t cmd[] = { addr, 0x36, 0x6B };
    UART_send_bytes(uart, cmd, sizeof(cmd));
}
