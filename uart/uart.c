#include "uart.h"
#include "../camera_uart.h"
#include "../motor_parser.h"

void UART_send_char(UART_Regs *uart,const uint8_t data)
{
    DL_UART_transmitDataBlocking(uart, data);
}

void UART_send_string(UART_Regs *uart, const char *str)
{
    while (*str)
    {
        UART_send_char(uart,(uint8_t)  *str);
        str++;
    }
}

void UART_send_bytes(UART_Regs *uart, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        UART_send_char(uart, data[i]);
// #ifdef DEBUG_INST
//         if (uart != DEBUG_INST) {
//             UART_send_char(DEBUG_INST, data[i]);
//         }
// #endif
    }
}

/* 将字节数组以HEX格式发送到调试串口，格式: "01 F3 AB ..." */
void UART_debug_dump(UART_Regs *debug_uart, const uint8_t *data, uint16_t len)
{
    static const char hex[] = "0123456789ABCDEF";
    for (uint16_t i = 0; i < len; i++) {
        UART_send_char(debug_uart, hex[data[i] >> 4]);
        UART_send_char(debug_uart, hex[data[i] & 0x0F]);
        UART_send_char(debug_uart, ' ');
    }
    UART_send_string(debug_uart, "\r\n");
}

/* UART0 接收中断：读取电机应答帧，喂给 motor_parser 解析实时位置，
   同时防止 RX FIFO 溢出触发空闲帧中断 */
void UART0_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(PC_INST) == DL_UART_IIDX_RX) {
        while (!DL_UART_isRXFIFOEmpty(PC_INST)) {
            MotorParser_processByte(DL_UART_receiveData(PC_INST));
        }
        return;
    }

    switch (DL_UART_getPendingInterrupt(PC_INST))
    {
        case DL_UART_IIDX_RX:
            MotorParser_processByte(DL_UART_receiveData(PC_INST));
            break;
        default:
            break;
    }
}

void UART1_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(CAMERA_INST) == DL_UART_IIDX_RX) {
        while (!DL_UART_isRXFIFOEmpty(CAMERA_INST)) {
            uint8_t byte = DL_UART_receiveData(CAMERA_INST);

            CameraUart_processByte(byte);
        }
    }
}
