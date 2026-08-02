#ifndef CAMERA_UART_H
#define CAMERA_UART_H

#include <stdbool.h>
#include <stdint.h>

void CameraUart_reset(void);
void CameraUart_processByte(uint8_t byte);
bool CameraUart_getLatestX(uint16_t *x);
bool CameraUart_takeLatestX(uint16_t *x);
/* 诊断接口：UART1 收到字节数 / 解析有效帧数 / 最近原始字节 */
bool CameraUart_getRxByteCount(uint32_t *count);
bool CameraUart_getRxFrameCount(uint32_t *count);
bool CameraUart_getRxLastByte(uint8_t *byte);

#endif
