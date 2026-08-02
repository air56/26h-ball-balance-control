#ifndef MOTOR_PARSER_H
#define MOTOR_PARSER_H

#include <stdbool.h>
#include <stdint.h>

void MotorParser_reset(void);
void MotorParser_processByte(uint8_t byte);
/* 返回整数度角度（含符号）。成功返回 true。 */
bool MotorParser_getAngleDegrees(int32_t *angle);
/* 返回 0.1° 精度的带符号位置（位置×360/65536×10）。 */
bool MotorParser_getPosX10(int32_t *pos_x10);

#endif
