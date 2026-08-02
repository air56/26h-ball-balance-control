#include "motor_parser.h"

/*
 * 电机应答帧格式：[Addr][Code][Data...][0x6B]
 *
 * 读取实时位置命令 01 36 6B 的应答帧为固定长度：
 *   01 36 [符号1B] [位置4B大端] 6B
 *   位置为编码器计数，角度 = pos × 360 / 65536。
 *
 * 其它常见应答数据长度各不相同（使能 01 F3 02 6B 仅 1 字节数据、
 * 速度 01 F6 ... 6B、状态 01 3A [1B] 6B、版本 01 1F [2B][4B] 6B、
 * 完整系统状态 01 43 ... 6B 等），无法用单一固定长度解析。
 * 因此对非 0x36 应答采取「跳过数据区直到帧尾 0x6B」的策略，
 * 因为所有应答帧都以 0x6B 结尾。应答帧不用于角度显示，仅用于
 * 正确回到 WAIT_ADDR 以解析后续位置帧。若跳过期间出现坏帧（截断
 * 无 0x6B），残留字节会被下一个位置帧的 0x6B 一起消费（丢弃一帧），
 * 再下一帧即恢复——位置轮询频率高，丢失一帧无影响。
 */

typedef enum {
    MP_WAIT_ADDR,
    MP_WAIT_CODE,
    MP_WAIT_SIGN,
    MP_WAIT_POS3,
    MP_WAIT_POS2,
    MP_WAIT_POS1,
    MP_WAIT_POS0,
    MP_WAIT_TAIL,
    MP_WAIT_SKIP,      /* 非 0x36 应答：跳过数据区直到 0x6B */
} MPParserState;

#define MOTOR_ADDR  0x01U
#define CODE_POS    0x36U
#define FRAME_TAIL  0x6BU
/* 单圈编码器位置上限：65536 计数 = 一圈 360°。
   位置应答的 pos 超出单圈范围即视为坏帧残余（见 WAIT_TAIL 注释）。 */
#define POS_MAX_SINGLE_TURN  0x10000UL

static MPParserState parser_state;
static uint8_t parser_code;
static uint8_t parser_sign;
static uint32_t parser_pos;
static volatile int32_t parser_pos_x10;
static volatile bool parser_has_pos;

void MotorParser_reset(void)
{
    parser_state = MP_WAIT_ADDR;
    parser_code = 0;
    parser_sign = 0;
    parser_pos = 0;
    parser_has_pos = false;
}

static void MotorParser_storePosition(uint8_t sign, uint32_t pos)
{
    int32_t signed_pos;
    int64_t deg_x10;

    /* sign-magnitude 约定：sign=0 为正，sign=1 为负。
       负值时 -(int32_t)pos 在 pos ≥ 0x80000000 理论上对 INT32_MIN
       取负是未定义行为，但实际编码器计数远小于该边界，可忽略。 */
    if (sign != 0) {
        signed_pos = -(int32_t)pos;
    } else {
        signed_pos = (int32_t)pos;
    }
    /* pos × 360 / 65536，保留 0.1° → ×10 */
    deg_x10 = ((int64_t)signed_pos * 3600) / 65536;
    parser_pos_x10 = (int32_t)deg_x10;
    parser_has_pos = true;
}

void MotorParser_processByte(uint8_t byte)
{
    switch (parser_state) {
    case MP_WAIT_ADDR:
        if (byte == MOTOR_ADDR) {
            parser_state = MP_WAIT_CODE;
        }
        break;

    case MP_WAIT_CODE:
        parser_code = byte;
        if (byte == CODE_POS) {
            parser_state = MP_WAIT_SIGN;
        } else {
            /* 非位置应答：跳到跳过阶段，等帧尾 0x6B */
            parser_state = MP_WAIT_SKIP;
        }
        break;

    case MP_WAIT_SIGN:
        parser_sign = byte;
        parser_state = MP_WAIT_POS3;
        break;

    case MP_WAIT_POS3:
        parser_pos = (uint32_t)byte << 24;
        parser_state = MP_WAIT_POS2;
        break;

    case MP_WAIT_POS2:
        parser_pos |= (uint32_t)byte << 16;
        parser_state = MP_WAIT_POS1;
        break;

    case MP_WAIT_POS1:
        parser_pos |= (uint32_t)byte << 8;
        parser_state = MP_WAIT_POS0;
        break;

    case MP_WAIT_POS0:
        parser_pos |= (uint32_t)byte;
        parser_state = MP_WAIT_TAIL;
        break;

    case MP_WAIT_TAIL:
        /* 同时满足三项才认为是一帧完整的位置应答：
           ① 帧尾 0x6B；
           ② 帧头 code 是 0x36（防御性校验，WAIT_TAIL 实际只能由 0x36 帧体
              到达，此处 parser_code 恒为 0x36，仅作未来改动的保险）；
           ③ pos 在单圈 360° 范围内。③ 是真正起作用的守卫：
              截断的 0x36 帧（如已收 01 36 00 00 停在 POS2）会吞掉紧随的
              应答字节，使能应答 01 F3 02 6B 的 F3/02 被当作 pos 残余，
              以 sign=0、pos=0x0001F302 拼出约 701.7° 的假位置——超单圈，
              据此拒绝。 */
        if (byte == FRAME_TAIL && parser_code == CODE_POS &&
            parser_pos <= POS_MAX_SINGLE_TURN) {
            MotorParser_storePosition(parser_sign, parser_pos);
            parser_state = MP_WAIT_ADDR;
        } else if (byte == FRAME_TAIL) {
            /* 0x6B 结尾但 code 非 0x36，或 pos 超单圈范围：丢帧，不更新角度 */
            parser_state = MP_WAIT_ADDR;
        } else if (byte == MOTOR_ADDR) {
            /* 期望的帧尾被破坏：字节是地址，视为新帧起始候选 */
            parser_state = MP_WAIT_CODE;
        } else {
            parser_state = MP_WAIT_ADDR;
        }
        break;

    case MP_WAIT_SKIP:
        if (byte == FRAME_TAIL) {
            parser_state = MP_WAIT_ADDR;
        }
        break;

    default:
        MotorParser_reset();
        break;
    }
}

bool MotorParser_getPosX10(int32_t *pos_x10)
{
    if (!parser_has_pos || pos_x10 == 0) {
        return false;
    }
    *pos_x10 = parser_pos_x10;
    return true;
}

bool MotorParser_getAngleDegrees(int32_t *angle)
{
    int32_t x10;

    if (!MotorParser_getPosX10(&x10)) {
        return false;
    }
    if (x10 >= 0) {
        *angle = (x10 + 5) / 10;      /* 四舍五入 */
    } else {
        *angle = -((-x10 + 5) / 10);
    }
    return true;
}
