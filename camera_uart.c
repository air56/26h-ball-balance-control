#include "camera_uart.h"

typedef enum {
    CAMERA_WAIT_HEADER,
    CAMERA_WAIT_X_LOW,
    CAMERA_WAIT_X_HIGH,
    CAMERA_WAIT_TAIL
} CameraUart_ParseState;

#define CAMERA_FRAME_QUEUE_SIZE 8U

static CameraUart_ParseState camera_parse_state;
static uint8_t camera_x_low;
static uint8_t camera_x_high;
static volatile uint16_t camera_latest_x;
static volatile bool camera_latest_x_valid;
static volatile uint16_t camera_frame_queue[CAMERA_FRAME_QUEUE_SIZE];
static volatile uint8_t camera_queue_write_index;
static volatile uint8_t camera_queue_read_index;
/* 诊断计数器：UART1 收到的总字节数、解析出的有效帧数（供 OLED 定位链路）*/
static volatile uint32_t camera_rx_byte_count;
static volatile uint32_t camera_rx_frame_count;
/* 诊断：记录最近一个收到的原始字节（判断数据内容是否匹配 0xAA 帧头）*/
static volatile uint8_t camera_rx_last_byte;

static uint8_t CameraUart_nextQueueIndex(uint8_t index)
{
    index++;
    return (index == CAMERA_FRAME_QUEUE_SIZE) ? 0U : index;
}

static void CameraUart_publishX(uint16_t x)
{
    uint8_t next_write_index =
        CameraUart_nextQueueIndex(camera_queue_write_index);

    camera_latest_x = x;
    camera_latest_x_valid = true;
    camera_rx_frame_count++;

    if (next_write_index != camera_queue_read_index) {
        camera_frame_queue[camera_queue_write_index] = x;
        camera_queue_write_index = next_write_index;
    }
}

void CameraUart_reset(void)
{
    camera_parse_state = CAMERA_WAIT_HEADER;
    camera_x_low = 0U;
    camera_x_high = 0U;
    camera_latest_x = 0U;
    camera_latest_x_valid = false;
    camera_queue_write_index = 0U;
    camera_queue_read_index = 0U;
}

void CameraUart_processByte(uint8_t byte)
{
    camera_rx_byte_count++;
    camera_rx_last_byte = byte;

    switch (camera_parse_state) {
    case CAMERA_WAIT_HEADER:
        if (byte == 0xAAU) {
            camera_parse_state = CAMERA_WAIT_X_LOW;
        }
        break;

    case CAMERA_WAIT_X_LOW:
        camera_x_low = byte;
        camera_parse_state = CAMERA_WAIT_X_HIGH;
        break;

    case CAMERA_WAIT_X_HIGH:
        if (byte == 0xAAU) {
            camera_parse_state = CAMERA_WAIT_X_LOW;
        } else {
            camera_x_high = byte;
            camera_parse_state = CAMERA_WAIT_TAIL;
        }
        break;

    case CAMERA_WAIT_TAIL:
        if (byte == 0xBBU) {
            CameraUart_publishX(((uint16_t)camera_x_high << 8) | camera_x_low);
            camera_parse_state = CAMERA_WAIT_HEADER;
        } else if (byte == 0xAAU) {
            camera_parse_state = CAMERA_WAIT_X_LOW;
        } else {
            camera_parse_state = CAMERA_WAIT_HEADER;
        }
        break;

    default:
        CameraUart_reset();
        break;
    }
}

bool CameraUart_getLatestX(uint16_t *x)
{
    if (!camera_latest_x_valid || x == 0) {
        return false;
    }

    *x = camera_latest_x;
    return true;
}

bool CameraUart_takeLatestX(uint16_t *x)
{
    uint8_t read_index;

    if (x == 0) {
        return false;
    }

    read_index = camera_queue_read_index;
    if (read_index == camera_queue_write_index) {
        return false;
    }

    *x = camera_frame_queue[read_index];
    camera_queue_read_index = CameraUart_nextQueueIndex(read_index);
    return true;
}

/* 诊断：返回 UART1 收到的总字节数（判断中断是否触发） */
bool CameraUart_getRxByteCount(uint32_t *count)
{
    if (count == 0) {
        return false;
    }
    *count = camera_rx_byte_count;
    return true;
}

/* 诊断：返回解析出的有效帧数（判断协议是否匹配） */
bool CameraUart_getRxFrameCount(uint32_t *count)
{
    if (count == 0) {
        return false;
    }
    *count = camera_rx_frame_count;
    return true;
}

/* 诊断：返回最近收到的原始字节（十六进制值，判断数据内容） */
bool CameraUart_getRxLastByte(uint8_t *byte)
{
    if (byte == 0) {
        return false;
    }
    *byte = camera_rx_last_byte;
    return true;
}
