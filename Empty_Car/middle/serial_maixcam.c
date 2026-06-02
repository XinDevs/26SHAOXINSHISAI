/**
 * @file    serial_maixcam.c
 * @brief   MaixCam UART2 communication module.
 * @details Frame: 0xFF, length, payload, 0xFE.
 *          Result payload uses one byte: 0x00 None, 0x01 red circle,
 *          0x02 red square, 0x03 green circle, 0x04 green square.
 */

#include "serial_maixcam.h"
#include "Serial.h"
#include "ti_msp_dl_config.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 模块内接收缓冲 */
static char s_cmdBuf[SERIAL_PACKET_SIZE];
static uint8_t s_resultCode = SERIAL_MAIXCAM_RESULT_NONE;

/* 发送拼包缓冲 */
static uint8_t s_txBuf[SERIAL_PACKET_SIZE];

#define SERIAL_MAIXCAM_FRAME_HEAD 0xFFU
#define SERIAL_MAIXCAM_FRAME_TAIL 0xFEU

static uint8_t SerialMaixCam_SendPayloadByte(uint8_t payload)
{
    s_txBuf[0] = SERIAL_MAIXCAM_FRAME_HEAD;
    s_txBuf[1] = 0x01U;
    s_txBuf[2] = payload;
    s_txBuf[3] = SERIAL_MAIXCAM_FRAME_TAIL;

    return Serial2_SendArrayTry(s_txBuf, 4U);
}

void SerialMaixCam_Init(void)
{
    Serial2_Init();
}

uint8_t SerialMaixCam_Process(void)
{
    uint8_t payload;

    /* 无新数据到达，直接返回 */
    if (Serial_RxFlag[SERIAL_UART2] == 0U) {
        return 0U;
    }

    /* 临界区：拷贝中断接收缓冲到本地，防止拷贝过程中被中断覆盖 */
    __disable_irq();
    Serial_RxFlag[SERIAL_UART2] = 0U;
    payload = (uint8_t)Serial_RxPacket[SERIAL_UART2][0];
    __enable_irq();

    /* Decode binary result payload. */
    s_resultCode = payload;
    switch (payload) {
        case SERIAL_MAIXCAM_RESULT_NONE:
            (void)strncpy(s_cmdBuf, "None", SERIAL_PACKET_SIZE - 1U);
            break;

        case SERIAL_MAIXCAM_RESULT_RED_CIRCLE:
            (void)strncpy(s_cmdBuf, "RedCircle", SERIAL_PACKET_SIZE - 1U);
            break;

        case SERIAL_MAIXCAM_RESULT_RED_SQUARE:
            (void)strncpy(s_cmdBuf, "RedSquare", SERIAL_PACKET_SIZE - 1U);
            break;

        case SERIAL_MAIXCAM_RESULT_GREEN_CIRCLE:
            (void)strncpy(s_cmdBuf, "GreenCircle", SERIAL_PACKET_SIZE - 1U);
            break;

        case SERIAL_MAIXCAM_RESULT_GREEN_SQUARE:
            (void)strncpy(s_cmdBuf, "GreenSquare", SERIAL_PACKET_SIZE - 1U);
            break;

        default:
            (void)snprintf(s_cmdBuf, SERIAL_PACKET_SIZE, "Unknown:%02X", (unsigned int)payload);
            break;
    }
    s_cmdBuf[SERIAL_PACKET_SIZE - 1U] = '\0';

    return 1U;
}

const char *SerialMaixCam_GetCommand(void)
{
    return s_cmdBuf;
}

uint8_t SerialMaixCam_GetResultCode(void)
{
    return s_resultCode;
}

uint8_t SerialMaixCam_SendStartRequest(void)
{
    return SerialMaixCam_SendPayloadByte(SERIAL_MAIXCAM_CMD_START_REQUEST);
}

uint8_t SerialMaixCam_SendStopRequest(void)
{
    return SerialMaixCam_SendPayloadByte(SERIAL_MAIXCAM_CMD_STOP_REQUEST);
}

uint8_t SerialMaixCam_SendCommand(const char *cmd)
{
    size_t len;

    if (cmd == NULL) {
        return 0U;
    }

    len = strlen(cmd);
    if ((len == 0U) || (len > (sizeof(s_txBuf) - 3U))) {
        return 0U;
    }

    s_txBuf[0] = SERIAL_MAIXCAM_FRAME_HEAD;
    s_txBuf[1] = (uint8_t)len;
    memcpy(&s_txBuf[2], cmd, len);
    s_txBuf[2U + len] = SERIAL_MAIXCAM_FRAME_TAIL;

    return Serial2_SendArrayTry(s_txBuf, (uint16_t)(len + 3U));
}

//最好直接使用这个，当printf用
uint8_t SerialMaixCam_SendCommandf(const char *format, ...)
{
    va_list arg;
    int len;

    if (format == NULL) {
        return 0U;
    }

    s_txBuf[0] = SERIAL_MAIXCAM_FRAME_HEAD;
    va_start(arg, format);
    len = vsnprintf((char *)&s_txBuf[2], sizeof(s_txBuf) - 3U, format, arg);
    va_end(arg);

    if (len <= 0 || len >= (int)(sizeof(s_txBuf) - 3U)) {
        return 0U;
    }

    s_txBuf[1] = (uint8_t)len;
    s_txBuf[2U + (unsigned)len] = SERIAL_MAIXCAM_FRAME_TAIL;

    return Serial2_SendArrayTry(s_txBuf, (uint16_t)((unsigned)len + 3U));
}

uint8_t SerialMaixCam_SendRaw(const uint8_t *data, uint16_t length)
{
    return Serial2_SendArrayTry(data, length);
}

uint32_t SerialMaixCam_GetTxDropCount(void)
{
    return Serial2_GetTxDropCount();
}
