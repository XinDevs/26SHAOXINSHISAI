/**
 * @file    serial_maixcam.c
 * @brief   MaixCam 串口通信模块实现（UART2，115200bps，非阻塞）
 * @details 协议: @payload\r\n
 *          接收仿照 serial_cmd.c 的帧读取方式；
 *          发送通过 Serial2 的非阻塞接口完成。
 */

#include "serial_maixcam.h"
#include "Serial.h"
#include "ti_msp_dl_config.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 模块内接收缓冲 */
static char s_rxLocal[SERIAL_PACKET_SIZE];
static char s_cmdBuf[SERIAL_PACKET_SIZE];

/* 发送拼包缓冲 */
static char s_txBuf[SERIAL_PACKET_SIZE];

void SerialMaixCam_Init(void)
{
    Serial2_Init();
}

uint8_t SerialMaixCam_Process(void)
{
    char *rxCmdLine;

    /* 无新数据到达，直接返回 */
    if (Serial_RxFlag[SERIAL_UART2] == 0U) {
        return 0U;
    }

    /* 临界区：拷贝中断接收缓冲到本地，防止拷贝过程中被中断覆盖 */
    __disable_irq();
    Serial_RxFlag[SERIAL_UART2] = 0U;
    (void)strncpy(s_rxLocal, Serial_RxPacket[SERIAL_UART2], SERIAL_PACKET_SIZE - 1U);
    s_rxLocal[SERIAL_PACKET_SIZE - 1U] = '\0';
    __enable_irq();

    /* 跳过协议前缀 '@'，提取有效载荷 */
    rxCmdLine = s_rxLocal;
    if (rxCmdLine[0] == '@') {
        rxCmdLine++;
    }

    /* 存入命令缓冲区，供 GetCommand() 读取 */
    (void)strncpy(s_cmdBuf, rxCmdLine, SERIAL_PACKET_SIZE - 1U);
    s_cmdBuf[SERIAL_PACKET_SIZE - 1U] = '\0';

    return 1U;
}

const char *SerialMaixCam_GetCommand(void)
{
    return s_cmdBuf;
}

uint8_t SerialMaixCam_SendCommand(const char *cmd)
{
    int len;

    if (cmd == NULL) {
        return 0U;
    }

    len = snprintf(s_txBuf, sizeof(s_txBuf), "@%s\r\n", cmd);
    if (len < 0 || len >= (int)sizeof(s_txBuf)) {
        return 0U;
    }

    return Serial2_SendStringTry(s_txBuf);
}

//最好直接使用这个，当printf用
uint8_t SerialMaixCam_SendCommandf(const char *format, ...)
{
    va_list arg;
    int len;

    if (format == NULL) {
        return 0U;
    }

    s_txBuf[0] = '@';
    va_start(arg, format);
    len = vsnprintf(&s_txBuf[1], sizeof(s_txBuf) - 3U, format, arg);
    va_end(arg);

    if (len < 0 || len >= (int)(sizeof(s_txBuf) - 3U)) {
        return 0U;
    }

    s_txBuf[1U + (unsigned)len] = '\r';
    s_txBuf[2U + (unsigned)len] = '\n';
    s_txBuf[3U + (unsigned)len] = '\0';

    return Serial2_SendStringTry(s_txBuf);
}

uint8_t SerialMaixCam_SendRaw(const uint8_t *data, uint16_t length)
{
    return Serial2_SendArrayTry(data, length);
}

uint32_t SerialMaixCam_GetTxDropCount(void)
{
    return Serial2_GetTxDropCount();
}
