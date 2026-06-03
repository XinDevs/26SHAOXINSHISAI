/**
 * @file    Serial.c
 * @brief   串口 UART 通信驱动
 * @details 基于 MSPM0 UART + DMA 实现收发。
 * 采用单套接口，通过 uartId 选择 UART_BLUETOOTH/UART_STEPMOTOR/UART_CAM。
 */

#include "Serial.h"

#include "ti_msp_dl_config.h"
#include "ti/driverlib/dl_dma.h"
#include "ti/driverlib/dl_uart_main.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

char Serial_RxPacket[SERIAL_PORT_COUNT][SERIAL_PACKET_SIZE];
volatile uint8_t Serial_RxFlag[SERIAL_PORT_COUNT] = {0U, 0U, 0U}; // 扩充为3个

#define SERIAL_BUF_INVALID      0xFFU
#define SERIAL_PARSE_WAIT_HEAD  0U
#define SERIAL_PARSE_IN_PAYLOAD 1U
#define SERIAL_PARSE_WAIT_LF    2U
#define SERIAL_CAM_PARSE_WAIT_HEAD    0U
#define SERIAL_CAM_PARSE_WAIT_LENGTH  1U
#define SERIAL_CAM_PARSE_IN_PAYLOAD   2U
#define SERIAL_CAM_PARSE_WAIT_TAIL    3U

#define SERIAL_CAM_FRAME_HEAD   0xFFU
#define SERIAL_CAM_FRAME_TAIL   0xFEU

static uint8_t s_rxPacketDoubleBuf[SERIAL_PORT_COUNT][2][SERIAL_PACKET_SIZE];
static volatile uint8_t s_rxWriteBufIndex[SERIAL_PORT_COUNT] = {0U, 0U, 0U}; // 扩充
static volatile uint16_t s_rxWritePos[SERIAL_PORT_COUNT] = {0U, 0U, 0U};     // 扩充
static volatile uint16_t s_rxExpectedLen[SERIAL_PORT_COUNT] = {0U, 0U, 0U};   // 摄像头帧长度
static volatile uint8_t s_rxParseState[SERIAL_PORT_COUNT] = {
    SERIAL_PARSE_WAIT_HEAD,
    SERIAL_PARSE_WAIT_HEAD,
    SERIAL_PARSE_WAIT_HEAD  // 扩充
};

static uint8_t s_rxDmaStaging[SERIAL_PORT_COUNT][2] = {{0U, 0U}, {0U, 0U}, {0U, 0U}}; // 扩充
static volatile uint8_t s_rxDmaActiveIndex[SERIAL_PORT_COUNT] = {0U, 0U, 0U};         // 扩充

static uint8_t s_txDoubleBuf[SERIAL_PORT_COUNT][2][SERIAL_TX_BUFFER_SIZE];
static volatile uint16_t s_txLen[SERIAL_PORT_COUNT][2] = {{0U, 0U}, {0U, 0U}, {0U, 0U}}; // 扩充
static volatile uint8_t s_txActiveBuf[SERIAL_PORT_COUNT] = {
    SERIAL_BUF_INVALID,
    SERIAL_BUF_INVALID,
    SERIAL_BUF_INVALID // 扩充
};
static volatile uint8_t s_txPendingBuf[SERIAL_PORT_COUNT] = {
    SERIAL_BUF_INVALID,
    SERIAL_BUF_INVALID,
    SERIAL_BUF_INVALID // 扩充
};
static volatile uint32_t s_txDropCount[SERIAL_PORT_COUNT] = {0U, 0U, 0U}; // 扩充

static uint8_t s_echoBuf[SERIAL_PORT_COUNT][SERIAL_TX_BUFFER_SIZE];

/**
 * @brief  校验 uartId 是否有效
 */
static uint8_t Serial_IsValidUart(uint8_t uartId)
{
    return (uartId < SERIAL_PORT_COUNT) ? 1U : 0U;
}

/**
 * @brief  计算整数幂 x^y
 */
static uint32_t Serial_Pow(uint32_t x, uint8_t y)
{
    uint32_t result = 1U;
    while (y--) {
        result *= x;
    }
    return result;
}

/**
 * @brief  进入临界区
 */
static uint32_t Serial_CriticalEnter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/**
 * @brief  退出临界区
 */
static void Serial_CriticalExit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief  获取指定串口的 TX DMA 通道
 */
static uint8_t Serial_GetTxDmaChannel(uint8_t uartId)
{
    if (uartId == SERIAL_UART2) {
        return DMA_CH5_CHAN_ID; // UART2 TX
    }
    return (uartId == SERIAL_UART1) ? DMA_CH3_CHAN_ID : DMA_CH1_CHAN_ID;
}

/**
 * @brief  获取指定串口的 RX DMA 通道
 */
static uint8_t Serial_GetRxDmaChannel(uint8_t uartId)
{
    if (uartId == SERIAL_UART2) {
        return DMA_CH4_CHAN_ID; // UART2 RX
    }
    return (uartId == SERIAL_UART1) ? DMA_CH2_CHAN_ID : DMA_CH0_CHAN_ID;
}

/**
 * @brief  获取指定串口 TXDATA 地址
 */
static uint32_t Serial_GetTxDataAddr(uint8_t uartId)
{
    if (uartId == SERIAL_UART2) {
        return (uint32_t) (uintptr_t) &(UART_CAM_INST->TXDATA);
    } else if (uartId == SERIAL_UART1) {
        return (uint32_t) (uintptr_t) &(UART_STEPMOTOR_INST->TXDATA);
    }
    return (uint32_t) (uintptr_t) &(UART_BLUETOOTH_INST->TXDATA);
}

/**
 * @brief  获取指定串口 RXDATA 地址
 */
static uint32_t Serial_GetRxDataAddr(uint8_t uartId)
{
    if (uartId == SERIAL_UART2) {
        return (uint32_t) (uintptr_t) &(UART_CAM_INST->RXDATA);
    } else if (uartId == SERIAL_UART1) {
        return (uint32_t) (uintptr_t) &(UART_STEPMOTOR_INST->RXDATA);
    }
    return (uint32_t) (uintptr_t) &(UART_BLUETOOTH_INST->RXDATA);
}

/**
 * @brief  启动发送 DMA
 */
static void Serial_StartTxDMA(uint8_t uartId, uint8_t bufIndex)
{
    uint8_t txChan = Serial_GetTxDmaChannel(uartId);

    DL_DMA_disableChannel(DMA, txChan);
    DL_DMA_setSrcAddr(
        DMA,
        txChan,
        (uint32_t) (uintptr_t) &s_txDoubleBuf[uartId][bufIndex][0]);
    DL_DMA_setDestAddr(DMA, txChan, Serial_GetTxDataAddr(uartId));
    DL_DMA_setTransferSize(DMA, txChan, s_txLen[uartId][bufIndex]);
    DL_DMA_enableChannel(DMA, txChan);
}

/**
 * @brief  启动接收 DMA(单字节)
 */
static void Serial_StartRxDMA(uint8_t uartId, uint8_t stagingIndex)
{
    uint8_t rxChan = Serial_GetRxDmaChannel(uartId);

    DL_DMA_disableChannel(DMA, rxChan);
    DL_DMA_setSrcAddr(DMA, rxChan, Serial_GetRxDataAddr(uartId));
    DL_DMA_setDestAddr(
        DMA,
        rxChan,
        (uint32_t) (uintptr_t) &s_rxDmaStaging[uartId][stagingIndex]);
    DL_DMA_setTransferSize(DMA, rxChan, 1U);
    DL_DMA_enableChannel(DMA, rxChan);
}

/**
 * @brief  尝试入队发送缓冲
 */
static uint8_t Serial_QueueTxBufferTry(uint8_t uartId, const uint8_t *data, uint16_t length)
{
    uint32_t key;
    uint8_t active;
    uint8_t pending;

    key = Serial_CriticalEnter();
    active = s_txActiveBuf[uartId];
    pending = s_txPendingBuf[uartId];

    if (active == SERIAL_BUF_INVALID) {
        memcpy(&s_txDoubleBuf[uartId][0][0], data, length);
        s_txLen[uartId][0] = length;
        s_txActiveBuf[uartId] = 0U;
        Serial_StartTxDMA(uartId, 0U);
        Serial_CriticalExit(key);
        return 1U;
    }

    if (pending == SERIAL_BUF_INVALID) {
        uint8_t pendingIndex = (uint8_t) (active ^ 1U);
        memcpy(&s_txDoubleBuf[uartId][pendingIndex][0], data, length);
        s_txLen[uartId][pendingIndex] = length;
        s_txPendingBuf[uartId] = pendingIndex;
        Serial_CriticalExit(key);
        return 1U;
    }

    Serial_CriticalExit(key);
    return 0U;
}

/**
 * @brief  阻塞等待发送缓冲可用
 */
static void Serial_QueueTxBufferBlocking(uint8_t uartId, const uint8_t *data, uint16_t length)
{
    while (Serial_QueueTxBufferTry(uartId, data, length) == 0U) {
    }
}

/**
 * @brief  处理一帧接收完成
 */
static void Serial_HandleFrameComplete(uint8_t uartId)
{
    uint8_t frameIndex = s_rxWriteBufIndex[uartId];
    uint16_t payloadLen = s_rxWritePos[uartId];

    if (payloadLen >= SERIAL_PACKET_SIZE) {
        payloadLen = SERIAL_PACKET_SIZE - 1U;
    }

    s_rxPacketDoubleBuf[uartId][frameIndex][payloadLen] = '\0';

    if (payloadLen > (SERIAL_TX_BUFFER_SIZE - 2U)) {
        payloadLen = SERIAL_TX_BUFFER_SIZE - 2U;
    }

    memcpy(s_echoBuf[uartId], s_rxPacketDoubleBuf[uartId][frameIndex], payloadLen);
    s_echoBuf[uartId][payloadLen] = '\r';
    s_echoBuf[uartId][payloadLen + 1U] = '\n';
    (void) Serial_SendArrayTry(uartId, s_echoBuf[uartId], (uint16_t) (payloadLen + 2U));

    Serial_DMA_RxEvent(uartId, payloadLen);
}

static void Serial_HandleCameraFrameComplete(uint16_t payloadLen)
{
    uint8_t frameIndex = s_rxWriteBufIndex[SERIAL_UART2];

    if (payloadLen >= SERIAL_PACKET_SIZE) {
        payloadLen = SERIAL_PACKET_SIZE - 1U;
    }

    s_rxPacketDoubleBuf[SERIAL_UART2][frameIndex][payloadLen] = '\0';
    Serial_DMA_RxEvent(SERIAL_UART2, payloadLen);
}

static void Serial_PushCameraReceivedByte(uint8_t data)
{
    switch (s_rxParseState[SERIAL_UART2]) {
        case SERIAL_CAM_PARSE_WAIT_HEAD:
            if (data == SERIAL_CAM_FRAME_HEAD) {
                s_rxWritePos[SERIAL_UART2] = 0U;
                s_rxExpectedLen[SERIAL_UART2] = 0U;
                s_rxParseState[SERIAL_UART2] = SERIAL_CAM_PARSE_WAIT_LENGTH;
            }
            break;

        case SERIAL_CAM_PARSE_WAIT_LENGTH:
            if ((data > 0U) && (data < SERIAL_PACKET_SIZE)) {
                s_rxWritePos[SERIAL_UART2] = 0U;
                s_rxExpectedLen[SERIAL_UART2] = data;
                s_rxParseState[SERIAL_UART2] = SERIAL_CAM_PARSE_IN_PAYLOAD;
            } else {
                s_rxWritePos[SERIAL_UART2] = 0U;
                s_rxExpectedLen[SERIAL_UART2] = 0U;
                s_rxParseState[SERIAL_UART2] = SERIAL_CAM_PARSE_WAIT_HEAD;
            }
            break;

        case SERIAL_CAM_PARSE_IN_PAYLOAD:
            s_rxPacketDoubleBuf[SERIAL_UART2][s_rxWriteBufIndex[SERIAL_UART2]]
                               [s_rxWritePos[SERIAL_UART2]++] = data;
            if (s_rxWritePos[SERIAL_UART2] >= s_rxExpectedLen[SERIAL_UART2]) {
                s_rxParseState[SERIAL_UART2] = SERIAL_CAM_PARSE_WAIT_TAIL;
            }
            break;

        case SERIAL_CAM_PARSE_WAIT_TAIL:
            if (data == SERIAL_CAM_FRAME_TAIL) {
                Serial_HandleCameraFrameComplete(s_rxExpectedLen[SERIAL_UART2]);
                s_rxWritePos[SERIAL_UART2] = 0U;
                s_rxExpectedLen[SERIAL_UART2] = 0U;
                s_rxParseState[SERIAL_UART2] = SERIAL_CAM_PARSE_WAIT_HEAD;
            } else if (data == SERIAL_CAM_FRAME_HEAD) {
                s_rxWritePos[SERIAL_UART2] = 0U;
                s_rxExpectedLen[SERIAL_UART2] = 0U;
                s_rxParseState[SERIAL_UART2] = SERIAL_CAM_PARSE_WAIT_LENGTH;
            } else {
                s_rxWritePos[SERIAL_UART2] = 0U;
                s_rxExpectedLen[SERIAL_UART2] = 0U;
                s_rxParseState[SERIAL_UART2] = SERIAL_CAM_PARSE_WAIT_HEAD;
            }
            break;

        default:
            s_rxWritePos[SERIAL_UART2] = 0U;
            s_rxExpectedLen[SERIAL_UART2] = 0U;
            s_rxParseState[SERIAL_UART2] = SERIAL_CAM_PARSE_WAIT_HEAD;
            break;
    }
}

/**
 * @brief  接收状态机压入单字节
 */
static void Serial_PushReceivedByte(uint8_t uartId, uint8_t data)
{
    if (uartId == SERIAL_UART2) {
        Serial_PushCameraReceivedByte(data);
        return;
    }

    switch (s_rxParseState[uartId]) {
        case SERIAL_PARSE_WAIT_HEAD:
            if (data == '@') {
                s_rxWritePos[uartId] = 0U;
                s_rxParseState[uartId] = SERIAL_PARSE_IN_PAYLOAD;
            }
            break;

        case SERIAL_PARSE_IN_PAYLOAD:
            if (data == '\r') {
                s_rxParseState[uartId] = SERIAL_PARSE_WAIT_LF;
            } else {
                if (s_rxWritePos[uartId] < (SERIAL_PACKET_SIZE - 1U)) {
                    s_rxPacketDoubleBuf[uartId][s_rxWriteBufIndex[uartId]][s_rxWritePos[uartId]++] =
                        (char) data;
                } else {
                    s_rxWritePos[uartId] = 0U;
                    s_rxParseState[uartId] = SERIAL_PARSE_WAIT_HEAD;
                }
            }
            break;

        case SERIAL_PARSE_WAIT_LF:
            if (data == '\n') {
                Serial_HandleFrameComplete(uartId);
                s_rxWritePos[uartId] = 0U;
                s_rxParseState[uartId] = SERIAL_PARSE_WAIT_HEAD;
            } else if (data == '@') {
                s_rxWritePos[uartId] = 0U;
                s_rxParseState[uartId] = SERIAL_PARSE_IN_PAYLOAD;
            } else {
                s_rxWritePos[uartId] = 0U;
                s_rxParseState[uartId] = SERIAL_PARSE_WAIT_HEAD;
            }
            break;

        default:
            s_rxWritePos[uartId] = 0U;
            s_rxParseState[uartId] = SERIAL_PARSE_WAIT_HEAD;
            break;
    }
}

/**
 * @brief  串口驱动初始化
 */
void Serial_Init(uint8_t uartId)
{
    if (Serial_IsValidUart(uartId) == 0U) {
        return;
    }

    memset((void *) s_rxPacketDoubleBuf[uartId], 0, sizeof(s_rxPacketDoubleBuf[uartId]));
    memset(Serial_RxPacket[uartId], 0, SERIAL_PACKET_SIZE);
    memset((void *) s_txDoubleBuf[uartId], 0, sizeof(s_txDoubleBuf[uartId]));

    s_rxWriteBufIndex[uartId] = 0U;
    s_rxWritePos[uartId] = 0U;
    s_rxExpectedLen[uartId] = 0U;
    s_rxParseState[uartId] = SERIAL_PARSE_WAIT_HEAD;
    s_rxDmaActiveIndex[uartId] = 0U;

    s_txLen[uartId][0] = 0U;
    s_txLen[uartId][1] = 0U;
    s_txActiveBuf[uartId] = SERIAL_BUF_INVALID;
    s_txPendingBuf[uartId] = SERIAL_BUF_INVALID;
    s_txDropCount[uartId] = 0U;

    Serial_RxFlag[uartId] = 0U;

    if (uartId == SERIAL_UART2) {
        DL_UART_Main_enableInterrupt(
            UART_CAM_INST,
            DL_UART_MAIN_INTERRUPT_DMA_DONE_TX | DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
        Serial_StartRxDMA(uartId, s_rxDmaActiveIndex[uartId]);
        NVIC_ClearPendingIRQ(UART_CAM_INST_INT_IRQN);
        NVIC_EnableIRQ(UART_CAM_INST_INT_IRQN);
    } else if (uartId == SERIAL_UART1) {
        DL_UART_Main_enableInterrupt(
            UART_STEPMOTOR_INST,
            DL_UART_MAIN_INTERRUPT_DMA_DONE_TX | DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
        Serial_StartRxDMA(uartId, s_rxDmaActiveIndex[uartId]);
        NVIC_ClearPendingIRQ(UART_STEPMOTOR_INST_INT_IRQN);
        NVIC_EnableIRQ(UART_STEPMOTOR_INST_INT_IRQN);
    } else {
        DL_UART_Main_enableInterrupt(
            UART_BLUETOOTH_INST,
            DL_UART_MAIN_INTERRUPT_DMA_DONE_TX | DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
        Serial_StartRxDMA(uartId, s_rxDmaActiveIndex[uartId]);
        NVIC_ClearPendingIRQ(UART_BLUETOOTH_INST_INT_IRQN);
        NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);
    }
}

/**
 * @brief  发送单字节(阻塞)
 */
void Serial_SendByte(uint8_t uartId, uint8_t byte)
{
    Serial_SendArray(uartId, &byte, 1U);
}

/**
 * @brief  发送字节数组(阻塞)
 */
void Serial_SendArray(uint8_t uartId, uint8_t *array, uint16_t length)
{
    uint16_t offset = 0U;

    if ((Serial_IsValidUart(uartId) == 0U) || (array == NULL) || (length == 0U)) {
        return;
    }

    while (offset < length) {
        uint16_t chunk = (uint16_t) (length - offset);

        if (chunk > SERIAL_TX_BUFFER_SIZE) {
            chunk = SERIAL_TX_BUFFER_SIZE;
        }

        Serial_QueueTxBufferBlocking(uartId, &array[offset], chunk);
        offset = (uint16_t) (offset + chunk);
    }
}

/**
 * @brief  发送字符串(阻塞)
 */
void Serial_SendString(uint8_t uartId, char *str)
{
    if ((Serial_IsValidUart(uartId) == 0U) || (str == NULL)) {
        return;
    }

    Serial_SendArray(uartId, (uint8_t *) str, (uint16_t) strlen(str));
}

/**
 * @brief  发送字节数组(非阻塞)
 */
uint8_t Serial_SendArrayTry(uint8_t uartId, const uint8_t *array, uint16_t length)
{
    uint16_t offset = 0U;

    if ((Serial_IsValidUart(uartId) == 0U) || (array == NULL) || (length == 0U)) {
        return 0U;
    }

    while (offset < length) {
        uint16_t chunk = (uint16_t) (length - offset);

        if (chunk > SERIAL_TX_BUFFER_SIZE) {
            chunk = SERIAL_TX_BUFFER_SIZE;
        }

        if (Serial_QueueTxBufferTry(uartId, &array[offset], chunk) == 0U) {
            s_txDropCount[uartId]++;
            return 0U;
        }

        offset = (uint16_t) (offset + chunk);
    }

    return 1U;
}

/**
 * @brief  发送字符串(非阻塞)
 */
uint8_t Serial_SendStringTry(uint8_t uartId, const char *str)
{
    if ((Serial_IsValidUart(uartId) == 0U) || (str == NULL)) {
        return 0U;
    }

    return Serial_SendArrayTry(uartId, (const uint8_t *) str, (uint16_t) strlen(str));
}

/**
 * @brief  获取发送丢包计数
 */
uint32_t Serial_GetTxDropCount(uint8_t uartId)
{
    if (Serial_IsValidUart(uartId) == 0U) {
        return 0U;
    }
    return s_txDropCount[uartId];
}

/**
 * @brief  按十进制固定宽度发送无符号整数
 */
void Serial_SendNumber(uint8_t uartId, uint32_t num, uint8_t length)
{
    uint8_t i;

    if (Serial_IsValidUart(uartId) == 0U) {
        return;
    }

    for (i = 0U; i < length; i++) {
        uint32_t div = Serial_Pow(10U, (uint8_t) (length - i - 1U));
        Serial_SendByte(uartId, (uint8_t) (num / div % 10U + '0'));
    }
}

/**
 * @brief  指定串口的格式化输出内部实现
 */
static void Serial_PrintfByPort(uint8_t uartId, char *format, va_list arg)
{
    char str[128];

    if ((Serial_IsValidUart(uartId) == 0U) || (format == NULL)) {
        return;
    }

    (void) vsnprintf(str, sizeof(str), format, arg);
    Serial_SendString(uartId, str);
}

/**
 * @brief  串口格式化输出
 */
void Serial_Printf(uint8_t uartId, char *format, ...)
{
    va_list arg;

    va_start(arg, format);
    Serial_PrintfByPort(uartId, format, arg);
    va_end(arg);
}


/* =========================================================================
 * 各串口的快捷封装接口 (UART0 / UART1 / UART2)
 * ========================================================================= */

/**
 * @brief  UART0 专用接口
 */
void Serial0_Init(void) { Serial_Init(SERIAL_UART0); }
void Serial0_SendByte(uint8_t byte) { Serial_SendByte(SERIAL_UART0, byte); }
void Serial0_SendArray(uint8_t *array, uint16_t length) { Serial_SendArray(SERIAL_UART0, array, length); }
void Serial0_SendString(char *str) { Serial_SendString(SERIAL_UART0, str); }
void Serial0_SendNumber(uint32_t num, uint8_t length) { Serial_SendNumber(SERIAL_UART0, num, length); }
void Serial0_Printf(char *format, ...) {
    va_list arg;
    va_start(arg, format);
    Serial_PrintfByPort(SERIAL_UART0, format, arg);
    va_end(arg);
}
void Serial0_DMA_RxEvent(uint16_t size) { Serial_DMA_RxEvent(SERIAL_UART0, size); }
uint8_t Serial0_SendArrayTry(const uint8_t *array, uint16_t length) { return Serial_SendArrayTry(SERIAL_UART0, array, length); }
uint8_t Serial0_SendStringTry(const char *str) { return Serial_SendStringTry(SERIAL_UART0, str); }
uint32_t Serial0_GetTxDropCount(void) { return Serial_GetTxDropCount(SERIAL_UART0); }

/**
 * @brief  UART1 专用接口
 */
void Serial1_Init(void) { Serial_Init(SERIAL_UART1); }
void Serial1_SendByte(uint8_t byte) { Serial_SendByte(SERIAL_UART1, byte); }
void Serial1_SendArray(uint8_t *array, uint16_t length) { Serial_SendArray(SERIAL_UART1, array, length); }
void Serial1_SendString(char *str) { Serial_SendString(SERIAL_UART1, str); }
void Serial1_SendNumber(uint32_t num, uint8_t length) { Serial_SendNumber(SERIAL_UART1, num, length); }
void Serial1_Printf(char *format, ...) {
    va_list arg;
    va_start(arg, format);
    Serial_PrintfByPort(SERIAL_UART1, format, arg);
    va_end(arg);
}
void Serial1_DMA_RxEvent(uint16_t size) { Serial_DMA_RxEvent(SERIAL_UART1, size); }
uint8_t Serial1_SendArrayTry(const uint8_t *array, uint16_t length) { return Serial_SendArrayTry(SERIAL_UART1, array, length); }
uint8_t Serial1_SendStringTry(const char *str) { return Serial_SendStringTry(SERIAL_UART1, str); }
uint32_t Serial1_GetTxDropCount(void) { return Serial_GetTxDropCount(SERIAL_UART1); }

/**
 * @brief  UART2 专用接口 (新增)
 */
void Serial2_Init(void) { Serial_Init(SERIAL_UART2); }
void Serial2_SendByte(uint8_t byte) { Serial_SendByte(SERIAL_UART2, byte); }
void Serial2_SendArray(uint8_t *array, uint16_t length) { Serial_SendArray(SERIAL_UART2, array, length); }
void Serial2_SendString(char *str) { Serial_SendString(SERIAL_UART2, str); }
void Serial2_SendNumber(uint32_t num, uint8_t length) { Serial_SendNumber(SERIAL_UART2, num, length); }
void Serial2_Printf(char *format, ...) {
    va_list arg;
    va_start(arg, format);
    Serial_PrintfByPort(SERIAL_UART2, format, arg);
    va_end(arg);
}
void Serial2_DMA_RxEvent(uint16_t size) { Serial_DMA_RxEvent(SERIAL_UART2, size); }
uint8_t Serial2_SendArrayTry(const uint8_t *array, uint16_t length) { return Serial_SendArrayTry(SERIAL_UART2, array, length); }
uint8_t Serial2_SendStringTry(const char *str) { return Serial_SendStringTry(SERIAL_UART2, str); }
uint32_t Serial2_GetTxDropCount(void) { return Serial_GetTxDropCount(SERIAL_UART2); }

/* =========================================================================
 * 中断及 DMA 回调处理
 * ========================================================================= */

/**
 * @brief  DMA 接收帧完成回调
 */
void Serial_DMA_RxEvent(uint8_t uartId, uint16_t size)
{
    uint16_t copyLen = size;

    if (Serial_IsValidUart(uartId) == 0U) {
        return;
    }

    if (copyLen >= SERIAL_PACKET_SIZE) {
        copyLen = SERIAL_PACKET_SIZE - 1U;
    }

    memcpy(
        Serial_RxPacket[uartId],
        s_rxPacketDoubleBuf[uartId][s_rxWriteBufIndex[uartId]],
        copyLen);
    Serial_RxPacket[uartId][copyLen] = '\0';
    Serial_RxFlag[uartId] = 1U;

    s_rxWriteBufIndex[uartId] ^= 1U;
    memset(
        (void *) s_rxPacketDoubleBuf[uartId][s_rxWriteBufIndex[uartId]],
        0,
        SERIAL_PACKET_SIZE);
}

/**
 * @brief  统一 UART DMA 中断处理
 */
static void Serial_UART_IRQHandlerCommon(uint8_t uartId)
{
    DL_UART_IIDX iidx;

    if (Serial_IsValidUart(uartId) == 0U) {
        return;
    }

    if (uartId == SERIAL_UART2) {
        iidx = DL_UART_getPendingInterrupt(UART_CAM_INST);
    } else if (uartId == SERIAL_UART1) {
        iidx = DL_UART_getPendingInterrupt(UART_STEPMOTOR_INST);
    } else {
        iidx = DL_UART_getPendingInterrupt(UART_BLUETOOTH_INST);
    }

    if (iidx == DL_UART_IIDX_DMA_DONE_RX) {
        uint8_t byte;

        byte = s_rxDmaStaging[uartId][s_rxDmaActiveIndex[uartId]];
        s_rxDmaActiveIndex[uartId] ^= 1U;
        Serial_StartRxDMA(uartId, s_rxDmaActiveIndex[uartId]);
        Serial_PushReceivedByte(uartId, byte);
    } else if (iidx == DL_UART_IIDX_DMA_DONE_TX) {
        if (s_txPendingBuf[uartId] != SERIAL_BUF_INVALID) {
            uint8_t nextBuf = s_txPendingBuf[uartId];

            s_txPendingBuf[uartId] = SERIAL_BUF_INVALID;
            s_txActiveBuf[uartId] = nextBuf;
            Serial_StartTxDMA(uartId, nextBuf);
        } else {
            s_txActiveBuf[uartId] = SERIAL_BUF_INVALID;
        }
    } else {
        /* 忽略其他 UART 中断源 */
    }
}

/**
 * @brief  蓝牙串口中断服务函数
 */
void UART_BLUETOOTH_INST_IRQHandler(void)
{
    Serial_UART_IRQHandlerCommon(SERIAL_UART0);
}

/**
 * @brief  步进电机串口中断服务函数
 */
void UART_STEPMOTOR_INST_IRQHandler(void)
{
    Serial_UART_IRQHandlerCommon(SERIAL_UART1);
}

/**
 * @brief  摄像头串口中断服务函数
 */
void UART_CAM_INST_IRQHandler(void)
{
    Serial_UART_IRQHandlerCommon(SERIAL_UART2);
}
