/**
 * @file    Serial.h
 * @brief   串口UART通信驱动 — 头文件
 * @details 单套API通过 uartId 选择蓝牙/步进电机/摄像头串口，支持 DMA 收发与帧解析。
 */
#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERIAL_PORT_COUNT      3U
#define SERIAL_UART_BLUETOOTH  0U
#define SERIAL_UART_STEPMOTOR  1U
#define SERIAL_UART_CAM        2U

#define SERIAL_UART0           SERIAL_UART_BLUETOOTH
#define SERIAL_UART1           SERIAL_UART_STEPMOTOR
#define SERIAL_UART2           SERIAL_UART_CAM

#define SERIAL_PACKET_SIZE     128U
#define SERIAL_TX_BUFFER_SIZE  128U

extern char Serial_RxPacket[SERIAL_PORT_COUNT][SERIAL_PACKET_SIZE];
extern volatile uint8_t Serial_RxFlag[SERIAL_PORT_COUNT];

void Serial_Init(uint8_t uartId);
void Serial_SendByte(uint8_t uartId, uint8_t byte);
void Serial_SendArray(uint8_t uartId, uint8_t *array, uint16_t length);
void Serial_SendString(uint8_t uartId, char *str);
void Serial_SendNumber(uint8_t uartId, uint32_t num, uint8_t length);
void Serial_Printf(uint8_t uartId, char *format, ...);
void Serial_DMA_RxEvent(uint8_t uartId, uint16_t size);

uint8_t Serial_SendArrayTry(uint8_t uartId, const uint8_t *array, uint16_t length);
uint8_t Serial_SendStringTry(uint8_t uartId, const char *str);
uint32_t Serial_GetTxDropCount(uint8_t uartId);

/* UART0 专用接口 */
void Serial0_Init(void);
void Serial0_SendByte(uint8_t byte);
void Serial0_SendArray(uint8_t *array, uint16_t length);
void Serial0_SendString(char *str);
void Serial0_SendNumber(uint32_t num, uint8_t length);
void Serial0_Printf(char *format, ...);
void Serial0_DMA_RxEvent(uint16_t size);
uint8_t Serial0_SendArrayTry(const uint8_t *array, uint16_t length);
uint8_t Serial0_SendStringTry(const char *str);
uint32_t Serial0_GetTxDropCount(void);

/* UART1 专用接口 */
void Serial1_Init(void);
void Serial1_SendByte(uint8_t byte);
void Serial1_SendArray(uint8_t *array, uint16_t length);
void Serial1_SendString(char *str);
void Serial1_SendNumber(uint32_t num, uint8_t length);
void Serial1_Printf(char *format, ...);
void Serial1_DMA_RxEvent(uint16_t size);
uint8_t Serial1_SendArrayTry(const uint8_t *array, uint16_t length);
uint8_t Serial1_SendStringTry(const char *str);
uint32_t Serial1_GetTxDropCount(void);

/* UART2 专用接口 */
void Serial2_Init(void);
void Serial2_SendByte(uint8_t byte);
void Serial2_SendArray(uint8_t *array, uint16_t length);
void Serial2_SendString(char *str);
void Serial2_SendNumber(uint32_t num, uint8_t length);
void Serial2_Printf(char *format, ...);
void Serial2_DMA_RxEvent(uint16_t size);
uint8_t Serial2_SendArrayTry(const uint8_t *array, uint16_t length);
uint8_t Serial2_SendStringTry(const char *str);
uint32_t Serial2_GetTxDropCount(void);

#ifdef __cplusplus
}
#endif

#endif
