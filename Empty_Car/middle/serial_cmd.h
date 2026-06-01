/**
 * @file    serial_cmd.h
 * @brief   串口命令解析与在线调参接口
 * @details 负责处理 @payload\r\n 协议命令，向上层提供单入口处理函数。
 */
#ifndef SERIAL_CMD_H_
#define SERIAL_CMD_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  处理一帧串口命令(@payload\r\n)
 * @param  currentYawDeg 当前航向角，供 RST 指令复位运行态时使用
 * @return 1 表示调用者应跳过本轮主循环剩余逻辑，0 表示继续执行
 */
uint8_t SerialCmd_Process(float currentYawDeg);

#ifdef __cplusplus
}
#endif

#endif

