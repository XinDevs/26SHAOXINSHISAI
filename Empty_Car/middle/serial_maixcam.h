/**
 * @file    serial_maixcam.h
 * @brief   MaixCam 串口通信模块接口（UART2，115200bps）
 * @details 协议格式与 serial_cmd 一致：@payload\r\n
 *          通过 Serial2(UART2) 与 MaixCam 进行非阻塞收发。
/* =================================================================================
 * MaixCam 视觉识别通信协议总表 (UART2)
 * =================================================================================
 *
 * ---------------------------------------------------------------------------------
 * >>> 发送指令表 (STM32 -> MaixCam)
 * ---------------------------------------------------------------------------------
 * 基础调用: SerialMaixCam_SendCommand("指令");  (底层自动拼接 @ 和 \r\n)
 *
 * 【系统状态控制】
 * -> "start communication" : 进入通信模式 (唤醒摄像头，从默认画面进入等待指令状态)
 * 适用场景: 系统刚上电、或按下小车“一键启动”按键后调用。
 * * -> "exit"                : 退出通信模式 (让摄像头恢复到初始的 HOME 界面)
 * 适用场景: 小车跑完两圈回到圆心，巡检任务彻底结束时调用。
 *
 * 【识别阶段切换】
 * -> "Period1"             : 切换到阶段 1 (导航模式：找“行进箭头”和“黑色十字路口”)
 * 适用场景: 小车在赛道上寻迹行驶时调用。
 * * -> "Period2"             : 切换到阶段 2 (图形模式：找红蓝绿、圆方三角)
 * 适用场景: 小车到达 A~G 任意一个打卡点并停稳后调用。
 *
 * 【触发识别】
 * -> "need look"           : 请求识别结果 (触发单次抓拍，分析当前画面并返回结果)
 * 适用场景: 切换完 Period 后，或定时器每隔几十毫秒周期调用。
 *
 * * ---------------------------------------------------------------------------------
 * <<< 接收协议表 (MaixCam -> STM32)
 * ---------------------------------------------------------------------------------
 * 基础帧格式: @ + 有效载荷(Payload) + \r\n
 *
 * 【阶段 1: 导航与寻迹模式 (Period1 返回值)】
 * -> 识别目标: 行进箭头、黑色十字路口
 * - 检测到箭头朝前       : @forward\r\n
 * - 检测到箭头朝后       : @backward\r\n
 * - 仅检测到十字(无箭头) : @True\r\n
 * - 画面中无有效目标     : @None\r\n
 *
 * 【阶段 2: 环境图形打卡模式 (Period2 返回值)】
 * -> 识别目标: A~G 区域的色块 (红/蓝/绿, 圆/方/三角)
 * - 检测到图形           : @{颜色} {形状}\r\n  (例: @red rectangle\r\n)
 * - 画面中无有效目标     : @None\r\n
 *
 * 【异常状态返回】
 * - 未设置阶段直接请求   : @error: no period set\r\n
 * ================================================================================= */
#ifndef SERIAL_MAIXCAM_H_
#define SERIAL_MAIXCAM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化 MaixCam 串口（UART2）
 */
void SerialMaixCam_Init(void);

/**
 * @brief  处理来自 MaixCam 的一帧数据（非阻塞，应在主循环中周期调用）
 * @return 1 表示有新命令到达，调用者可随后通过 SerialMaixCam_GetCommand 获取；
 *         0 表示无新数据。
 */
uint8_t SerialMaixCam_Process(void);

/**
 * @brief  获取最近一次解析到的命令字符串（去掉 @ 前缀）
 * @return 命令字符串指针，指向模块内部静态缓冲；无命令时返回空串 ""。
 *         下次 Process 调用后内容会被覆盖。
 */
const char *SerialMaixCam_GetCommand(void);

/**
 * @brief  向 MaixCam 发送命令（非阻塞），自动拼接 @ 前缀与 \r\n 后缀
 * @param  cmd 命令字符串（不含 @ 和 \r\n）
 * @return 1 入队成功，0 发送缓冲满（丢弃）
 */
uint8_t SerialMaixCam_SendCommand(const char *cmd);

/**
 * @brief  向 MaixCam 发送格式化命令（非阻塞）
 * @note   用法同 printf，自动拼接 @ 前缀与 \r\n 后缀
 * @return 1 入队成功，0 发送缓冲满（丢弃）
 */
uint8_t SerialMaixCam_SendCommandf(const char *format, ...);

/**
 * @brief  向 MaixCam 发送原始字节（非阻塞），调用者自行处理协议格式
 * @return 1 入队成功，0 发送缓冲满（丢弃）
 */
uint8_t SerialMaixCam_SendRaw(const uint8_t *data, uint16_t length);

/**
 * @brief  获取 UART2 发送丢包计数
 */
uint32_t SerialMaixCam_GetTxDropCount(void);

#ifdef __cplusplus
}
#endif

#endif
