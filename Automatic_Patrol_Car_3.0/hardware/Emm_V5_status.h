/**
 * @file    emm_v5_status.h
 * @brief   Emm_V5 运动状态查询与解析接口
 */
#ifndef EMM_V5_STATUS_H_
#define EMM_V5_STATUS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  清空已解析的电机状态与接收缓存
 */
void EmmV5Status_Reset(void);

/**
 * @brief  喂入 UART1 接收字节给 Emm_V5 状态解析器
 */
void EmmV5Status_OnUart1Byte(uint8_t byte);

/**
 * @brief  主动请求一次运动到位标志状态(S_FLAG)
 */
void EmmV5Status_RequestMotionState(uint8_t addr, uint32_t nowMs);

/**
 * @brief  主动请求一次回零状态(S_OFLAG)
 */
void EmmV5Status_RequestHomeState(uint8_t addr, uint32_t nowMs);

/**
 * @brief  判断目标电机是否已停止运动
 * @retval 1 已停止, 0 仍在运动或尚无有效状态
 */
uint8_t EmmV5Status_IsMotionDone(uint8_t addr);

/**
 * @brief  判断目标电机是否已完成回零
 * @retval 1 已完成, 0 未完成或尚无有效状态
 */
uint8_t EmmV5Status_IsHomeDone(uint8_t addr);

/**
 * @brief  判断是否已收到该电机任意状态
 */
uint8_t EmmV5Status_HasAnyState(uint8_t addr);

/**
 * @brief  获取最近一次解析到的转速(RPM)
 */
int16_t EmmV5Status_GetLastVelRpm(uint8_t addr);

/**
 * @brief  获取最近一次解析到的状态标志原始值
 */
uint8_t EmmV5Status_GetLastFlagRaw(uint8_t addr);

/**
 * @brief  获取最近一次解析到的回零状态标志原始值
 */
uint8_t EmmV5Status_GetLastHomeFlagRaw(uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif
