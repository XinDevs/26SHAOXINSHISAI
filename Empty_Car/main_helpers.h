/**
 * @file    main_helpers.h
 * @brief   main.c 主循环辅助函数
 * @details 提供转弯方向决策的辅助函数，包括随机转向和摄像头识别转向。
 */
#ifndef MAIN_HELPERS_H
#define MAIN_HELPERS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 摄像头判向等待超时时间(ms)，超时后默认左转 */
#define CAMERA_TURN_TIMEOUT_MS (1000U)

/**
 * @brief  随机决定左转还是右转
 * @retval 1:左转, 0:右转
 */
uint8_t RandomDirection(void);

/**
 * @brief  根据摄像头识别结果决定转向
 * @param  resultCode 摄像头识别结果码
 * @retval 1:左转(绿色/无效), 0:右转(红色)
 */
uint8_t CameraDirection(uint8_t resultCode);

/**
 * @brief  清零起终点横线检测状态
 */
void Main_ResetStartFinishLineState(void);

/**
 * @brief  检查起终点横线通过状态
 * @retval 0:未检测到横线, 1:第一次横线, 2:第二次横线
 */
uint8_t Main_CheckStartFinishLineCrossing(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_HELPERS_H */
