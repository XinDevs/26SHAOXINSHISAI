/**
 * @file    pid_gray.h
 * @brief   灰度巡线PID控制器 — 头文件
 * @details 声明灰度巡线环的初始化、单步计算、差速输出及串级执行接口。
 */
#ifndef __PID_GRAY_H
#define __PID_GRAY_H

#include <stdint.h>
#include "grayscale_sensor.h"

/**
 * @brief  灰度巡线环参数初始化
 */
void PID_Grayscale_Init(float kp, float ki, float kd);
/**
 * @brief  更新 PID 模块使用的当前 Yaw 反馈值（由 main.c 的 10ms IMU 更新调用）
 */
void PID_UpdateYawFeedback(float yawDeg);
/**
 * @brief  获取灰度加权位置
 */
float PID_GetGrayscaleWeightedPosition(void);
/**
 * @brief  检测Y型岔路口
 * @retval 1: 检测到稳定的Y型岔路口, 0: 未检测到
 * @note   带防抖：连续3次检测确认，连续5次未检测取消
 */
uint8_t PID_Gray_IsYJunction(void);
/**
 * @brief  灰度环单步计算
 */
float PID_Calculate_GrayscaleStep(float targetPosition);
/**
 * @brief  灰度环输出换算为差速
 */
float PID_Calculate_GrayscaleSpeedDiff(float targetPosition, float speedDiffScale);
/**
 * @brief  灰度外环 + 速度内环级联执行
 */
void PID_ExecuteGrayCascade(float baseSpeedMps, float targetPosition,
                            float speedDiffScale, float yawForReportDeg);
void PID_SetGrayscaleWeights(const float weights[GW_GRAY_CHANNEL_COUNT]);
void PID_GetGrayscaleWeights(float weightsOut[GW_GRAY_CHANNEL_COUNT]);
void PID_SetGrayscaleLeftWeights(const float weights[GW_GRAY_MODULE_CHANNEL_COUNT]);
void PID_GetGrayscaleLeftWeights(float weightsOut[GW_GRAY_MODULE_CHANNEL_COUNT]);
/**
 * @brief  设置灰度 2 路兼容权重
 */
void PID_SetGrayscaleWeights2(const float weights[2]);
/**
 * @brief  读取灰度 2 路兼容权重
 */
void PID_GetGrayscaleWeights2(float weightsOut[2]);

#endif /* __PID_GRAY_H */
