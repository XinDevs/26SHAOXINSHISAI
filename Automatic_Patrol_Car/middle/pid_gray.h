/**
 * @file    pid_gray.h
 * @brief   灰度巡线PID控制器 — 头文件
 * @details 声明灰度巡线环的初始化、单步计算、差速输出及串级执行接口。
 */
#ifndef __PID_GRAY_H
#define __PID_GRAY_H

#include <stdint.h>
#include "grayscale_sensor.h"

/* ===== 循迹传感器范围 =====
 * 循迹只用中间12路(sensor[2]-sensor[13])，
 * 最左2路和最右2路仅在路口判断时使用。
 */
#define GRAY_LINE_START  2U
#define GRAY_LINE_END    13U
/**
 * @brief  灰度巡线环参数初始化
 * @param  kp 比例系数
 * @param  ki 积分系数
 * @param  kd 微分系数
 */
void PID_Grayscale_Init(float kp, float ki, float kd);
/**
 * @brief  初始化灰度左半区权重
 * @param  leftWeights 8路左半区权重，右半区自动镜像取负
 */
void PID_GrayscaleWeights_Init(const float leftWeights[GW_GRAY_MODULE_CHANNEL_COUNT]);
/**
 * @brief  同时初始化灰度PID参数和左半区权重
 * @param  kp 比例系数
 * @param  ki 积分系数
 * @param  kd 微分系数
 * @param  leftWeights 8路左半区权重
 */
void PID_Grayscale_InitWithWeights(float kp, float ki, float kd,
                                   const float leftWeights[GW_GRAY_MODULE_CHANNEL_COUNT]);
/**
 * @brief  更新 PID 模块使用的当前 Yaw 反馈值（由 main.c 的 10ms IMU 更新调用）
 * @param  yawDeg 当前航向角，单位为度
 */
void PID_UpdateYawFeedback(float yawDeg);
/**
 * @brief  更新 PID 模块使用的当前 Z 轴角速度反馈值（deg/s）
 */
void PID_UpdateGyroZFeedback(float gyroZDps);
/**
 * @brief  获取灰度加权位置
 * @retval 灰度位置误差，中心为0，丢线时保持上一次有效值
 */
float PID_GetGrayscaleWeightedPosition(void);
/**
 * @brief  检测Y型岔路口
 * @retval 1: 检测到稳定的Y型岔路口, 0: 未检测到
 * @note   带防抖：左右分支同时检测到线、中间7~8路无线时1次确认，连续5次未检测取消
 */
uint8_t PID_Gray_IsYJunction(void);

/**
 * @brief  重置Y型岔路口检测的防抖状态
 */
void PID_Gray_ResetYJunctionState(void);

/**
 * @brief  检测任务三/任务五使用的中间宽线路口
 * @retval 1: sensor[6]~sensor[9] 全部检测到线，且 sensor[5]/sensor[10] 至少一路检测到线
 */
uint8_t PID_Gray_IsCenterYJunction(void);

/**
 * @brief  检测起终点线（起终点线特征相同）
 * @retval 1: 激活数量和覆盖跨度符合横线特征, 0: 未检测到
 */
uint8_t PID_Gray_IsStartFinishLine(void);
/**
 * @brief  灰度环单步计算
 * @param  targetPosition 目标位置，通常为0.0f
 * @retval 灰度PID外环输出
 */
float PID_Calculate_GrayscaleStep(float targetPosition);
/**
 * @brief  灰度环输出换算为差速
 * @param  targetPosition 目标位置，通常为0.0f
 * @param  speedDiffScale PID输出到差速的缩放系数
 * @retval 左右轮目标速度差速量
 */
float PID_Calculate_GrayscaleSpeedDiff(float targetPosition, float speedDiffScale);
/**
 * @brief  灰度外环 + 速度内环级联执行
 * @param  baseSpeedMps 巡线基准速度
 * @param  targetPosition 灰度目标位置，通常为0.0f
 * @param  speedDiffScale 外环输出到速度差的缩放系数
 */
void PID_ExecuteGrayCascade(float baseSpeedMps, float targetPosition,
                            float speedDiffScale);
/**
 * @brief  灰度外环 + Z轴角速度阻尼 + 速度内环级联执行
 */
void PID_ExecuteGrayGyroCascade(float baseSpeedMps, float targetPosition,
                                float graySpeedDiffScale,
                                float gyroDampingScale);
/**
 * @brief  设置灰度16路兼容权重
 * @param  weights 16路权重数组，实际使用前8路作为左半区权重
 */
void PID_SetGrayscaleWeights(const float weights[GW_GRAY_CHANNEL_COUNT]);
/**
 * @brief  读取灰度16路完整权重
 * @param  weightsOut 输出16路权重，右半区为左半区镜像负值
 */
void PID_GetGrayscaleWeights(float weightsOut[GW_GRAY_CHANNEL_COUNT]);
/**
 * @brief  设置左半区8路权重
 * @param  weights 8路左半区权重
 */
void PID_SetGrayscaleLeftWeights(const float weights[GW_GRAY_MODULE_CHANNEL_COUNT]);
/**
 * @brief  直接设置巡线使用的左半区6路权重
 * @param  weight2 sensor[2] 对应的左半区权重
 * @param  weight3 sensor[3] 对应的左半区权重
 * @param  weight4 sensor[4] 对应的左半区权重
 * @param  weight5 sensor[5] 对应的左半区权重
 * @param  weight6 sensor[6] 对应的左半区权重
 * @param  weight7 sensor[7] 对应的左半区权重
 */
void PID_SetGrayscaleLeftWeights6(float weight2, float weight3, float weight4,
                                  float weight5, float weight6, float weight7);
/**
 * @brief  读取左半区8路权重
 * @param  weightsOut 输出当前左半区权重
 */
void PID_GetGrayscaleLeftWeights(float weightsOut[GW_GRAY_MODULE_CHANNEL_COUNT]);
/**
 * @brief  设置灰度 2 路兼容权重
 * @param  weights 兼容旧接口的2路权重数组，仅使用weights[0]
 */
void PID_SetGrayscaleWeights2(const float weights[2]);
/**
 * @brief  读取灰度 2 路兼容权重
 * @param  weightsOut 输出最左和最右两路的镜像权重
 */
void PID_GetGrayscaleWeights2(float weightsOut[2]);

#endif /* __PID_GRAY_H */
