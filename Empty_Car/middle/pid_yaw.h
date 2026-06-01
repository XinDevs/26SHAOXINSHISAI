/**
 * @file    pid_yaw.h
 * @brief   航向角(Yaw)PID控制器 — 头文件
 * @details 声明航向环的初始化、单步计算、差速输出、串级执行及转向接口。
 */
#ifndef __PID_YAW_H
#define __PID_YAW_H

#include <stdint.h>

/**
 * @brief  航向外环参数初始化
 */
void PID_Yaw_Init(float kp, float ki, float kd);
/**
 * @brief  Yaw 环单步计算（读取 g_pidCurrentYawDeg）
 */
float PID_Calculate_YawStep(float targetYaw);
/**
 * @brief  Yaw 环输出换算为差速
 */
float PID_Calculate_YawSpeedDiff(float targetYaw, float speedDiffScale);
/**
 * @brief  Yaw外环 + 速度内环串级执行
 */
void PID_ExecuteYawCascade(float baseSpeedMps, float targetYawDeg, float speedDiffScale);
/**
 * @brief  转向模式执行(Yaw外环 + 单轮速度约束 + 速度内环)
 */
void PID_ExecuteTurnCascade(float targetYawDeg, float speedDiffScale, int8_t turnDir);

#endif /* __PID_YAW_H */
