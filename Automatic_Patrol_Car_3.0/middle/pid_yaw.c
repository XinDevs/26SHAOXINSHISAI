/**
 * @file    pid_yaw.c
 * @brief   航向保持环 PID 控制器实现
 * @details 基于 ICM42688 的航向角进行闭环控制，支持直行保持和定点转弯。
 */
#include "pid.h"
#include "pid_yaw.h"
#include <math.h>

/**
 * @brief  航向环参数初始化
 * @param  kp 比例系数
 * @param  ki 积分系数
 * @param  kd 微分系数
 */
void PID_Yaw_Init(float kp, float ki, float kd)
{
    PID_Yaw.OutputMax = 0.5f;
    PID_Yaw.OutputMin = -0.5f;
    PID_Yaw.IntegralMax = 10.0f;
    PID_Yaw.IntegralMin = -10.0f;
    pid_set_gains(&PID_Yaw, kp, ki, kd);
}

/**
 * @brief  航向环单步计算
 * @param  targetYaw 目标航向角（度）
 * @retval PID 输出值
 */
float PID_Calculate_YawStep(float targetYaw)
{
    return PID_Calculate_Step(&PID_Yaw, targetYaw, (float)g_pidCurrentYawDeg);
}

/**
 * @brief  航向环输出转换为差速值
 * @param  targetYaw     目标航向角（度）
 * @param  speedDiffScale 缩放系数
 * @retval 差速值（m/s）
 */
float PID_Calculate_YawSpeedDiff(float targetYaw, float speedDiffScale)
{
    return PID_Calculate_YawStep(targetYaw) * speedDiffScale;
}

/**
 * @brief  航向外环 + 速度内环串级执行（直行保持）
 * @param  baseSpeedMps   基准速度（m/s）
 * @param  targetYawDeg   目标航向角（度）
 * @param  speedDiffScale 外环到差速的缩放系数
 * @details 覆盖模式下跳过外环，直接执行速度内环。
 */
void PID_ExecuteYawCascade(float baseSpeedMps, float targetYawDeg, float speedDiffScale)
{
    float speed_diff;

    if (g_pidRuntime.speedOverrideActive != 0U) {
        /* 覆盖模式下跳过外环，直接走速度内环 */
        g_pidRuntime.grayOuterSpeedDiff = 0.0f;
        g_pidRuntime.yawOuterSpeedDiff = 0.0f;
        g_pidRuntime.targetYawForReport = targetYawDeg;
        PID_ExecuteSpeedInnerLoop();
        return;
    }

    /* 计算航向差速 */
    speed_diff = PID_Calculate_YawSpeedDiff(targetYawDeg, speedDiffScale);
    g_pidRuntime.yawOuterSpeedDiff = speed_diff;
    g_pidRuntime.grayOuterSpeedDiff = 0.0f;
    g_pidRuntime.targetYawForReport = targetYawDeg;

    g_pidRuntime.targetLeftSpeedMps = baseSpeedMps + speed_diff;
    g_pidRuntime.targetRightSpeedMps = baseSpeedMps - speed_diff;
    PID_ExecuteSpeedInnerLoop();
}

/**
 * @brief  航向外环 + 速度内环串级执行（定点转弯）
 * @param  targetYawDeg   目标航向角（度）
 * @param  speedDiffScale 缩放系数
 * @param  turnDir        转弯方向: 正=右转, 负=左转
 * @details 转弯时一轮停止、另一轮以差速值转动，实现原地转向。
 */
void PID_ExecuteTurnCascade(float targetYawDeg, float speedDiffScale, int8_t turnDir)
{
    float speed_diff;
    float turn_speed_cmd;

    if (g_pidRuntime.speedOverrideActive != 0U) {
        /* 覆盖模式下跳过外环 */
        g_pidRuntime.grayOuterSpeedDiff = 0.0f;
        g_pidRuntime.yawOuterSpeedDiff = 0.0f;
        g_pidRuntime.targetYawForReport = targetYawDeg;
        PID_ExecuteSpeedInnerLoop();
        return;
    }

    speed_diff = PID_Calculate_YawSpeedDiff(targetYawDeg, speedDiffScale);
    g_pidRuntime.yawOuterSpeedDiff = speed_diff;
    g_pidRuntime.grayOuterSpeedDiff = 0.0f;
    g_pidRuntime.targetYawForReport = targetYawDeg;

    /* 根据转弯方向分配左右轮速度 */
    turn_speed_cmd = fabsf(speed_diff);
    if (turnDir < 0) {
        g_pidRuntime.targetLeftSpeedMps = 0.0f;       /* 左轮停止 */
        g_pidRuntime.targetRightSpeedMps = turn_speed_cmd; /* 右轮转动 */
    } else {
        g_pidRuntime.targetLeftSpeedMps = turn_speed_cmd;  /* 左轮转动 */
        g_pidRuntime.targetRightSpeedMps = 0.0f;       /* 右轮停止 */
    }
    PID_ExecuteSpeedInnerLoop();
}
