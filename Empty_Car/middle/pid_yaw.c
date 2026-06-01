/**
 * @file    pid_yaw.c
 * @brief   航向角(Yaw)PID控制器实现
 * @details 实现航向环初始化、Yaw单步计算、差速输出、Yaw串级及转向串级控制。
 */
#include "pid.h"
#include "pid_yaw.h"
#include <math.h>

/**
 * @brief  航向外环参数初始化
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
 * @brief Yaw 角度环单步计算（读取 main.c 提供的 g_currentYaw）
 * @note 避免在 PID 中再次调用 Mahony，所有 Mahony 调用应集中在 main 的定时器中进行
 */
float PID_Calculate_YawStep(float targetYaw)
{
    return PID_Calculate_Step(&PID_Yaw, targetYaw, (float)g_pidCurrentYawDeg);
}

/**
 * @brief Yaw外环输出转换为差速值
 */
float PID_Calculate_YawSpeedDiff(float targetYaw, float speedDiffScale)
{
    return PID_Calculate_YawStep(targetYaw) * speedDiffScale;
}

/**
 * @brief  Yaw 外环 + 速度内环串级执行
 * @param  baseSpeedMps 直行基准速度
 * @param  targetYawDeg 目标航向角
 * @param  speedDiffScale 外环到差速的缩放系数
 */
void PID_ExecuteYawCascade(float baseSpeedMps, float targetYawDeg, float speedDiffScale)
{
    float speed_diff;

    if (g_pidRuntime.speedOverrideActive != 0U) {
        /* 覆盖模式下跳过外环, 直接按既定左右目标走速度内环。 */
        g_pidRuntime.grayOuterSpeedDiff = 0.0f;
        g_pidRuntime.yawOuterSpeedDiff = 0.0f;
        g_pidRuntime.targetYawForReport = targetYawDeg;
        PID_ExecuteSpeedInnerLoop();
        return;
    }

    /* 外环输出 speed_diff: 左轮加、右轮减，形成差速纠偏。 */
    speed_diff = PID_Calculate_YawSpeedDiff(targetYawDeg, speedDiffScale);
    g_pidRuntime.yawOuterSpeedDiff = speed_diff;
    g_pidRuntime.grayOuterSpeedDiff = 0.0f;
    g_pidRuntime.targetYawForReport = targetYawDeg;

    g_pidRuntime.targetLeftSpeedMps = baseSpeedMps + speed_diff;
    g_pidRuntime.targetRightSpeedMps = baseSpeedMps - speed_diff;
    PID_ExecuteSpeedInnerLoop();
}

/**
 * @brief  转向模式串级执行(Yaw 外环 + 单轮驱动约束 + 速度内环)
 * @param  targetYawDeg 转向目标航向
 * @param  speedDiffScale 外环到差速的缩放系数
 * @param  turnDir 转向方向: <0 左转, >=0 右转
 */
void PID_ExecuteTurnCascade(float targetYawDeg, float speedDiffScale, int8_t turnDir)
{
    float speed_diff;
    float turn_speed_cmd;

    if (g_pidRuntime.speedOverrideActive != 0U) {
        /* 覆盖模式下直接走速度内环, 不对转向目标进行闭环。 */
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

    /* 转向时将差速绝对值作为单轮目标速度, 另一轮置 0。 */
    turn_speed_cmd = fabsf(speed_diff);
    if (turnDir < 0) {
        g_pidRuntime.targetLeftSpeedMps = 0.0f;
        g_pidRuntime.targetRightSpeedMps = turn_speed_cmd;
    } else {
        g_pidRuntime.targetLeftSpeedMps = turn_speed_cmd;
        g_pidRuntime.targetRightSpeedMps = 0.0f;
    }
    PID_ExecuteSpeedInnerLoop();
}
