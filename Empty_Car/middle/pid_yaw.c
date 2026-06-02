/**
 * @file    pid_yaw.c
  *
  *
 */
#include "pid.h"
#include "pid_yaw.h"
#include <math.h>

/**
  *
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
  *
  *
 */
float PID_Calculate_YawStep(float targetYaw)
{
    return PID_Calculate_Step(&PID_Yaw, targetYaw, (float)g_pidCurrentYawDeg);
}

/**
  *
 */
float PID_Calculate_YawSpeedDiff(float targetYaw, float speedDiffScale)
{
    return PID_Calculate_YawStep(targetYaw) * speedDiffScale;
}

/**
  *
  *
  *
  *
 */
void PID_ExecuteYawCascade(float baseSpeedMps, float targetYawDeg, float speedDiffScale)
{
    float speed_diff;

    if (g_pidRuntime.speedOverrideActive != 0U) {
        /* */
        g_pidRuntime.grayOuterSpeedDiff = 0.0f;
        g_pidRuntime.yawOuterSpeedDiff = 0.0f;
        g_pidRuntime.targetYawForReport = targetYawDeg;
        PID_ExecuteSpeedInnerLoop();
        return;
    }

    /* */
    speed_diff = PID_Calculate_YawSpeedDiff(targetYawDeg, speedDiffScale);
    g_pidRuntime.yawOuterSpeedDiff = speed_diff;
    g_pidRuntime.grayOuterSpeedDiff = 0.0f;
    g_pidRuntime.targetYawForReport = targetYawDeg;

    g_pidRuntime.targetLeftSpeedMps = baseSpeedMps + speed_diff;
    g_pidRuntime.targetRightSpeedMps = baseSpeedMps - speed_diff;
    PID_ExecuteSpeedInnerLoop();
}

/**
  *
  *
  *
  *
 */
void PID_ExecuteTurnCascade(float targetYawDeg, float speedDiffScale, int8_t turnDir)
{
    float speed_diff;
    float turn_speed_cmd;

    if (g_pidRuntime.speedOverrideActive != 0U) {
        /* */
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

    /* */
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
