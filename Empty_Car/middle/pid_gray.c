/**
 * @file    pid_gray.c
 * @brief   灰度巡线PID控制器实现
 * @details 实现灰度加权位置计算、巡线PID单步、差速输出及外环+速度内环串级控制。
 */
#include "pid.h"
#include "pid_gray.h"
#include "grayscale_sensor.h"
#include "main.h"
#include <math.h>

/* ===== 灰度 2 路权重表 =====
 * 约定: gGrayscaleWeights2[i] 与 sensor[i] 一一对应
 * 含义: 用于将 2 路数字灰度映射为"横向位置误差"(中心约为 0)
 * 布局: [0]-左(正值, 偏左时正偏) [1]-右(负值, 偏右时负偏)
 */
static float gGrayscaleWeights2[2] = {
    1.0f, -1.0f
};
/**
 * @brief  灰度巡线环参数初始化
 */
void PID_Grayscale_Init(float kp, float ki, float kd)
{
    PID_Grayscale.OutputMax = 0.5f;
    PID_Grayscale.OutputMin = -0.5f;
    PID_Grayscale.IntegralMax = 3.0f;
    PID_Grayscale.IntegralMin = -3.0f;
    pid_set_gains(&PID_Grayscale, kp, ki, kd);
}

/**
 * @brief  更新 PID 模块使用的 Yaw 反馈值
 * @details 建议在 main 的 IMU 更新周期(当前为 10ms)调用该接口。
 */
void PID_UpdateYawFeedback(float yawDeg)
{
    g_pidCurrentYawDeg = yawDeg;
}

/**
 * @brief 基于2路灰度状态计算加权位置（中心为0）
 * @retval 位置误差，范围[-1.0, 1.0]
 */
float PID_GetGrayscaleWeightedPosition(void)
{
    uint8_t i;
    float weighted_sum = 0.0f;
    uint8_t active_count = 0U;
    static float last_position = 0.0f;

    /* 2路直接加权：sensor[i] 对应 gGrayscaleWeights2[i]
     * 约定: sensor[i]=1 表示检测到黑线，sensor[i]=0 表示白地
     */
    for (i = 0U; i < 2U; i++) {
        if (sensor[i] != 0U) {
            weighted_sum += gGrayscaleWeights2[i];
            active_count++;
        }
    }

    if (active_count > 0U) {
        last_position = weighted_sum / (float)active_count;
    }

    return last_position;
}

/**
 * @brief 灰度循迹 PID 单步计算
 * @param targetPosition 目标中心位置，建议为0.0f
 */
float PID_Calculate_GrayscaleStep(float targetPosition)
{
    float actualPosition = PID_GetGrayscaleWeightedPosition();
    return PID_Calculate_Step(&PID_Grayscale, targetPosition, actualPosition);
}

/**
 * @brief 灰度外环输出转换为差速值
 */
float PID_Calculate_GrayscaleSpeedDiff(float targetPosition, float speedDiffScale)
{
    return PID_Calculate_GrayscaleStep(targetPosition) * speedDiffScale;
}

/**
 * @brief  灰度外环 + 速度内环串级执行
 * @param  baseSpeedMps 巡线基准速度
 * @param  targetPosition 灰度目标位置(通常为0, 表示居中)
 * @param  speedDiffScale 外环到差速的缩放系数
 * @param  yawForReportDeg 上报用航向角(不参与本函数控制计算)
 */
void PID_ExecuteGrayCascade(float baseSpeedMps, float targetPosition,
                            float speedDiffScale, float yawForReportDeg)
{
    float speed_diff;

    if (g_pidRuntime.speedOverrideActive != 0U) {
        /* 覆盖模式下仅保留上报字段更新, 不执行外环纠偏。 */
        g_pidRuntime.grayOuterSpeedDiff = 0.0f;
        g_pidRuntime.yawOuterSpeedDiff = 0.0f;
        g_pidRuntime.targetYawForReport = yawForReportDeg;
        PID_ExecuteSpeedInnerLoop();
        return;
    }

    speed_diff = PID_Calculate_GrayscaleSpeedDiff(targetPosition, speedDiffScale);
    g_pidRuntime.grayOuterSpeedDiff = speed_diff;
    g_pidRuntime.yawOuterSpeedDiff = 0.0f;
    g_pidRuntime.targetYawForReport = yawForReportDeg;

    g_pidRuntime.targetLeftSpeedMps = baseSpeedMps + speed_diff;
    g_pidRuntime.targetRightSpeedMps = baseSpeedMps - speed_diff;
    PID_ExecuteSpeedInnerLoop();
}

/**
 * @brief 设置灰度 2 路权重
 */
void PID_SetGrayscaleWeights2(const float weights[2])
{
    if (weights == NULL) {
        return;
    }
    gGrayscaleWeights2[0] = weights[0];
    gGrayscaleWeights2[1] = weights[1];
}

/**
 * @brief 读取灰度 2 路权重
 */
void PID_GetGrayscaleWeights2(float weightsOut[2])
{
    if (weightsOut == NULL) {
        return;
    }
    weightsOut[0] = gGrayscaleWeights2[0];
    weightsOut[1] = gGrayscaleWeights2[1];
}
