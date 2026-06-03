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


/* ===== 灰度左半区权重表 =====
 * 只维护 1~8 号左侧灰度权重，9~16 号右侧权重自动镜像取负。
 * 例: sensor[0] 权重为 +0.55f, sensor[15] 权重自动为 -0.55f。
 */
static float gGrayscaleLeftWeights[GW_GRAY_MODULE_CHANNEL_COUNT] = {
    0.55f,  0.5f,  0.35f,  0.3f,
    0.3f,  0.2f,  0.08f,  0.05f
};

static uint8_t g_yJunctionHitCount = 0U;    /* Y岔路口连续检测到计数 */
static uint8_t g_yJunctionMissCount = 0U;   /* Y岔路口连续未检测到计数 */
static uint8_t g_yJunctionStable = 0U;      /* 稳定的Y岔路口状态(0/1) */

/**
 * @brief  获取指定传感器的权重值
 * @param  index 传感器索引(0~15)
 * @retval 左半区返回正值，右半区自动镜像取负
 * @details 左半区[0-7]直接查表，右半区[8-15]镜像取负。
 *          例: index=0 返回 +weight[0], index=15 返回 -weight[0]
 */
static float PID_GetGrayscaleWeight(uint8_t index)
{
    if (index < GW_GRAY_MODULE_CHANNEL_COUNT) {
        return gGrayscaleLeftWeights[index];
    }

    return -gGrayscaleLeftWeights[GW_GRAY_CHANNEL_COUNT - 1U - index];
}

/**
 * @brief  统计指定范围内激活(检测到黑线)的传感器数量
 * @param  startIndex 起始索引
 * @param  endIndex   结束索引
 * @retval 激活的传感器数量
 */
static uint8_t PID_Gray_CountActive(uint8_t startIndex, uint8_t endIndex)
{
    uint8_t i;
    uint8_t count = 0U;

    if (endIndex >= GW_GRAY_CHANNEL_COUNT) {
        endIndex = GW_GRAY_CHANNEL_COUNT - 1U;
    }

    for (i = startIndex; i <= endIndex; i++) {
        if (sensor[i] != 0U) {
            count++;
        }
    }

    return count;
}

/**
 * @brief  灰度巡线环参数初始化
 */
void PID_Grayscale_Init(float kp, float ki, float kd)
{
    PID_Grayscale.OutputMax = 0.8f;
    PID_Grayscale.OutputMin = -0.8f;
    PID_Grayscale.IntegralMax = 2.0f;
    PID_Grayscale.IntegralMin = -2.0f;
    pid_set_gains(&PID_Grayscale, kp, ki, kd);
}

void PID_GrayscaleWeights_Init(const float leftWeights[GW_GRAY_MODULE_CHANNEL_COUNT])
{
    PID_SetGrayscaleLeftWeights(leftWeights);
}

void PID_Grayscale_InitWithWeights(float kp, float ki, float kd,
                                   const float leftWeights[GW_GRAY_MODULE_CHANNEL_COUNT])
{
    PID_Grayscale_Init(kp, ki, kd);
    PID_GrayscaleWeights_Init(leftWeights);
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
 * @brief 基于灰度状态计算加权位置（中心为0）
 * @details 循迹只用中间12路[GRAY_LINE_START, GRAY_LINE_END]，
 *          最左2路和最右2路留给路口判断使用。
 * @retval 位置误差，范围[-1.0, 1.0]
 */
float PID_GetGrayscaleWeightedPosition(void)
{
    uint8_t i;
    float weighted_sum = 0.0f;
    uint8_t active_count = 0U;
    static float last_position = 0.0f;

    /* 只用中间12路加权：sensor[i] 对应 gGrayscaleWeights[i]
     * 约定: sensor[i]=1 表示检测到黑线，sensor[i]=0 表示白地
     */
    for (i = GRAY_LINE_START; i <= GRAY_LINE_END; i++) {
        if (sensor[i] != 0U) {
            weighted_sum += PID_GetGrayscaleWeight(i);
            active_count++;
        }
    }

    if (active_count > 0U) {
        last_position = weighted_sum / (float)active_count;
    }

    return last_position;
}

/**
 * @brief  检测Y型岔路口
 * @details 通过分析16路灰度传感器的激活状态判断是否处于Y型岔路口。
 *          判断逻辑：
 *          1. 总激活数>=5 且 左右分支>=2
 *          2. 且中间4路有2个以上未检测到线
 *
 *          防抖逻辑：
 *          - 连续检测到2次才确认为Y岔路口（防止误触发）
 *          - 连续5次未检测到才取消Y岔路口状态（防止漏检）
 *
 * @retval 1: 检测到稳定的Y型岔路口
 * @retval 0: 未检测到Y型岔路口
 *
 * @note   传感器分布：
 *         - [0-5]  : 左分支区域
 *         - [6-9]  : 中间区域
 *         - [10-15]: 右分支区域
 */
uint8_t PID_Gray_IsYJunction(void)
{
    uint8_t totalCount;        /* 总激活传感器数量 */
    uint8_t centerCount;       /* 中间区域激活数量 */
    uint8_t leftBranchCount;   /* 左分支区域激活数量 */
    uint8_t rightBranchCount;  /* 右分支区域激活数量 */
    uint8_t rawYJunction;      /* 原始判断结果（未防抖） */

    /* 统计各区域激活的传感器数量 */
    totalCount = PID_Gray_CountActive(0U, 15U);   /* 全部16路 */
    centerCount = PID_Gray_CountActive(6U, 9U);   /* 中间4路 */
    leftBranchCount = PID_Gray_CountActive(0U, 5U);  /* 左边6路 */
    rightBranchCount = PID_Gray_CountActive(10U, 15U); /* 右边6路 */

    /* 判断是否为Y型岔路口：
     * 条件: 总数>=5 且 左分支>=2 且 右分支>=2 且 中间4路有2个以上未检测到线
     * 说明: Y岔路口时左右分支有线，中间区域（分岔处）无线 */
    rawYJunction =
        ((totalCount >= 5U) &&
         (leftBranchCount >= 2U) &&
         (rightBranchCount >= 2U) &&
         (centerCount <= 4U)) ? 1U : 0U;

    /* 防抖处理 */
    if (rawYJunction != 0U) {
        /* 检测到Y岔路口，累加命中计数 */
        if (g_yJunctionHitCount < 3U) {
            g_yJunctionHitCount++;
        }
        g_yJunctionMissCount = 0U;  /* 重置未命中计数 */
        /* 连续2次检测到才确认 */
        if (g_yJunctionHitCount >= 2U) {
            g_yJunctionStable = 1U;
        }
    } else {
        /* 未检测到Y岔路口 */
        g_yJunctionHitCount = 0U;   /* 重置命中计数 */
        if (g_yJunctionMissCount < 5U) {
            g_yJunctionMissCount++;
        }
        /* 连续5次未检测到才取消 */
        if (g_yJunctionMissCount >= 5U) {
            g_yJunctionStable = 0U;
        }
    }

    return g_yJunctionStable;
}

/**
 * @brief  重置Y岔路口检测状态
 */
void PID_Gray_ResetYJunctionState(void)
{
    g_yJunctionHitCount = 0U;
    g_yJunctionMissCount = 0U;
    g_yJunctionStable = 0U;
}

/**
 * @brief  检测起终点线（起终点线特征相同）
 * @details 当 16 路灰度中检测到线的传感器数量 >= 12 时，判断为横线。
 * @retval 1: 检测到横线
 * @retval 0: 未检测到横线
 */
uint8_t PID_Gray_IsStartFinishLine(void)
{
    return (PID_Gray_CountActive(0U, 15U) >= 12U) ? 1U : 0U;
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
 */
void PID_ExecuteGrayCascade(float baseSpeedMps, float targetPosition,
                            float speedDiffScale)
{
    float speed_diff;

    if (g_pidRuntime.speedOverrideActive != 0U) {
        /* 覆盖模式下仅保留上报字段更新, 不执行外环纠偏。 */
        g_pidRuntime.grayOuterSpeedDiff = 0.0f;
        g_pidRuntime.yawOuterSpeedDiff = 0.0f;
        g_pidRuntime.targetYawForReport = g_pidCurrentYawDeg;
        PID_ExecuteSpeedInnerLoop();
        return;
    }

    speed_diff = PID_Calculate_GrayscaleSpeedDiff(targetPosition, speedDiffScale);
    g_pidRuntime.grayOuterSpeedDiff = speed_diff;
    g_pidRuntime.yawOuterSpeedDiff = 0.0f;
    g_pidRuntime.targetYawForReport = g_pidCurrentYawDeg;

    g_pidRuntime.targetLeftSpeedMps = baseSpeedMps + speed_diff;
    g_pidRuntime.targetRightSpeedMps = baseSpeedMps - speed_diff;
    PID_ExecuteSpeedInnerLoop();
}

/**
 * @brief 设置灰度 16 路权重
 */
void PID_SetGrayscaleWeights(const float weights[GW_GRAY_CHANNEL_COUNT])
{
    if (weights == NULL) {
        return;
    }

    PID_SetGrayscaleLeftWeights(weights);
}

/**
 * @brief 读取灰度 16 路权重
 */
void PID_GetGrayscaleWeights(float weightsOut[GW_GRAY_CHANNEL_COUNT])
{
    uint8_t i;

    if (weightsOut == NULL) {
        return;
    }

    for (i = 0U; i < GW_GRAY_CHANNEL_COUNT; i++) {
        weightsOut[i] = PID_GetGrayscaleWeight(i);
    }
}

/**
 * @brief  设置左半区8路权重
 */
void PID_SetGrayscaleLeftWeights(const float weights[GW_GRAY_MODULE_CHANNEL_COUNT])
{
    uint8_t i;

    if (weights == NULL) {
        return;
    }

    for (i = 0U; i < GW_GRAY_MODULE_CHANNEL_COUNT; i++) {
        gGrayscaleLeftWeights[i] = weights[i];
    }
}

/**
 * @brief  读取左半区8路权重
 */
void PID_GetGrayscaleLeftWeights(float weightsOut[GW_GRAY_MODULE_CHANNEL_COUNT])
{
    uint8_t i;

    if (weightsOut == NULL) {
        return;
    }

    for (i = 0U; i < GW_GRAY_MODULE_CHANNEL_COUNT; i++) {
        weightsOut[i] = gGrayscaleLeftWeights[i];
    }
}

/**
 * @brief  设置2路兼容权重（仅设置sensor[0]权重，sensor[15]自动取负）
 */
void PID_SetGrayscaleWeights2(const float weights[2])
{
    if (weights == NULL) {
        return;
    }

    gGrayscaleLeftWeights[0] = weights[0];
}

/**
 * @brief  读取2路兼容权重
 */
void PID_GetGrayscaleWeights2(float weightsOut[2])
{
    if (weightsOut == NULL) {
        return;
    }

    weightsOut[0] = gGrayscaleLeftWeights[0];
    weightsOut[1] = -gGrayscaleLeftWeights[0];
}
