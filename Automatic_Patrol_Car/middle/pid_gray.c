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

static uint8_t g_yJunctionMissCount = 0U;   /* Y岔路口连续未检测到计数 */
static uint8_t g_yJunctionHitCount = 0U;    /* Y岔路口连续检测到计数 */
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
 * @param  kp 比例系数，决定纠偏响应强度
 * @param  ki 积分系数，用于消除长期偏差
 * @param  kd 微分系数，用于抑制快速变化和超调
 * @details 同时设置灰度外环的输出限幅和积分限幅，避免差速输出过大。
 */
void PID_Grayscale_Init(float kp, float ki, float kd)
{
    PID_Grayscale.OutputMax = 1.5f;
    PID_Grayscale.OutputMin = -1.5f;
    PID_Grayscale.IntegralMax = 2.0f;
    PID_Grayscale.IntegralMin = -2.0f;
    pid_set_gains(&PID_Grayscale, kp, ki, kd);
}

/**
 * @brief  初始化灰度左半区权重表
 * @param  leftWeights 8路左半区权重数组，右半区权重由镜像自动取负
 */
void PID_GrayscaleWeights_Init(const float leftWeights[GW_GRAY_MODULE_CHANNEL_COUNT])
{
    PID_SetGrayscaleLeftWeights(leftWeights);
}

/**
 * @brief  同时初始化灰度PID参数和权重表
 * @param  kp 比例系数
 * @param  ki 积分系数
 * @param  kd 微分系数
 * @param  leftWeights 8路左半区权重数组
 */
void PID_Grayscale_InitWithWeights(float kp, float ki, float kd,
                                   const float leftWeights[GW_GRAY_MODULE_CHANNEL_COUNT])
{
    PID_Grayscale_Init(kp, ki, kd);
    PID_GrayscaleWeights_Init(leftWeights);
}

/**
 * @brief  更新 PID 模块使用的 Yaw 反馈值
 * @param  yawDeg 当前航向角，单位为度
 * @details 建议在 main 的 IMU 更新周期(当前为 10ms)调用该接口。
 */
void PID_UpdateYawFeedback(float yawDeg)
{
    g_pidCurrentYawDeg = yawDeg;
}

/**
 * @brief  更新 PID 模块使用的 Z 轴角速度反馈值
 * @param  gyroZDps 当前 Z 轴角速度，单位为 deg/s
 */
void PID_UpdateGyroZFeedback(float gyroZDps)
{
    g_pidCurrentGyroZDps = gyroZDps;
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
            /* 多路同时压线时取权重平均，降低单个传感器抖动的影响。 */
            weighted_sum += PID_GetGrayscaleWeight(i);
            active_count++;
        }
    }

    if (active_count > 0U) {
        last_position = weighted_sum / (float)active_count;
    }

    /* 丢线时返回上一次有效位置，避免PID输入突变。 */
    return last_position;
}

// /**
//  * @brief  检测Y型岔路口
//  * @details 通过分析16路灰度传感器的激活状态判断是否处于Y型岔路口。
//  *          判断逻辑：
//  *          1. 总激活数>=5 且 左右分支>=2
//  *          2. 中间4路激活数不超过阈值(当前阈值为4)
//  *
//  *          防抖逻辑：
//  *          - 连续检测到2次才确认为Y岔路口（防止误触发）
//  *          - 连续5次未检测到才取消Y岔路口状态（防止漏检）
//  *
//  * @retval 1: 检测到稳定的Y型岔路口
//  * @retval 0: 未检测到Y型岔路口
//  *
//  * @note   传感器分布：
//  *         - [2-5]  : 左分支区域
//  *         - [7-8]  : 中间区域
//  *         - [10-13]: 右分支区域
//  */
// uint8_t PID_Gray_IsYJunction(void)
// {
//     uint8_t totalCount;        /* 总激活传感器数量 */
//     uint8_t centerCount;       /* 中间区域激活数量 */
//     uint8_t leftBranchCount;   /* 左分支区域激活数量 */
//     uint8_t rightBranchCount;  /* 右分支区域激活数量 */
//     uint8_t rawYJunction;      /* 原始判断结果（未防抖） */

//     /* 统计各区域激活的传感器数量 */
//     totalCount = PID_Gray_CountActive(0U, 15U);   /* 全部16路 */
//     centerCount = PID_Gray_CountActive(7U, 8U);    /* 中间2路 (7、8号) */
//     leftBranchCount = PID_Gray_CountActive(2U, 5U);  /* 左边4路 (2~5号) */
//     rightBranchCount = PID_Gray_CountActive(10U, 13U); /* 右边4路 (10~13号) */

//     /* 判断是否为Y型岔路口：
//      * 条件: 总数>=4 且 左分支>=2 且 右分支>=2 且 中间2路激活数不超过阈值。
//      * 说明: Y岔路口时左右分支同时有线，因此左右两侧命中数量是主要判据。 */
//     rawYJunction =
//         ((totalCount >= 4U) &&
//          (leftBranchCount >= 2U) &&
//          (rightBranchCount >= 2U) &&
//          (centerCount <= 2U)) ? 1U : 0U;

//     /* 防抖处理 */
//     if (rawYJunction != 0U) {
//         /* 检测到Y岔路口，累加命中计数 */
//         if (g_yJunctionHitCount < 3U) {
//             g_yJunctionHitCount++;
//         }
//         g_yJunctionMissCount = 0U;  /* 重置未命中计数 */
//         /* 连续2次检测到才确认 */
//         if (g_yJunctionHitCount >= 2U) {
//             g_yJunctionStable = 1U;
//         }
//     } else {
//         /* 未检测到Y岔路口 */
//         g_yJunctionHitCount = 0U;   /* 重置命中计数 */
//         if (g_yJunctionMissCount < 5U) {
//             g_yJunctionMissCount++;
//         }
//         /* 连续5次未检测到才取消 */
//         if (g_yJunctionMissCount >= 5U) {
//             g_yJunctionStable = 0U;
//         }
//     }

//     return g_yJunctionStable;
// }
/**
 * @brief  检测Y型岔路口
 * @details 通过分析16路灰度传感器的激活状态判断是否处于Y型岔路口。
 *          判断逻辑：
 *          1. 总激活数>=3 且 左分支>=1 且 右分支>=1
 *          2. 中间2路(sensor[7-8])均未激活
 *
 *          防抖逻辑：
 *          - 检测到1次即确认
 *          - 连续5次未检测到才取消（防止转弯过程中误取消）
 *
 * @retval 1: 检测到Y型岔路口
 * @retval 0: 未检测到Y型岔路口
 *
 * @note   传感器分布：
 *         - [2-5]  : 左分支区域
 *         - [7-8]  : 中间区域
 *         - [10-13]: 右分支区域
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
    centerCount = PID_Gray_CountActive(7U, 8U);    /* 中间2路 (7、8号) */
    leftBranchCount = PID_Gray_CountActive(2U, 5U);  /* 左边4路 (2~5号) */
    rightBranchCount = PID_Gray_CountActive(10U, 13U); /* 右边4路 (10~13号) */

    /* 判断是否为Y型岔路口：
     * 条件: 总数>=3 且 左分支>=1 且 右分支>=1 且 中间2路无线。
     * 说明: Y岔路口时左右分支同时有线，中间无线说明车正对岔口。 */
    rawYJunction =
        ((totalCount >= 3U) &&
         (leftBranchCount >= 1U) &&
         (rightBranchCount >= 1U) &&
         (centerCount == 0U)) ? 1U : 0U;

    /* 防抖：检测到1次即确认，未检测到连续5次才取消 */
    if (rawYJunction != 0U) {
        g_yJunctionHitCount = 0U;
        g_yJunctionMissCount = 0U;
        g_yJunctionStable = 1U;
    } else {
        g_yJunctionHitCount = 0U;
        if (g_yJunctionMissCount < 5U) {
            g_yJunctionMissCount++;
        }
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
    g_yJunctionMissCount = 0U;
    g_yJunctionHitCount = 0U;
    g_yJunctionStable = 0U;
}

/**
 * @brief  检测中间全白路口
 * @details sensor[6]~sensor[9] 四路必须全部未检测到黑线（中间全白），
 *          且 sensor[5] 或 sensor[10] 至少一路未检测到黑线时，
 *          判断为路口。用于循迹测试任务的停车判断。
 */
uint8_t PID_Gray_IsCenterYJunction(void)
{
    uint8_t centerCount;
    uint8_t edgeCount;

    centerCount = PID_Gray_CountActive(6U, 9U);     /* 中间4路 */
    edgeCount = (uint8_t)(PID_Gray_CountActive(5U, 5U) +
                          PID_Gray_CountActive(10U, 10U)); /* sensor[5]和[10] */

    return ((centerCount == 0U) && (edgeCount < 2U)) ? 1U : 0U;
}

/**
 * @brief  检测起终点线（起终点线特征相同）
 * @details 当 16 路灰度中检测到黑线的传感器数量 >= 11 时，判断为横线。
 * @retval 1: 检测到横线
 * @retval 0: 未检测到横线
 */
uint8_t PID_Gray_IsStartFinishLine(void)
{
    return (PID_Gray_CountActive(0U, 15U) >= 11U) ? 1U : 0U;
}

/**
 * @brief 灰度循迹 PID 单步计算
 * @param targetPosition 目标中心位置，建议为0.0f
 * @retval PID外环输出，正负方向由灰度权重符号决定
 */
float PID_Calculate_GrayscaleStep(float targetPosition)
{
    float actualPosition = PID_GetGrayscaleWeightedPosition();
    return PID_Calculate_Step(&PID_Grayscale, targetPosition, actualPosition);
}

/**
 * @brief 灰度外环输出转换为差速值
 * @param targetPosition 目标中心位置
 * @param speedDiffScale PID输出到左右轮差速的缩放系数
 * @retval 用于叠加到左右目标速度的差速量
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

    /* 左右轮反向叠加差速：一侧加速、另一侧减速，从而产生转向。 */
    g_pidRuntime.targetLeftSpeedMps = baseSpeedMps + speed_diff;
    g_pidRuntime.targetRightSpeedMps = baseSpeedMps - speed_diff;
    PID_ExecuteSpeedInnerLoop();
}

/**
 * @brief  灰度外环 + Z轴角速度阻尼 + 速度内环串级执行
 * @param  baseSpeedMps 巡线基准速度
 * @param  targetPosition 灰度目标位置(通常为0, 表示居中)
 * @param  graySpeedDiffScale 灰度外环到差速的缩放系数
 * @param  gyroDampingScale Z轴角速度阻尼系数，单位约为(m/s)/(deg/s)
 */
void PID_ExecuteGrayGyroCascade(float baseSpeedMps, float targetPosition,
                                float graySpeedDiffScale,
                                float gyroDampingScale)
{
    float gray_diff;
    float gyro_diff;
    float speed_diff;

    if (g_pidRuntime.speedOverrideActive != 0U) {
        g_pidRuntime.grayOuterSpeedDiff = 0.0f;
        g_pidRuntime.yawOuterSpeedDiff = 0.0f;
        g_pidRuntime.targetYawForReport = g_pidCurrentYawDeg;
        PID_ExecuteSpeedInnerLoop();
        return;
    }

    gray_diff = PID_Calculate_GrayscaleSpeedDiff(targetPosition, graySpeedDiffScale);
    gyro_diff = -g_pidCurrentGyroZDps * gyroDampingScale;
    speed_diff = gray_diff + gyro_diff;

    g_pidRuntime.grayOuterSpeedDiff = gray_diff;
    g_pidRuntime.yawOuterSpeedDiff = gyro_diff;
    g_pidRuntime.targetYawForReport = g_pidCurrentYawDeg;

    g_pidRuntime.targetLeftSpeedMps = baseSpeedMps + speed_diff;
    g_pidRuntime.targetRightSpeedMps = baseSpeedMps - speed_diff;
    PID_ExecuteSpeedInnerLoop();
}

/**
 * @brief 设置灰度 16 路权重
 * @param weights 兼容16路接口的权重数组，实际读取前8路作为左半区权重
 * @details 右半区权重不单独存储，由左半区权重按传感器位置镜像取负。
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
 * @param weightsOut 输出16路完整权重，右半区为左半区镜像负值
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
 * @param  weights 8路左半区权重，索引从最左侧传感器开始
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
 * @brief  直接设置巡线使用的左半区6路权重
 * @param  weight2 sensor[2] 对应的左半区权重
 * @param  weight3 sensor[3] 对应的左半区权重
 * @param  weight4 sensor[4] 对应的左半区权重
 * @param  weight5 sensor[5] 对应的左半区权重
 * @param  weight6 sensor[6] 对应的左半区权重
 * @param  weight7 sensor[7] 对应的左半区权重
 * @details 仅更新 sensor[2]~sensor[7]。sensor[8]~sensor[13] 的权重由镜像自动取负。
 */
void PID_SetGrayscaleLeftWeights6(float weight2, float weight3, float weight4,
                                  float weight5, float weight6, float weight7)
{
    gGrayscaleLeftWeights[2] = weight2;
    gGrayscaleLeftWeights[3] = weight3;
    gGrayscaleLeftWeights[4] = weight4;
    gGrayscaleLeftWeights[5] = weight5;
    gGrayscaleLeftWeights[6] = weight6;
    gGrayscaleLeftWeights[7] = weight7;
}

/**
 * @brief  读取左半区8路权重
 * @param  weightsOut 输出当前左半区权重表
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
 * @param  weights 兼容旧接口的2路权重数组，仅使用weights[0]
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
 * @param  weightsOut 输出最左和最右两路的镜像权重
 */
void PID_GetGrayscaleWeights2(float weightsOut[2])
{
    if (weightsOut == NULL) {
        return;
    }

    weightsOut[0] = gGrayscaleLeftWeights[0];
    weightsOut[1] = -gGrayscaleLeftWeights[0];
}
