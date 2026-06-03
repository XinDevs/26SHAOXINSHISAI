/**
 * @file    car_turn.c
 * @brief   转弯辅助模块（基于速度环控制）
 * @details 通过设置左右轮不同目标速度实现单轮转弯，
 *          支持转弯后检测中间传感器回归循线状态。
 */
#include "car_turn.h"
#include "pid.h"
#include "grayscale_sensor.h"
#include "main.h"

/** @brief 转弯是否激活 (0:未激活, 1:激活) */
static uint8_t s_turnActive = 0U;

/** @brief 转弯开始时间戳 (毫秒) */
static uint32_t s_turnStartMs = 0U;

/** @brief 当前转向方向 (1: 左转, 0: 右转) */
static uint8_t s_turnLeft = 0U;

/**
 * @brief  重置转弯状态
 */
void Turn_Reset(void)
{
    s_turnActive = 0U;
    s_turnStartMs = 0U;
    s_turnLeft = 0U;
    /* 清除直控反转标志，恢复正常速度环 */
    PID_GoalSpeedPair_Set(0.0f, 0.0f);
}

/**
 * @brief  查询转弯是否正在进行
 * @retval 1: 正在转弯, 0: 未转弯
 */
uint8_t Turn_IsRunning(void)
{
    return s_turnActive;
}

/**
 * @brief  启动转弯（速度环方式）
 * @param  turnLeft 1:左转, 0:右转
 * @param  outerSpeed 外侧轮目标速度 (m/s)
 * @param  innerSpeed 内侧轮目标速度 (m/s)
 * @param  nowMs    当前时间戳 (毫秒)
 * @details 左转时：左轮=内侧, 右轮=外侧
 *          右转时：左轮=外侧, 右轮=内侧
 */
void Turn_Start(uint8_t turnLeft, float outerSpeed, float innerSpeed, uint32_t nowMs)
{
    float leftSpeed;
    float rightSpeed;

    s_turnActive = 1U;
    s_turnStartMs = nowMs;
    s_turnLeft = (turnLeft != 0U) ? 1U : 0U;

    if (turnLeft != 0U) {
        leftSpeed  = innerSpeed;   /* 左轮=内侧 */
        rightSpeed = outerSpeed;   /* 右轮=外侧 */
    } else {
        leftSpeed  = outerSpeed;   /* 左轮=外侧 */
        rightSpeed = innerSpeed;   /* 右轮=内侧 */
    }

    /* 先清除反转标志并设置目标速度 */
    PID_GoalSpeedPair_Set(leftSpeed, rightSpeed);

    /* 负速度的轮启用直控反转 */
    if (leftSpeed < 0.0f) {
        PID_SetSingleTargetSpeed(MOTOR_LEFT, leftSpeed);
    }
    if (rightSpeed < 0.0f) {
        PID_SetSingleTargetSpeed(MOTOR_RIGHT, rightSpeed);
    }
}

/**
 * @brief  执行转弯速度环控制
 * @note   每个控制周期(20ms)调用一次
 */
void Turn_Run(void)
{
    PID_ExecuteSpeedInnerLoop();
}

/**
 * @brief  检测转弯后是否回到目标线
 * @param  nowMs   当前时间戳 (毫秒)
 * @param  delayMs 转弯后等待时间 (毫秒)
 * @retval 1: 延时过后检测到目标传感器在线
 * @retval 0: 未检测到或延时未到
 * @details 左转检测 sensor[7]，右转检测 sensor[8]。
 */
uint8_t Turn_IsDone(uint32_t nowMs, uint32_t delayMs)
{
    if (s_turnActive == 0U) {
        return 0U;
    }

    if ((uint32_t)(nowMs - s_turnStartMs) < delayMs) {
        return 0U;
    }

    if (s_turnLeft != 0U) {
        return (sensor[7] != 0U) ? 1U : 0U;
    }

    return (sensor[8] != 0U) ? 1U : 0U;
}

