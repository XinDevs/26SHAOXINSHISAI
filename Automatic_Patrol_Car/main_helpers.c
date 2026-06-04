/**
 * @file    main_helpers.c
 * @brief   main.c 主循环辅助函数实现
 * @details 提供随机转向、摄像头结果判向、起终点横线计数等辅助逻辑。
 */
#include "main_helpers.h"
#include "main.h"
#include "encoder.h"
#include "pid.h"
#include "serial_maixcam.h"



static uint32_t s_turnRandomSeed = 0xA5A55A5AU; /* 伪随机数种子(Xorshift32) */
static uint8_t LineCount = 0U;    /* 起终点横线累计通过次数(0/1/2) */
static uint8_t LineLatch = 0U;    /* 横线锁存标志，防止同一次通过重复计数 */
static uint8_t s_finishAdvanceActive = 0U; /* 终点前进状态(0=未激活, 1=前进中) */

/**
 * @brief  随机决定左转还是右转
 * @retval 1:左转, 0:右转
 */
uint8_t RandomDirection(void)
{
    uint32_t mixed;

    /* Xorshift32 伪随机数生成 */
    s_turnRandomSeed ^= (SysMs + 0x9E3779B9U);
    s_turnRandomSeed ^= (s_turnRandomSeed << 13);
    s_turnRandomSeed ^= (s_turnRandomSeed >> 17);
    s_turnRandomSeed ^= (s_turnRandomSeed << 5);

    mixed = s_turnRandomSeed ^ (s_turnRandomSeed >> 16) ^ (SysMs >> 3);
    return ((mixed & 1U) != 0U) ? 1U : 0U;
}

/**
 * @brief  根据摄像头识别结果决定转向
 * @param  resultCode 摄像头识别结果码
 * @retval 1:左转(绿色/无效), 0:右转(红色)
 */
uint8_t CameraDirection(uint8_t resultCode)
{
    switch (resultCode) {
        case SERIAL_MAIXCAM_RESULT_RED_CIRCLE:
        case SERIAL_MAIXCAM_RESULT_RED_SQUARE:
            return 0U;

        case SERIAL_MAIXCAM_RESULT_GREEN_CIRCLE:
        case SERIAL_MAIXCAM_RESULT_GREEN_SQUARE:
        default:
            return 1U;
    }
}

/**
 * @brief  重置起终点横线检测状态
 * @note   任务开始或重新开始时调用
 */
void Main_ResetStartFinishLineState(void)
{
    LineCount = 0U;
    LineLatch = 0U;
}

/**
 * @brief  检查起终点横线通过状态（带锁存防重复计数）
 * @retval 0: 未检测到横线
 * @retval 1: 第一次经过横线（起点）
 * @retval 2: 第二次经过横线（终点）
 * @details 第一次横线记录为起点，第二次横线才判定为终点。
 *          同一次压线期间通过 LineLatch 防止重复计数。
 */
uint8_t Main_CheckStartFinishLineCrossing(void)
{
    /* 未检测到横线：清除锁存，返回 0 */
    if (PID_Gray_IsStartFinishLine() == 0U) {
        LineLatch = 0U;
        return 0U;
    }

    /* 检测到横线：仅在锁存从 0→1 时计数（防抖） */
    if (LineLatch == 0U) {
        LineLatch = 1U;
        if (LineCount < 2U) {
            LineCount++;
        }
    }

    return (LineCount >= 2U) ? 2U : 1U;
}

/**
 * @brief  重置终点前进状态
 */
void Main_ResetFinishAdvanceState(void)
{
    s_finishAdvanceActive = 0U;
}

/**
 * @brief  终点前进：检测到第二次横线后继续前行指定距离再停车
 * @param  traceSpeed 循迹速度(m/s)
 * @param  dutyReady  输出标志指针，置1表示需要更新电机
 * @retval 1: 已完成前进，可以结束任务
 * @retval 0: 仍在前进中
 * @details 首次调用时重置里程计，之后每帧检查行驶距离，
 *          达到 FINISH_ADVANCE_DISTANCE_M(0.30m) 后返回完成。
 */
uint8_t Main_RunFinishAdvance(float traceSpeed, uint8_t *dutyReady)
{
    /* 首次调用：初始化 */
    if (s_finishAdvanceActive == 0U) {
        encoder_reset_odometry();
        PID_ResetAll();
        s_finishAdvanceActive = 1U;
    }

    /* 行驶距离达标：完成 */
    if (encoder_get_total_distance_m() >= FINISH_ADVANCE_DISTANCE_M) {
        s_finishAdvanceActive = 0U;
        return 1U;
    }

    /* 速度环直行，不再读取灰度位置修正 */
    PID_GoalSpeedPair_Set(traceSpeed, traceSpeed);
    PID_ExecuteSpeedInnerLoop();
    *dutyReady = 1U;
    return 0U;
}
