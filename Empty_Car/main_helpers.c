/**
 * @file    main_helpers.c
 * @brief   main.c 主循环辅助函数实现
 * @details 提供随机转向、摄像头结果判向、起终点横线计数等辅助逻辑。
 */
#include "main_helpers.h"
#include "main.h"
#include "pid.h"
#include "serial_maixcam.h"

/* 伪随机数种子 */
static uint32_t s_turnRandomSeed = 0xA5A55A5AU;
/* 起终点横线状态 */
static uint8_t LineCount = 0U;
static uint8_t LineLatch = 0U;

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

void Main_ResetStartFinishLineState(void)
{
    LineCount = 0U;
    LineLatch = 0U;
}

uint8_t Main_CheckStartFinishLineCrossing(void)
{
    if (PID_Gray_IsStartFinishLine() == 0U) {
        LineLatch = 0U;
        return 0U;
    }

    if (LineLatch == 0U) {
        LineLatch = 1U;
        if (LineCount < 2U) {
            LineCount++;
        }
    }

    return (LineCount >= 2U) ? 2U : 1U;
}
