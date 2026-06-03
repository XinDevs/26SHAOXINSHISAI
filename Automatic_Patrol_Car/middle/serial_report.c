/**
 * @file    serial_report.c
 * @brief   串口周期状态上报实现
 * @details 由 main 在 100ms 事件到来时调用，统一打包 @DAT 与 @PIDT 上报帧。
 */

#include "serial_report.h"
#include "ti_msp_dl_config.h"
#include "Serial.h"
#include "encoder.h"
#include "pid.h"
#include "pid_gray.h"
#include "grayscale_sensor.h"
#include <stddef.h>
#include <stdio.h>

static char s_speedLine[320];
static char s_pidTimingLine[96];
static char s_grayBitsLine[GW_GRAY_CHANNEL_COUNT + 1U];
static char s_task1SpeedLine[64];

void SerialReport_Task1Speed(void)
{
    int len;

    len = snprintf(s_task1SpeedLine,
                   sizeof(s_task1SpeedLine),
                   "d: %f, %f\n",
                   (double)encoder_get_left_speed_mps(),
                   (double)encoder_get_right_speed_mps());
    if (len > 0) {
        (void)Serial0_SendStringTry(s_task1SpeedLine);
    }
}

/**
 * @brief  执行一次状态上报
 * @param  eventFlag 上报事件标志(由 ISR 置位，函数内原子清零)
 * @param  snapshot 主循环传入的时序/状态快照
 * @return 1: 本次已处理并完成上报路径；0: 入参无效或当前无事件
 */
uint8_t SerialReport_Process(volatile uint8_t *eventFlag,
                             const SerialReportSnapshot_t *snapshot)
{
    uint8_t i;
    int speedLen;
    float leftSpeed;
    float rightSpeed;
    float leftKdCur;
    float rightKdCur;
    float odomHeadingDeg;
    float grayError;
    EncoderOdometry_t odom;
    PID_RuntimeState_t pidRuntime;

    if ((eventFlag == NULL) || (snapshot == NULL)) {
        return 0U;
    }

    if (*eventFlag == 0U) {
        return 0U;
    }

    /* 原子清事件标志，避免与 ISR 并发读写导致重复上报。 */
    __disable_irq();
    if (*eventFlag == 0U) {
        __enable_irq();
        return 0U;
    }
    *eventFlag = 0U;
    __enable_irq();

    leftSpeed = encoder_get_left_speed_mps();
    rightSpeed = encoder_get_right_speed_mps();
    PID_GetRuntimeState(&pidRuntime);
    encoder_get_odometry_snapshot(&odom);
    odomHeadingDeg = odom.heading_rad * (180.0f / 3.1415926f);
    PID_GetParameters(MOTOR_LEFT, NULL, NULL, &leftKdCur);
    PID_GetParameters(MOTOR_RIGHT, NULL, NULL, &rightKdCur);
    grayError = PID_GetGrayscaleWeightedPosition();

    /* 将 16 路灰度状态转为 16 位字符显示，1号在最左。 */
    for (i = 0U; i < GW_GRAY_CHANNEL_COUNT; i++) {
        s_grayBitsLine[i] = (sensor[i] != 0U) ? '1' : '0';
    }
    s_grayBitsLine[GW_GRAY_CHANNEL_COUNT] = '\0';

    /* @DAT: 运行状态上报(任务号/航向/目标与实测速度/外环差速等)。 */
    speedLen = snprintf(s_speedLine,
                        sizeof(s_speedLine),
                        "@DAT M=%u Y=%.2f G=%s GE=%.3f TL=%.3f AL=%.3f TR=%.3f AR=%.3f DG=%.3f DY=%.3f TY=%.2f AY=%.2f LKD=%.3f RKD=%.3f OD=%.3f X=%.3f YP=%.3f TH=%.2f\r\n",
                        (unsigned int)snapshot->taskId,
                        (float)snapshot->currentYawDeg,
                        s_grayBitsLine,
                        (float)grayError,
                        (float)pidRuntime.targetLeftSpeedMps,
                        (float)leftSpeed,
                        (float)pidRuntime.targetRightSpeedMps,
                        (float)rightSpeed,
                        (float)pidRuntime.grayOuterSpeedDiff,
                        (float)pidRuntime.yawOuterSpeedDiff,
                        (float)pidRuntime.targetYawForReport,
                        (float)snapshot->currentYawDeg,
                        (float)leftKdCur,
                        (float)rightKdCur,
                        (float)odom.total_distance_m,
                        (float)odom.x_m,
                        (float)odom.y_m,
                        (float)odomHeadingDeg);
    if (speedLen > 0) {
        Serial0_SendString(s_speedLine);
    }

    /* @PIDT: 时序诊断上报，保留格式但当前保持与旧版一致(不实际发送)。 */
    speedLen = snprintf(s_pidTimingLine,
                        sizeof(s_pidTimingLine),
                        "@PIDT trig=%u handle=%u cnt=%lu lost=%lu pend=%u\r\n",
                        (unsigned int)snapshot->pidTrigIntervalMs,
                        (unsigned int)snapshot->pidHandleIntervalMs,
                        (unsigned long)snapshot->pidTriggerCount,
                        (unsigned long)snapshot->pidOverwriteCount,
                        (unsigned int)snapshot->pidPendingCount);
    if (speedLen > 0) {
        /* 保持与历史行为一致: 仅格式化，不发送。 */
    }

    return 1U;
}

