/**
 * @file    serial_report.h
 * @brief   串口周期状态上报接口
 */
#ifndef SERIAL_REPORT_H_
#define SERIAL_REPORT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t taskId;               /* 当前任务号 */
    float currentYawDeg;          /* 当前航向角(deg) */
    uint16_t pidTrigIntervalMs;   /* ISR 触发控制周期间隔(ms) */
    uint16_t pidHandleIntervalMs; /* 主循环实际处理控制间隔(ms) */
    uint16_t pidPendingCount;     /* 本次处理前积压控制事件数 */
    uint32_t pidTriggerCount;     /* ISR 累计触发控制事件次数 */
    uint32_t pidOverwriteCount;   /* 累计丢周期/覆盖计数 */
} SerialReportSnapshot_t;

/**
 * @brief  处理一次串口周期状态上报事件
 * @param  eventFlag ISR 置位的上报事件标志指针
 * @param  snapshot 主循环填充的状态快照
 * @return 1 表示本次完成上报处理，0 表示无需处理
 */
uint8_t SerialReport_Process(volatile uint8_t *eventFlag,
                             const SerialReportSnapshot_t *snapshot);
void SerialReport_Task1Speed(void);

#ifdef __cplusplus
}
#endif

#endif
