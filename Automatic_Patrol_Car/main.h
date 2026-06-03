/**
 * @file    main.h
 * @brief   主程序全局参数与接口声明
 */
#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PID 等待执行计数上限，用于限制主循环中待处理控制周期的累积。 */
#define PID_PENDING_COUNT_MAX  (100U)

/* 循迹基础速度，单位 m/s。 */
#define BASE_LINE_SPEED        (0.3f)
/* 灰度循迹外环输出到左右轮差速的缩放系数。 */
#define LINE_SPEED_DIFF_SCALE  (1.0f)

/* 直行控制基础速度，单位 m/s。 */
#define BASE_STRAIGHT_SPEED    (0.4f)
/* 航向外环允许输出的最大左右轮差速，单位 m/s。 */
#define MAX_YAW_SPEED_DIFF     (1.0f)

/* 航向转弯到位误差阈值，单位度。 */
#define TURN_DONE_ERR_DEG      (5.0f)
/* 单次目标转弯角度，单位度。 */
#define TURN_ANGLE_DEG         (90.0f)

/* 路口转向外侧轮目标速度，单位 m/s。 */
#define TURN_OUTER_SPEED       (0.3f)
/* 路口转向内侧轮目标速度，单位 m/s。 */
#define TURN_INNER_SPEED       (0.06f)
/* 转弯开始后延迟检测回线的时间，单位 ms。 */
#define TURN_LINE_DETECT_DELAY_MS (200U)
/* 原地找线/转向阶段延时，单位 ms。 */
#define SPIN_LINE_DELAY_MS     (500U)

/* 摄像头判向等待超时时间(ms) */
#define CAMERA_TURN_TIMEOUT_MS (300U)
/* 当前航向角，由 IMU 更新，单位度。 */
extern volatile float CurrentYaw;
/* 系统毫秒计时。 */
extern volatile uint32_t SysMs;
/* OLED 刷新标志。 */
extern volatile uint8_t  OledFlag;

/* 当前任务编号。 */
extern uint8_t  TaskId;
/* 目标航向角，单位度。 */
extern float    TargetYaw;

int main(void);
void TIMER_FOR_1MS_INST_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */

