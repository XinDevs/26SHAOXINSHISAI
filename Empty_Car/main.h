/**
 * @file    main.h
 * @brief   主程序公共定义
 * @details 存放 main.c 使用的通用常量、外部变量声明。
 */
#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PID_PENDING_COUNT_MAX  (100U)               /* pending计数上限, 防止ISR无限累加 */
#define BUZZER_BEEP_MS         (100U)               /* 蜂鸣器单次鸣叫时长(ms) */

/* 通用巡线参数 */
#define BASE_LINE_SPEED        (0.2f)               /* 巡线基础速度(m/s) */
#define MAX_LINE_SPEED_DIFF    (0.80f)              /* 灰度外环差速上限(m/s) */

/* 通用直线行驶参数 */
#define BASE_STRAIGHT_SPEED    (0.4f)               /* 直线基础速度(m/s) */
#define MAX_YAW_SPEED_DIFF     (1.0f)               /* Yaw外环差速上限(m/s) */

/* 通用转向参数 */
#define TURN_DONE_ERR_DEG      (5.0f)               /* 转向完成判定死区(deg) */
#define TURN_ANGLE_DEG         (90.0f)              /* 默认直角转弯角度(deg) */

/* 原地旋转找线参数 */
#define SPIN_TO_LINE_SPEED_MPS (0.25f)              /* 原地旋转找线速度(m/s) */
#define SPIN_LINE_DELAY_MS     (500U)               /* 旋转后延迟检测灰度(ms), 防止误判当前线 */

/* Emm_V5 步进电机参数 */
#define EMM_MOTOR_ADDR         (0x01U)              /* 步进电机驱动器地址 */
#define EMM_POS_VEL_RPM        (100U)               /* 位置模式目标速度(RPM) */
#define EMM_POS_ACC            (20U)                /* 位置模式加速度(RPM/s) */
#define EMM_HALF_TURN_PULSES   (1600U)              /* 半圈脉冲数 */
#define EMM_ORIGIN_MODE        (0U)                 /* 回零模式 */

/* IMU航向角反馈量, 由 main.c 中断更新 */
extern volatile float g_currentYaw;

/* ===== 共享状态变量(供菜单等模块访问) ===== */
extern uint8_t  g_taskId;                /* 当前任务编号: 0=待机, 1=巡线, 2=航向直行, 3=电机测试 */
extern float    target_straight_yaw;     /* 任务2航向直行目标航向角(deg) */
extern volatile uint8_t g_buzzerRequestFlag; /* 蜂鸣器请求标志: 1=请求响一次 */
extern uint8_t  g_gimbalZeroSetDone;     /* 云台零点设置完成标志: 1=已设置 */

int main(void);
void TIMER_FOR_1MS_INST_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
