/**
 * @file    pid.h
 * @brief   PID 控制器接口
 * @details 定义 PID 算法结构体、速度环/灰度环/航向环的运行时状态及控制接口。
 */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include "pid_gray.h"
#include "pid_yaw.h"

/* PID 控制对象枚举 */
typedef enum {
    MOTOR_LEFT = 0,   /* 左轮速度环 */
    MOTOR_RIGHT,      /* 右轮速度环 */
    GRAYSCALE,        /* 灰度巡线环 */
    YAW               /* 航向保持环 */
} PID_ITEM;

/* PID 算法结构体 */
typedef struct {
    float Kp;         /* 比例系数 */
    float Ki;         /* 积分系数 */
    float Kd;         /* 微分系数 */

    float Error0;     /* 当前误差 */
    float Error1;     /* 上次误差 */
    float ErrorInt;   /* 累积误差 */

    float Output;     /* 当前输出 */
    float OutputMax;  /* 输出上限 */
    float OutputMin;  /* 输出下限 */
    float IntegralMax;/* 积分上限 */
    float IntegralMin;/* 积分下限 */
} PID_TypeDef;

/* PID 运行时状态（供上报和调试） */
typedef struct {
    float targetLeftSpeedMps;    /* 左轮目标速度 m/s */
    float targetRightSpeedMps;   /* 右轮目标速度 m/s */
    float grayOuterSpeedDiff;    /* 灰度外环输出差速 */
    float yawOuterSpeedDiff;     /* 航向外环输出差速 */
    float targetYawForReport;    /* 上报用目标航向角 */
    int16_t leftDutyCmd;         /* 左轮占空比指令 */
    int16_t rightDutyCmd;        /* 右轮占空比指令 */
    uint8_t speedOverrideActive; /* 速度覆盖模式激活标志 */
} PID_RuntimeState_t;

/* PID 控制器实例 */
extern PID_TypeDef PID_Left_Speed;
extern PID_TypeDef PID_Right_Speed;
extern PID_TypeDef PID_Grayscale;
extern PID_TypeDef PID_Yaw;

/* 当前航向角 */
extern volatile float CurrentYaw;

/* PID 运行时状态 */
extern PID_RuntimeState_t g_pidRuntime;

/* 当前航向角（度） */
extern volatile float g_pidCurrentYawDeg;
/* 当前 Z 轴角速度（deg/s） */
extern volatile float g_pidCurrentGyroZDps;

/* 设置 PID 增益 */
void pid_set_gains(PID_TypeDef *pid, float kp, float ki, float kd);

/* PID 初始化 */
void PID_Init(void);

/* 速度环初始化 */
void PID_SPEED_INIT(float kp_r, float ki_r, float kd_r,
                    float kp_l, float ki_l, float kd_l);

/* 设置左轮目标速度 */
void PID_GoalSpeedLeft_Init(float goalSpeedLeftMps);

/* 设置右轮目标速度 */
void PID_GoalSpeedRight_Init(float goalSpeedRightMps);

/* 设置左轮目标速度及方向 */
void PID_GoalSpeedLeft_InitEx(float goalSpeedLeftMps, uint8_t forward);

/* 设置右轮目标速度及方向 */
void PID_GoalSpeedRight_InitEx(float goalSpeedRightMps, uint8_t forward);

/* 设置左右轮目标速度 */
void PID_GoalSpeedPair_Set(float goalLeftSpeedMps, float goalRightSpeedMps);

/* 启用速度覆盖模式 */
void PID_EnableSpeedOverride(float goalLeftSpeedMps, float goalRightSpeedMps);

/* 清除速度覆盖模式 */
void PID_ClearSpeedOverride(void);

/* 重置运行时状态 */
void PID_ResetRuntimeState(float yawForReportDeg);

/* 执行速度内环控制 */
void PID_ExecuteSpeedInnerLoop(void);

/* 获取目标速度 */
void PID_GetTargetSpeeds(float *leftTargetMps, float *rightTargetMps);

/* 获取外环差速 */
void PID_GetOuterDiffs(float *grayDiffMps, float *yawDiffMps);

/* 获取上报用目标航向角 */
float PID_GetTargetYawForReport(void);

/* 获取占空比指令 */
void PID_GetDutyCmd(int16_t *leftDuty, int16_t *rightDuty);

/* 获取运行时状态 */
void PID_GetRuntimeState(PID_RuntimeState_t *stateOut);

/* 获取 PID 参数 */
void PID_GetParameters(PID_ITEM item, float *kp, float *ki, float *kd);

/* PID 单步计算 */
float PID_Calculate_Step(PID_TypeDef *pid, float target, float actual);

/* 设置 PID 参数 */
void PID_SetParameters(PID_ITEM item, float kp, float ki, float kd);

/* 重置指定 PID 控制器 */
void PID_Reset(PID_ITEM item);

/* 获取 PID 输出 */
float PID_GetOutput(PID_ITEM item);

/* 根据目标速度计算占空比 */
void PID_Calculate_MotorDutyFromTargetSpeed(float targetLeftSpeedMps,
                                            float targetRightSpeedMps,
                                            int16_t *leftDutyOut,
                                            int16_t *rightDutyOut);

/* 重置所有 PID 控制器 */
void PID_ResetAll(void);

#endif /* __PID_H */
