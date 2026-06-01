/**
 * @file    pid.h
 * @brief   多环PID控制器 — 头文件
 * @details 声明PID结构体、四环（左轮/右轮/灰度/Yaw）的初始化、计算、参数设置与复位接口。
 */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include "pid_gray.h"
#include "pid_yaw.h"

// 选择PID控制对象
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    GRAYSCALE
    ,YAW
} PID_ITEM;

// PID 参数结构体
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float Error0;       // 当前误差
    float Error1;       // 上次误差
    float ErrorInt;     // 误差积分
    
    float Output;       // PID 输出值
    float OutputMax;    // 输出限幅最大值
    float OutputMin;    // 输出限幅最小值
    float IntegralMax;  // 积分限幅最大值
    float IntegralMin;  // 积分限幅最小值
} PID_TypeDef;

typedef struct {
    float targetLeftSpeedMps;      // 左轮目标速度，单位：米/秒
    float targetRightSpeedMps;     // 右轮目标速度，单位：米/秒
    float grayOuterSpeedDiff;      // 灰度外环输出的速度差值，单位：米/秒
    float yawOuterSpeedDiff;       // 航向(Yaw)外环输出的速度差值，单位：米/秒
    float targetYawForReport;      // 用于上报的目标航向角，单位：度
    int16_t leftDutyCmd;           // 左轮PWM占空比指令，范围：-1000~1000
    int16_t rightDutyCmd;          // 右轮PWM占空比指令，范围：-1000~1000
    uint8_t speedOverrideActive;   // 速度覆盖模式激活标志，1=激活(忽略外环)，0=正常级联模式
} PID_RuntimeState_t;

// 外部声明，方便其他文件调用
extern PID_TypeDef PID_Left_Speed;    
extern PID_TypeDef PID_Right_Speed;   
extern PID_TypeDef PID_Grayscale;
extern PID_TypeDef PID_Yaw;
// 主循环计算得到的 yaw 值（单位与 Mahony getYaw 一致），由 main.c 更新
extern volatile float g_currentYaw;
// PID 运行态快照，由 pid.c 定义，供 pid_gray/pid_yaw 读写
extern PID_RuntimeState_t g_pidRuntime;
// IMU 解算的当前航向角(deg)，由 pid_yaw.c 的 PID_UpdateYawFeedback 写入
extern volatile float g_pidCurrentYawDeg;
// PID 增益设置内部接口，供 pid_gray/pid_yaw 初始化时调用
void pid_set_gains(PID_TypeDef *pid, float kp, float ki, float kd);

// 函数声明
/**
 * @brief  PID 系统初始化
 */
void PID_Init(void);
/**
 * @brief  速度环参数初始化(右轮 + 左轮)
 */
void PID_SPEED_INIT(float kp_r, float ki_r, float kd_r,
                    float kp_l, float ki_l, float kd_l);
/**
 * @brief  初始化左轮目标速度
 */
void PID_GoalSpeedLeft_Init(float goalSpeedLeftMps);
/**
 * @brief  初始化右轮目标速度
 */
void PID_GoalSpeedRight_Init(float goalSpeedRightMps);
/**
 * @brief  初始化左轮目标速度(扩展: 支持正反转)
 * @param  goalSpeedLeftMps 目标速度(m/s)
 * @param  forward 1: 正向PID控制, 0: 直接反转控制
 */
void PID_GoalSpeedLeft_InitEx(float goalSpeedLeftMps, uint8_t forward);
/**
 * @brief  初始化右轮目标速度(扩展: 支持正反转)
 * @param  goalSpeedRightMps 目标速度(m/s)
 * @param  forward 1: 正向PID控制, 0: 直接反转控制
 */
void PID_GoalSpeedRight_InitEx(float goalSpeedRightMps, uint8_t forward);
/**
 * @brief  同时设置左右轮目标速度
 */
void PID_GoalSpeedPair_Set(float goalLeftSpeedMps, float goalRightSpeedMps);
/**
 * @brief  开启速度覆盖模式(忽略外环, 直接走速度内环)
 */
void PID_EnableSpeedOverride(float goalLeftSpeedMps, float goalRightSpeedMps);
/**
 * @brief  关闭速度覆盖模式
 */
void PID_ClearSpeedOverride(void);
/**
 * @brief  重置 PID 运行态(目标速度/差速/占空比)
 */
void PID_ResetRuntimeState(float yawForReportDeg);
/**
 * @brief  仅执行速度内环(使用当前左右目标速度)
 */
void PID_ExecuteSpeedInnerLoop(void);
/**
 * @brief  读取当前目标速度
 */
void PID_GetTargetSpeeds(float *leftTargetMps, float *rightTargetMps);
/**
 * @brief  读取当前外环差速输出
 */
void PID_GetOuterDiffs(float *grayDiffMps, float *yawDiffMps);
/**
 * @brief  读取上报用目标航向
 */
float PID_GetTargetYawForReport(void);
/**
 * @brief  读取当前占空比指令
 */
void PID_GetDutyCmd(int16_t *leftDuty, int16_t *rightDuty);
/**
 * @brief  读取 PID 运行态快照
 */
void PID_GetRuntimeState(PID_RuntimeState_t *stateOut);
/**
 * @brief  读取指定对象 PID 参数
 */
void PID_GetParameters(PID_ITEM item, float *kp, float *ki, float *kd);
/**
 * @brief  PID 单步计算
 * @param  pid PID 结构体指针
 * @param  target 目标值
 * @param  actual 实际值
 * @retval PID 输出
 */
float PID_Calculate_Step(PID_TypeDef *pid, float target, float actual);
/**
 * @brief  动态设置指定对象 PID 参数
 */
void PID_SetParameters(PID_ITEM item, float kp, float ki, float kd);
/**
 * @brief  重置指定对象 PID 历史状态
 */
void PID_Reset(PID_ITEM item);
/**
 * @brief  获取指定对象当前 PID 输出值
 */
float PID_GetOutput(PID_ITEM item);
/**
 * @brief  根据目标速度计算左右轮占空比
 */
void PID_Calculate_MotorDutyFromTargetSpeed(float targetLeftSpeedMps,
                                            float targetRightSpeedMps,
                                            int16_t *leftDutyOut,
                                            int16_t *rightDutyOut);
/**
 * @brief  重置全部 PID 历史状态
 */
void PID_ResetAll(void);

#endif // __PID_H