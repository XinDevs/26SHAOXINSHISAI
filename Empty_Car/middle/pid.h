/**
 * @file    pid.h
  *
  *
 */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include "pid_gray.h"
#include "pid_yaw.h"

//
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    GRAYSCALE
    ,YAW
} PID_ITEM;

//
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float Error0;
    float Error1;
    float ErrorInt;
    
    float Output;
    float OutputMax;
    float OutputMin;
    float IntegralMax;
    float IntegralMin;
} PID_TypeDef;

typedef struct {
    float targetLeftSpeedMps;
    float targetRightSpeedMps;
    float grayOuterSpeedDiff;
    float yawOuterSpeedDiff;
    float targetYawForReport;
    int16_t leftDutyCmd;
    int16_t rightDutyCmd;
    uint8_t speedOverrideActive;
} PID_RuntimeState_t;

//
extern PID_TypeDef PID_Left_Speed;    
extern PID_TypeDef PID_Right_Speed;   
extern PID_TypeDef PID_Grayscale;
extern PID_TypeDef PID_Yaw;
//
extern volatile float CurrentYaw;
//
extern PID_RuntimeState_t g_pidRuntime;
//
extern volatile float g_pidCurrentYawDeg;
//
void pid_set_gains(PID_TypeDef *pid, float kp, float ki, float kd);

//
/**
  *
 */
void PID_Init(void);
/**
  *
 */
void PID_SPEED_INIT(float kp_r, float ki_r, float kd_r,
                    float kp_l, float ki_l, float kd_l);
/**
  *
 */
void PID_GoalSpeedLeft_Init(float goalSpeedLeftMps);
/**
  *
 */
void PID_GoalSpeedRight_Init(float goalSpeedRightMps);
/**
  *
  *
  *
 */
void PID_GoalSpeedLeft_InitEx(float goalSpeedLeftMps, uint8_t forward);
/**
  *
  *
  *
 */
void PID_GoalSpeedRight_InitEx(float goalSpeedRightMps, uint8_t forward);
/**
  *
 */
void PID_GoalSpeedPair_Set(float goalLeftSpeedMps, float goalRightSpeedMps);
/**
  *
 */
void PID_EnableSpeedOverride(float goalLeftSpeedMps, float goalRightSpeedMps);
/**
  *
 */
void PID_ClearSpeedOverride(void);
/**
  *
 */
void PID_ResetRuntimeState(float yawForReportDeg);
/**
  *
 */
void PID_ExecuteSpeedInnerLoop(void);
/**
  *
 */
void PID_GetTargetSpeeds(float *leftTargetMps, float *rightTargetMps);
/**
  *
 */
void PID_GetOuterDiffs(float *grayDiffMps, float *yawDiffMps);
/**
  *
 */
float PID_GetTargetYawForReport(void);
/**
  *
 */
void PID_GetDutyCmd(int16_t *leftDuty, int16_t *rightDuty);
/**
  *
 */
void PID_GetRuntimeState(PID_RuntimeState_t *stateOut);
/**
  *
 */
void PID_GetParameters(PID_ITEM item, float *kp, float *ki, float *kd);
/**
  *
  *
  *
  *
  *
 */
float PID_Calculate_Step(PID_TypeDef *pid, float target, float actual);
/**
  *
 */
void PID_SetParameters(PID_ITEM item, float kp, float ki, float kd);
/**
  *
 */
void PID_Reset(PID_ITEM item);
/**
  *
 */
float PID_GetOutput(PID_ITEM item);
/**
  *
 */
void PID_Calculate_MotorDutyFromTargetSpeed(float targetLeftSpeedMps,
                                            float targetRightSpeedMps,
                                            int16_t *leftDutyOut,
                                            int16_t *rightDutyOut);
/**
  *
 */
void PID_ResetAll(void);

#endif // __PID_H
