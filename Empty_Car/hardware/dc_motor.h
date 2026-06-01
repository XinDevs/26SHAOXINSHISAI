/**
 * @file    dc_motor.h
 * @brief   双路直流电机PWM驱动 — 头文件
 * @details 声明电机初始化、使能、占空比设置、停止及状态查询接口。
 */
#ifndef ICODE_DC_MOTOR_H_
#define ICODE_DC_MOTOR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DCMOTOR_DIR_STOP = 0,
    DCMOTOR_DIR_FORWARD,
    DCMOTOR_DIR_BACKWARD
} DCMotor_Direction_t;

typedef struct {
    int16_t left_duty_percent;
    int16_t right_duty_percent;
    DCMotor_Direction_t left_dir;
    DCMotor_Direction_t right_dir;
    uint8_t enabled;
} DCMotor_Status_t;

/**
 * @brief  直流电机驱动初始化
 */
void DCMotor_Init(void);
/**
 * @brief  使能或关闭电机输出
 * @param  enable 0: 关闭, 非0: 使能
 */
void DCMotor_Enable(uint8_t enable);
/**
 * @brief  设置左右电机占空比
 * @param  left_duty_percent 左轮占空比(-100~100)
 * @param  right_duty_percent 右轮占空比(-100~100)
 */
void DCMotor_SetDuty(int16_t left_duty_percent, int16_t right_duty_percent);
/**
 * @brief  停止双电机输出
 */
void DCMotor_Stop(void);
/**
 * @brief  获取电机状态快照
 * @param  status 输出状态结构体指针
 */
void DCMotor_GetStatus(DCMotor_Status_t *status);
/**
 * @brief  将方向枚举转为字符串
 * @param  dir 方向枚举
 * @retval 方向字符串(FWD/BWD/STP)
 */
const char *DCMotor_DirectionString(DCMotor_Direction_t dir);

#ifdef __cplusplus
}
#endif

#endif /* ICODE_DC_MOTOR_H_ */
