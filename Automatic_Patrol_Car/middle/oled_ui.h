/**
 * @file    oled_ui.h
 * @brief   OLED 显示界面接口
 * @details 提供开机状态显示和主循环待机界面的渲染函数。
 */
#ifndef OLED_UI_H_
#define OLED_UI_H_

#include "dc_motor.h"
#include "pid.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  显示开机 IMU 状态
 * @param  imuId IMU 芯片 ID (0x47=正常)
 */
void OLEDUI_InitStatus(uint8_t imuId);

/**
 * @brief  显示待机/运行状态界面
 * @param  taskId      当前任务编号
 * @param  currentYaw  当前航向角
 * @param  pidRuntime  PID 运行时状态
 * @param  motorStatus 电机状态
 * @param  leftSpeed   左轮实际速度
 * @param  rightSpeed  右轮实际速度
 * @param  leftKp/Ki/Kd  左轮 PID 参数
 * @param  rightKp/Ki/Kd 右轮 PID 参数
 * @param  sensorBits  灰度传感器 8 路状态
 */
void OLEDUI_ShowIdle(uint8_t taskId,
                     float currentYaw,
                     const PID_RuntimeState_t *pidRuntime,
                     const DCMotor_Status_t *motorStatus,
                     float leftSpeed,
                     float rightSpeed,
                     float leftKp,
                     float leftKi,
                     float leftKd,
                     float rightKp,
                     float rightKi,
                     float rightKd,
                     const uint8_t sensorBits[8]);

#ifdef __cplusplus
}
#endif

#endif /* OLED_UI_H_ */
