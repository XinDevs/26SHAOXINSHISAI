#ifndef OLED_UI_H_
#define OLED_UI_H_

#include "dc_motor.h"
#include "pid.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void OLEDUI_InitStatus(uint8_t imuId);

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

#endif
