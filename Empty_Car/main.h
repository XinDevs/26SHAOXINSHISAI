/**
 * @file    main.h
 * @brief   Public definitions for main.c.
 */
#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PID_PENDING_COUNT_MAX  (100U)

#define BASE_LINE_SPEED        (0.2f)
#define MAX_LINE_SPEED_DIFF    (0.80f)
#define BASE_SPEED_TEST        (0.5f)
#define PWM_TEST_DUTY          (30)

#define BASE_STRAIGHT_SPEED    (0.4f)
#define MAX_YAW_SPEED_DIFF     (1.0f)

#define TURN_DONE_ERR_DEG      (5.0f)
#define TURN_ANGLE_DEG         (90.0f)

#define SPIN_TO_LINE_SPEED_MPS (0.25f)
#define SPIN_LINE_DELAY_MS     (500U)

extern volatile float g_currentYaw;

extern uint8_t  g_taskId;
extern float    target_straight_yaw;

int main(void);
uint32_t Main_GetSysTickMs(void);
void TIMER_FOR_1MS_INST_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */
