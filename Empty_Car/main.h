/**
 * @file    main.h
  *
 */
#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* */
#define PID_PENDING_COUNT_MAX  (100U)

/* */
#define BASE_LINE_SPEED        (0.3f)
/* */
#define LINE_SPEED_DIFF_SCALE  (1.0f)

/* */
#define BASE_STRAIGHT_SPEED    (0.4f)
/* */
#define MAX_YAW_SPEED_DIFF     (1.0f)

/* */
#define TURN_DONE_ERR_DEG      (5.0f)
/* */
#define TURN_ANGLE_DEG         (90.0f)

/* */
#define SPIN_TO_LINE_SPEED_MPS (0.3f)
/* */
#define TURN_LINE_DETECT_DELAY_MS (200U)
/* */
#define SPIN_LINE_DELAY_MS     (500U)

/* */
extern volatile float CurrentYaw;
/* */
extern volatile uint32_t SysMs;
extern volatile uint8_t  OledFlag;

/* */
extern uint8_t  TaskId;
/* */
extern float    TargetYaw;

int main(void);
void TIMER_FOR_1MS_INST_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */

