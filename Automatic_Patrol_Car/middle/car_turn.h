/**
 * @file    car_turn.h
 * @brief   小车转弯辅助接口
 */
#ifndef CAR_TURN_H_
#define CAR_TURN_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 重置转弯状态 */
void Turn_Reset(void);

/* 查询转弯是否进行中 */
uint8_t Turn_IsRunning(void);

/* 启动转弯: turnLeft=1左转, speedMps目标速度, nowMs当前时间 */
void Turn_Start(uint8_t turnLeft, float speedMps, uint32_t nowMs);

/* 执行转弯速度环(20ms周期调用) */
void Turn_Run(void);

/* 延时后检测中间传感器是否回到线上 */
uint8_t Turn_IsDone(uint32_t nowMs, uint32_t delayMs);

#ifdef __cplusplus
}
#endif

#endif /* CAR_TURN_H_ */
