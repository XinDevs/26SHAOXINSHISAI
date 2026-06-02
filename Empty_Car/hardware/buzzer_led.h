/**
 * @file    buzzer_led.h
 * @brief   蜂鸣器和 LED 控制接口
 * @details 提供蜂鸣器鸣响、LED 闪烁的非阻塞控制接口，由 1ms 定时中断驱动。
 */
#ifndef BUZZER_LED_H_
#define BUZZER_LED_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BUZZER_BEEP_MS         (100U)   /* 短鸣持续时间(ms) */
#define BUZZER_ALERT_MS        (2000U)  /* 警报持续时间(ms) */
#define BUZZER_BLINK_ON_MS     (100U)   /* 闪烁点亮时间(ms) */
#define BUZZER_BLINK_OFF_MS    (100U)   /* 闪烁熄灭时间(ms) */

/** @brief 蜂鸣器请求标志，置1触发一次短鸣 */
extern volatile uint8_t g_buzzerRequestFlag;

/** @brief 启动红色警报（红灯常亮 + 蜂鸣器常响） */
void BuzzerLed_StartRedAlert(void);

/** @brief 启动绿色警报（绿灯常亮 + 蜂鸣器闪烁） */
void BuzzerLed_StartGreenAlert(void);

/** @brief 关闭所有蜂鸣器和 LED */
void BuzzerLed_AllOff(void);

/** @brief 1ms 周期调用，处理蜂鸣器和 LED 的定时逻辑 */
void BuzzerLed_Tick1ms(void);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_LED_H_ */
