#ifndef BUZZER_LED_H_
#define BUZZER_LED_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BUZZER_BEEP_MS         (100U)
#define BUZZER_ALERT_MS        (2000U)
#define BUZZER_BLINK_ON_MS     (100U)
#define BUZZER_BLINK_OFF_MS    (100U)

extern volatile uint8_t g_buzzerRequestFlag;

void BuzzerLed_StartRedAlert(void);
void BuzzerLed_StartGreenAlert(void);
void BuzzerLed_AllOff(void);
void BuzzerLed_Tick1ms(void);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_LED_H_ */
