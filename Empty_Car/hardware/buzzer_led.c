#include "buzzer_led.h"

#include "ti_msp_dl_config.h"

typedef enum {
    BUZZER_MODE_OFF = 0,
    BUZZER_MODE_SOLID,
    BUZZER_MODE_BLINK
} BuzzerMode_t;

volatile uint8_t g_buzzerRequestFlag = 0U;

static volatile BuzzerMode_t s_buzzerMode          = BUZZER_MODE_OFF;
static volatile uint16_t     s_buzzerTotalRemainMs = 0U;
static volatile uint16_t     s_buzzerPhaseRemainMs = 0U;
static volatile uint8_t      s_buzzerOutputOn      = 0U;
static volatile uint16_t     s_ledRemainMs         = 0U;

static void Buzzer_SetOutput(uint8_t on)
{
    if (on != 0U) {
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN_0_PIN);
        s_buzzerOutputOn = 1U;
    } else {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN_0_PIN);
        s_buzzerOutputOn = 0U;
    }
}

static void Led_SetRed(uint8_t on)
{
    if (on != 0U) {
        DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_2_PIN);
    } else {
        DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN_2_PIN);
    }
}

static void Led_SetGreen(uint8_t on)
{
    if (on != 0U) {
        DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_3_PIN);
    } else {
        DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_3_PIN);
    }
}

void BuzzerLed_StartRedAlert(void)
{
    Led_SetGreen(0U);
    Led_SetRed(1U);
    s_ledRemainMs = BUZZER_ALERT_MS;
    s_buzzerMode = BUZZER_MODE_SOLID;
    s_buzzerTotalRemainMs = BUZZER_ALERT_MS;
    s_buzzerPhaseRemainMs = 0U;
    Buzzer_SetOutput(1U);
}

void BuzzerLed_StartGreenAlert(void)
{
    Led_SetRed(0U);
    Led_SetGreen(1U);
    s_ledRemainMs = BUZZER_ALERT_MS;
    s_buzzerMode = BUZZER_MODE_BLINK;
    s_buzzerTotalRemainMs = BUZZER_ALERT_MS;
    s_buzzerPhaseRemainMs = BUZZER_BLINK_ON_MS;
    Buzzer_SetOutput(1U);
}

void BuzzerLed_AllOff(void)
{
    s_buzzerMode = BUZZER_MODE_OFF;
    s_buzzerTotalRemainMs = 0U;
    s_buzzerPhaseRemainMs = 0U;
    s_ledRemainMs = 0U;
    Buzzer_SetOutput(0U);
    Led_SetRed(0U);
    Led_SetGreen(0U);
}

void BuzzerLed_Tick1ms(void)
{
    if (s_ledRemainMs > 0U) {
        s_ledRemainMs--;
        if (s_ledRemainMs == 0U) {
            Led_SetRed(0U);
            Led_SetGreen(0U);
        }
    }

    if (g_buzzerRequestFlag != 0U) {
        g_buzzerRequestFlag = 0U;
        s_buzzerMode = BUZZER_MODE_SOLID;
        s_buzzerTotalRemainMs = BUZZER_BEEP_MS;
        s_buzzerPhaseRemainMs = 0U;
        Buzzer_SetOutput(1U);
    } else if (s_buzzerTotalRemainMs > 0U) {
        s_buzzerTotalRemainMs--;

        if (s_buzzerMode == BUZZER_MODE_BLINK) {
            if (s_buzzerPhaseRemainMs > 0U) {
                s_buzzerPhaseRemainMs--;
            }
            if (s_buzzerPhaseRemainMs == 0U) {
                if (s_buzzerOutputOn != 0U) {
                    Buzzer_SetOutput(0U);
                    s_buzzerPhaseRemainMs = BUZZER_BLINK_OFF_MS;
                } else {
                    Buzzer_SetOutput(1U);
                    s_buzzerPhaseRemainMs = BUZZER_BLINK_ON_MS;
                }
            }
        }

        if (s_buzzerTotalRemainMs == 0U) {
            s_buzzerMode = BUZZER_MODE_OFF;
            s_buzzerPhaseRemainMs = 0U;
            Buzzer_SetOutput(0U);
        }
    }
}
