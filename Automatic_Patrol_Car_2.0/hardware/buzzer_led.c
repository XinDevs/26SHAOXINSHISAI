/**
 * @file    buzzer_led.c
 * @brief   蜂鸣器和 LED 控制实现
 * @details 非阻塞驱动，支持常响、闪烁模式，由 1ms 定时中断调用 Tick1ms() 驱动。
 */
#include "buzzer_led.h"

#include "ti_msp_dl_config.h"

/* 蜂鸣器工作模式 */
typedef enum {
    BUZZER_MODE_OFF = 0,  /* 关闭 */
    BUZZER_MODE_SOLID,    /* 常响 */
    BUZZER_MODE_BLINK     /* 闪烁 */
} BuzzerMode_t;

/** @brief 蜂鸣器短鸣请求标志 */
volatile uint8_t g_buzzerRequestFlag = 0U;

static volatile BuzzerMode_t s_buzzerMode          = BUZZER_MODE_OFF;  /* 当前模式 */
static volatile uint16_t     s_buzzerTotalRemainMs = 0U;  /* 总剩余时间 */
static volatile uint16_t     s_buzzerPhaseRemainMs = 0U;  /* 当前相位剩余时间 */
static volatile uint8_t      s_buzzerOutputOn      = 0U;  /* 当前输出状态 */
static volatile uint16_t     s_ledRemainMs         = 0U;  /* LED 剩余时间 */

/**
 * @brief  设置蜂鸣器输出
 * @param  on 1=响, 0=静音
 */
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

/**
 * @brief  设置红色 LED
 * @param  on 1=亮, 0=灭
 */
static void Led_SetRed(uint8_t on)
{
    if (on != 0U) {
        DL_GPIO_setPins(LED_RED_PORT, LED_RED_PIN_2_PIN);
    } else {
        DL_GPIO_clearPins(LED_RED_PORT, LED_RED_PIN_2_PIN);
    }
}

/**
 * @brief  设置绿色 LED
 * @param  on 1=亮, 0=灭
 */
static void Led_SetGreen(uint8_t on)
{
    if (on != 0U) {
        DL_GPIO_setPins(LED_GREEN_PORT, LED_GREEN_PIN_3_PIN);
    } else {
        DL_GPIO_clearPins(LED_GREEN_PORT, LED_GREEN_PIN_3_PIN);
    }
}

/**
 * @brief  启动红色警报
 * @details 红灯常亮 + 蜂鸣器常响，持续 2 秒
 */
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

/**
 * @brief  启动绿色警报
 * @details 绿灯常亮 + 蜂鸣器闪烁，持续 2 秒
 */
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

/**
 * @brief  关闭所有蜂鸣器和 LED
 */
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

/**
 * @brief  1ms 周期调用，处理蜂鸣器和 LED 定时逻辑
 * @details 处理逻辑：
 *          1. LED 倒计时，到期自动关闭
 *          2. 检测短鸣请求，触发 100ms 常响
 *          3. 闪烁模式下交替开关蜂鸣器
 *          4. 总时间到期后关闭蜂鸣器
 */
void BuzzerLed_Tick1ms(void)
{
    /* LED 倒计时 */
    if (s_ledRemainMs > 0U) {
        s_ledRemainMs--;
        if (s_ledRemainMs == 0U) {
            Led_SetRed(0U);
            Led_SetGreen(0U);
        }
    }

    /* 蜂鸣器控制 */
    if (g_buzzerRequestFlag != 0U) {
        /* 检测到短鸣请求 */
        g_buzzerRequestFlag = 0U;
        s_buzzerMode = BUZZER_MODE_SOLID;
        s_buzzerTotalRemainMs = BUZZER_BEEP_MS;
        s_buzzerPhaseRemainMs = 0U;
        Buzzer_SetOutput(1U);
    } else if (s_buzzerTotalRemainMs > 0U) {
        s_buzzerTotalRemainMs--;

        /* 闪烁模式处理 */
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

        /* 总时间到期，关闭蜂鸣器 */
        if (s_buzzerTotalRemainMs == 0U) {
            s_buzzerMode = BUZZER_MODE_OFF;
            s_buzzerPhaseRemainMs = 0U;
            Buzzer_SetOutput(0U);
        }
    }
}
