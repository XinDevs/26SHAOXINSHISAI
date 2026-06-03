/**
 * @file    key.c
 * @brief   按键扫描驱动
 * @details 基于1ms定时调用的4键扫描与消抖。
 *          在定时器中断里周期调用 Key_Tick()，
 *          在主循环中调用 Key_GetNum() 读取按键事件。
 */
#include "ti_msp_dl_config.h"

static uint8_t Key_Num = 0U; /* 保存最终按键值 */

#ifndef GPIO_PIN_RESET
#define GPIO_PIN_RESET  (0U)
#endif

#ifndef GPIO_PIN_SET
#define GPIO_PIN_SET    (1U)
#endif


/**
 * @brief  读取当前物理按键状态
 * @retval 0: 无按键按下
 * @retval 1~4: 对应 KEY1~KEY4 按下
 */
static uint8_t Key_GetState(void)
{
    if (DL_GPIO_readPins(KEY_PORT, KEY_KEY_1_PIN) == GPIO_PIN_RESET) return 1U;
    if (DL_GPIO_readPins(KEY_PORT, KEY_KEY_2_PIN) == GPIO_PIN_RESET) return 2U;
    if (DL_GPIO_readPins(KEY_PORT, KEY_KEY_3_PIN) == GPIO_PIN_RESET) return 3U;
    if (DL_GPIO_readPins(KEY_PORT, KEY_KEY_4_PIN) == GPIO_PIN_RESET) return 4U;
    return 0U;
}

/**
 * @brief  获取一次消抖后的按键事件（读后清零）
 * @retval 0: 当前无新按键事件
 * @retval 1~4: 本次检测到的按键编号
 */
uint8_t Key_GetNum(void)
{
    uint8_t temp = Key_Num;
    Key_Num = 0U; /* 读出后清零，保证一次按下只响应一次 */
    return temp;
}

/**
 * @brief  按键消抖与按下沿检测
 * @note   需固定周期调用（通常1ms一次）。
 *         每20ms进行一次状态判定，捕获按下沿（0->非0）后写入 Key_Num。
 */
void Key_Tick(void)
{
    static uint8_t count = 0U;
    static uint8_t currState = 0U;
    static uint8_t prevState = 0U;

    count++;
    if (count >= 20U) {
        count = 0U;
        prevState = currState;
        currState = Key_GetState();

        /* 按下沿触发：从无按键(0)变化为某键值(1~4) */
        if ((currState != 0U) && (prevState == 0U)) {
            Key_Num = currState;
        }
    }
}
