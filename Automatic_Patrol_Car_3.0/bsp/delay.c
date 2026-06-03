/**
 * @file    delay.c
 * @brief   微秒/毫秒延时函数
 * @details 基于CPU周期计数的us级忙等延时，以及基于SysTick的ms级延时。
 *          适用于ICM-42688等需要精确时序的SPI外设初始化。
 */

#include "ti_msp_dl_config.h"

/**
 * @brief  微秒级延时函数（精准CPU周期忙等）
 * @param  us 需要延时的微秒数
 * @note   利用 CPUCLK_FREQ 换算实际周期数，适用于短促精确的硬件时序处理
 */
void delay_us(uint32_t us)
{
    /* 每1us对应的CPU周期数 */
    const uint32_t cycles_per_us = CPUCLK_FREQ / 1000000U;

    while (us--) {
        delay_cycles(cycles_per_us);
    }
}

/**
 * @brief  毫秒级延时函数
 * @param  ms 需要延时的毫秒数
 * @note   内部循环调用延时微秒函数，会阻塞 CPU，应谨慎在主循环中使用
 */
void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000U);
    }
}

/**
 * @brief  秒级阻塞延时函数
 * @param  s 需要延时的秒数
 * @note   内部循环调用延时毫秒函数，适用于系统刚上电时大段的初始化等待
 */
void delay_s(uint32_t s)
{
    while (s--) {
        delay_ms(1000U);
    }
}
