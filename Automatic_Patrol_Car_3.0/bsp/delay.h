/**
 * @file    delay.h
 * @brief   微秒/毫秒延时函数 — 头文件
 * @details 声明delay_us()和delay_ms()接口。
 */
#ifndef ICODE_DELAY_H_
#define ICODE_DELAY_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 微秒级阻塞延时
 * @param us 延时时间（单位：us）
 */
// 接口说明：delay_us 函数声明
void delay_us(uint32_t us);

/**
 * @brief 毫秒级阻塞延时
 * @param ms 延时时间（单位：ms）
 */
// 接口说明：delay_ms 函数声明
void delay_ms(uint32_t ms);

/**
 * @brief 秒级阻塞延时
 * @param s 延时时间（单位：s）
 */
// 接口说明：delay_s 函数声明
void delay_s(uint32_t s);

#ifdef __cplusplus
}
#endif

#endif /* ICODE_DELAY_H_ */
