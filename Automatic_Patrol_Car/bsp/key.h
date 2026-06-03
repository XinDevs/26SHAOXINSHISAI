/**
 * @file    key.h
 * @brief   按键扫描驱动 — 头文件
 * @details 声明按键扫描定时Tick与键值获取接口，用于任务模式切换。
 */
#ifndef ICODE_KEY_H_
#define ICODE_KEY_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 周期调用的按键扫描任务（当前工程在1ms中断中调用，内部20ms消抖）
 */
// 接口说明：Key_Tick 函数声明
void Key_Tick(void);

/**
 * @brief 获取一次按键值（1~4），无新按键返回0
 */
// 接口说明：Key_GetNum 函数声明
uint8_t Key_GetNum(void);

#ifdef __cplusplus
}
#endif

#endif /* ICODE_KEY_H_ */
