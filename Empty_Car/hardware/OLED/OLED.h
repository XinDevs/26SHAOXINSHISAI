/**
 * @file    OLED.h
 * @brief   0.96寸OLED显示屏驱动 — 头文件（4针脚I2C接口）
 * @details 提供OLED初始化、清屏、绘点、显示字符/字符串/数字/汉字等接口。
 */
#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>
#include "OLED_Data.h"

/*参数宏定义*********************/

/*FontSize参数取值*/
/*此参数值不仅用于判断，而且用于计算横向字符偏移，默认值为字体像素宽度*/
#define OLED_8X16				8
#define OLED_6X8				6

/*IsFilled参数数值*/
#define OLED_UNFILLED			0
#define OLED_FILLED				1

/*********************参数宏定义*/


/*函数声明*********************/

/*初始化函数*/
// 接口说明：OLED_Init 函数声明
void OLED_Init(void);

/*更新函数*/
// 接口说明：OLED_Update 函数声明
void OLED_Update(void);
// 接口说明：OLED_UpdateArea 函数声明
void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);

/*显存控制函数*/
// 接口说明：OLED_Clear 函数声明
void OLED_Clear(void);
// 接口说明：OLED_ClearArea 函数声明
void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);
// 接口说明：OLED_Reverse 函数声明
void OLED_Reverse(void);
// 接口说明：OLED_ReverseArea 函数声明
void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);

/*显示函数*/
// 接口说明：OLED_ShowChar 函数声明
void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize);
// 接口说明：OLED_ShowString 函数声明
void OLED_ShowString(int16_t X, int16_t Y, const char *String, uint8_t FontSize);
// 接口说明：OLED_ShowNum 函数声明
void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
// 接口说明：OLED_ShowSignedNum 函数声明
void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize);
// 接口说明：OLED_ShowHexNum 函数声明
void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
// 接口说明：OLED_ShowBinNum 函数声明
void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
// 接口说明：OLED_ShowFloatNum 函数声明
void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize);
// 接口说明：OLED_ShowImage 函数声明
void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image);
// 接口说明：OLED_Printf 函数声明
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, const char *format, ...);

/*绘图函数*/
// 接口说明：OLED_DrawPoint 函数声明
void OLED_DrawPoint(int16_t X, int16_t Y);
// 接口说明：OLED_GetPoint 函数声明
uint8_t OLED_GetPoint(int16_t X, int16_t Y);
// 接口说明：OLED_DrawLine 函数声明
void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1);
// 接口说明：OLED_DrawRectangle 函数声明
void OLED_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled);
// 接口说明：OLED_DrawTriangle 函数声明
void OLED_DrawTriangle(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint8_t IsFilled);
// 接口说明：OLED_DrawCircle 函数声明
void OLED_DrawCircle(int16_t X, int16_t Y, uint8_t Radius, uint8_t IsFilled);
// 接口说明：OLED_DrawEllipse 函数声明
void OLED_DrawEllipse(int16_t X, int16_t Y, uint8_t A, uint8_t B, uint8_t IsFilled);
// 接口说明：OLED_DrawArc 函数声明
void OLED_DrawArc(int16_t X, int16_t Y, uint8_t Radius, int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled);

/*********************函数声明*/

#endif


/*****************江协科技|版权所有****************/
/*****************jiangxiekeji.com*****************/
