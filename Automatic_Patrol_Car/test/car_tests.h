#ifndef CAR_TESTS_H_
#define CAR_TESTS_H_

#include <stdint.h>

typedef struct {
    uint16_t flashId;
    uint8_t flashResult;
    uint8_t grayLeftRaw;
    uint8_t grayRightRaw;
    uint8_t grayStatus;
} CarTestState_t;

/* 速度环测试：左右轮以目标速度运行 */
void CarTest_SpeedLoop(void);

/* PWM测试：右电机固定占空比输出 */
void CarTest_PwmTest(void);

/* 速度环测试页面渲染：显示目标速度、实际速度、行驶距离 */
void CarTest_RenderSpeedLoop(void);
/* PWM测试页面渲染：显示占空比、实际速度 */
void CarTest_RenderPwm(void);
/* 执行Flash读写自检，结果存入state */
void CarTest_FlashRun(CarTestState_t *state);
/* Flash测试页面渲染：显示PASS/FAIL、芯片ID、错误码 */
void CarTest_RenderFlash(const CarTestState_t *state);
/* 灰度测试页面渲染：显示16路传感器数字状态 */
void CarTest_RenderGray(CarTestState_t *state);
/* 灰度模拟值测试页面渲染：显示16路传感器模拟量 */
void CarTest_RenderGrayAnalog(void);
/* 进入摄像头测试：发送开始识别请求 */
void CarTest_CameraEnter(void);
/* 摄像头测试中：发送一次识别请求 */
void CarTest_CameraLook(void);
/* 退出摄像头测试：发送停止识别请求 */
void CarTest_CameraExit(void);
/* 摄像头测试页面渲染：显示识别结果、收发数据 */
void CarTest_RenderCamera(void);
/* OLED中文字体测试页面渲染：显示12×12和16×16中文 */
void CarTest_RenderOledChinese(void);
/* 蜂鸣器LED测试页面渲染：K1红灯、K2绿灯、K3关闭 */
void CarTest_RenderBuzzerLed(void);

#endif /* CAR_TESTS_H_ */
