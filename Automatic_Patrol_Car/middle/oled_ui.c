/**
 * @file    oled_ui.c
 * @brief   OLED 显示界面实现
 * @details 提供开机 IMU 状态显示和主循环待机界面渲染。
 */
#include "oled_ui.h"
#include "OLED.h"

/**
 * @brief  显示开机 IMU 状态
 * @param  imuId IMU 芯片 ID (0x47=正常)
 */
void OLEDUI_InitStatus(uint8_t imuId)
{
    OLED_Init();
    OLED_Clear();
    if (imuId == 0x47U) {
        OLED_ShowString(0, 0, "IMU Ready", OLED_8X16);
    } else {
        OLED_ShowString(0, 0, "IMU FAIL", OLED_8X16);
    }
    OLED_Update();
}

/**
 * @brief  显示待机/运行状态界面
 * @details 显示内容（共8行）：
 *          - 行0: 任务编号 + 航向角
 *          - 行1: 左右轮目标速度
 *          - 行2: 左右轮实际速度
 *          - 行3: 左轮 PID 参数
 *          - 行4: 右轮 PID 参数
 *          - 行5: 左轮占空比和指令
 *          - 行6: 右轮占空比和指令
 *          - 行7: 灰度传感器 8 路状态
 */
void OLEDUI_ShowIdle(uint8_t taskId,
                     float currentYaw,
                     const PID_RuntimeState_t *pidRuntime,
                     const DCMotor_Status_t *motorStatus,
                     float leftSpeed,
                     float rightSpeed,
                     float leftKp,
                     float leftKi,
                     float leftKd,
                     float rightKp,
                     float rightKi,
                     float rightKd,
                     const uint8_t sensorBits[8])
{
    char grayBitsLine[9];
    uint8_t i;

    /* 将传感器状态转为 '0'/'1' 字符串 */
    for (i = 0U; i < 8U; i++) {
        grayBitsLine[i] = (sensorBits[i] != 0U) ? '1' : '0';
    }
    grayBitsLine[8] = '\0';

    OLED_Clear();
    /* 行0: 任务编号 + 航向角 */
    OLED_Printf(0, 0, OLED_6X8,
                "M%u Y%6.1f",
                (unsigned int)taskId,
                (float)currentYaw);
    /* 行1: 左右轮目标速度 */
    OLED_Printf(0, 8, OLED_6X8,
                "TL%1.2f TR%1.2f",
                (float)pidRuntime->targetLeftSpeedMps,
                (float)pidRuntime->targetRightSpeedMps);
    /* 行2: 左右轮实际速度 */
    OLED_Printf(0, 16, OLED_6X8,
                "AL%1.2f AR%1.2f",
                (float)leftSpeed,
                (float)rightSpeed);
    /* 行3: 左轮 PID 参数 */
    OLED_Printf(0, 24, OLED_6X8,
                "LP%.2f I%.2f D%.2f",
                (float)leftKp,
                (float)leftKi,
                (float)leftKd);
    /* 行4: 右轮 PID 参数 */
    OLED_Printf(0, 32, OLED_6X8,
                "RP%.2f I%.2f D%.2f",
                (float)rightKp,
                (float)rightKi,
                (float)rightKd);
    /* 行5: 左轮占空比和指令 */
    OLED_Printf(0, 40, OLED_6X8,
                "L%4d%% D%3d",
                motorStatus->left_duty_percent,
                (int)pidRuntime->leftDutyCmd);
    /* 行6: 右轮占空比和指令 */
    OLED_Printf(0, 48, OLED_6X8,
                "R%4d%% D%3d",
                motorStatus->right_duty_percent,
                (int)pidRuntime->rightDutyCmd);
    /* 行7: 灰度传感器状态 */
    OLED_Printf(0, 56, OLED_6X8,
                "%s",
                grayBitsLine);
    OLED_Update();
}
