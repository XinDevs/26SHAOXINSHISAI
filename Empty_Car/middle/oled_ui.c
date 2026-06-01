#include "oled_ui.h"
#include "OLED.h"

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

    for (i = 0U; i < 8U; i++) {
        grayBitsLine[i] = (sensorBits[i] != 0U) ? '1' : '0';
    }
    grayBitsLine[8] = '\0';

    OLED_Clear();
    OLED_Printf(0, 0, OLED_6X8,
                "M%u Y%6.1f",
                (unsigned int)taskId,
                (float)currentYaw);
    OLED_Printf(0, 8, OLED_6X8,
                "TL%1.2f TR%1.2f",
                (float)pidRuntime->targetLeftSpeedMps,
                (float)pidRuntime->targetRightSpeedMps);
    OLED_Printf(0, 16, OLED_6X8,
                "AL%1.2f AR%1.2f",
                (float)leftSpeed,
                (float)rightSpeed);
    OLED_Printf(0, 24, OLED_6X8,
                "LP%.2f I%.2f D%.2f",
                (float)leftKp,
                (float)leftKi,
                (float)leftKd);
    OLED_Printf(0, 32, OLED_6X8,
                "RP%.2f I%.2f D%.2f",
                (float)rightKp,
                (float)rightKi,
                (float)rightKd);
    OLED_Printf(0, 40, OLED_6X8,
                "L%4d%% D%3d",
                motorStatus->left_duty_percent,
                (int)pidRuntime->leftDutyCmd);
    OLED_Printf(0, 48, OLED_6X8,
                "R%4d%% D%3d",
                motorStatus->right_duty_percent,
                (int)pidRuntime->rightDutyCmd);
    OLED_Printf(0, 56, OLED_6X8,
                "%s",
                grayBitsLine);
    OLED_Update();
}
