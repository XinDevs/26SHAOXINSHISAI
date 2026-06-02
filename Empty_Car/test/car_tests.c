#include "car_tests.h"

#include "Flash.h"
#include "OLED.h"
#include "grayscale_sensor.h"
#include "serial_maixcam.h"

void CarTest_FlashRun(CarTestState_t *state)
{
    OLED_Clear();
    OLED_Printf(0, 0, OLED_8X16, "Flash Test");
    OLED_Printf(0, 16, OLED_6X8, "Testing...");
    OLED_Update();

    state->flashResult = Flash_Test(&state->flashId);
}

void CarTest_RenderFlash(const CarTestState_t *state)
{
    OLED_Clear();
    OLED_Printf(0, 0, OLED_8X16,
                (state->flashResult == FLASH_TEST_OK) ? "Flash PASS" : "Flash FAIL");
    OLED_Printf(0, 16, OLED_6X8, "ID: 0x%04X", (unsigned int)state->flashId);
    OLED_Printf(0, 24, OLED_6X8, "Code: %u", (unsigned int)state->flashResult);
    OLED_Printf(0, 56, OLED_6X8, "K4:Back");
    OLED_Update();
}

void CarTest_RenderGray(CarTestState_t *state)
{
    uint8_t i;
    char bits[GW_GRAY_CHANNEL_COUNT + 1U];
    char bitsLine[GW_GRAY_CHANNEL_COUNT + 2U];
    uint8_t leftOk;
    uint8_t rightOk;

    leftOk = IIC_Get_Digtal_Ex(GW_GRAY_ADDR_SENSOR_LEFT, &state->grayLeftRaw);
    rightOk = IIC_Get_Digtal_Ex(GW_GRAY_ADDR_SENSOR_RIGHT, &state->grayRightRaw);
    state->grayStatus = (leftOk == GW_GRAY_OK && rightOk == GW_GRAY_OK) ? GW_GRAY_OK : GW_GRAY_I2C_ERROR;

    if (state->grayStatus == GW_GRAY_OK) {
        grayscale_dual_byte_to_sensor_array(state->grayLeftRaw, state->grayRightRaw);
    }

    for (i = 0U; i < GW_GRAY_CHANNEL_COUNT; i++) {
        bits[i] = (sensor[i] != 0U) ? '1' : '0';
    }
    bits[GW_GRAY_CHANNEL_COUNT] = '\0';

    for (i = 0U; i < GW_GRAY_MODULE_CHANNEL_COUNT; i++) {
        bitsLine[i] = bits[i];
        bitsLine[i + GW_GRAY_MODULE_CHANNEL_COUNT + 1U] =
            bits[i + GW_GRAY_MODULE_CHANNEL_COUNT];
    }
    bitsLine[GW_GRAY_MODULE_CHANNEL_COUNT] = ' ';
    bitsLine[GW_GRAY_CHANNEL_COUNT + 1U] = '\0';

    OLED_Clear();
    OLED_Printf(0, 0, OLED_8X16, "Gray Test");
    OLED_Printf(0, 16, OLED_6X8, "L:%02X R:%02X S:%u",
                (unsigned int)state->grayLeftRaw,
                (unsigned int)state->grayRightRaw,
                (unsigned int)state->grayStatus);
    OLED_Printf(0, 24, OLED_6X8, "1-8      9-16");
    OLED_Printf(0, 32, OLED_6X8, "%s", bitsLine);
    OLED_Printf(0, 48, OLED_6X8, "1=Black 0=White");
    OLED_Printf(0, 56, OLED_6X8, "K4:Back");
    OLED_Update();
}

void CarTest_CameraEnter(void)
{
    (void)SerialMaixCam_SendCommand("start communication");
}

void CarTest_CameraLook(void)
{
    (void)SerialMaixCam_SendCommand("need look");
}

void CarTest_CameraExit(void)
{
    (void)SerialMaixCam_SendCommand("exit");
}

void CarTest_RenderCamera(void)
{
    uint8_t resultCode;
    const char *resultText;

    (void)SerialMaixCam_Process();
    resultCode = SerialMaixCam_GetResultCode();

    switch (resultCode) {
        case SERIAL_MAIXCAM_RESULT_NONE:
            resultText = "无";
            break;

        case SERIAL_MAIXCAM_RESULT_RED_CIRCLE:
            resultText = "红圆";
            break;

        case SERIAL_MAIXCAM_RESULT_RED_SQUARE:
            resultText = "红方";
            break;

        case SERIAL_MAIXCAM_RESULT_GREEN_CIRCLE:
            resultText = "绿圆";
            break;

        case SERIAL_MAIXCAM_RESULT_GREEN_SQUARE:
            resultText = "绿方";
            break;

        default:
            resultText = "Unknown";
            break;
    }

    OLED_Clear();
    OLED_Printf(0, 0, OLED_8X16, "Camera Test");
    OLED_Printf(0, 16, OLED_8X16, "%s", resultText);
    OLED_Printf(0, 32, OLED_6X8, "Code:0x%02X", (unsigned int)resultCode);
    OLED_Printf(0, 40, OLED_6X8, "Drop:%lu",
                (unsigned long)SerialMaixCam_GetTxDropCount());
    OLED_Printf(0, 56, OLED_6X8, "K2:Look K4:Back");
    OLED_Update();
}

void CarTest_RenderOledChinese(void)
{
    OLED_Clear();
    OLED_Printf(0, 0,  OLED_8X16, "一二三四五");
    OLED_Printf(0, 16, OLED_8X16, "红绿圆方弃");
    OLED_Printf(0, 32, OLED_8X16, "巡检用时秒");
    OLED_Printf(0, 56, OLED_6X8, "K4:Back");
    OLED_Update();
}

void CarTest_RenderBuzzerLed(void)
{
    OLED_Clear();
    OLED_Printf(0, 0,  OLED_8X16, "Buzzer LED");
    OLED_Printf(0, 16, OLED_6X8, "K1:Red solid");
    OLED_Printf(0, 24, OLED_6X8, "K2:Green blink");
    OLED_Printf(0, 32, OLED_6X8, "K3:All off");
    OLED_Printf(0, 56, OLED_6X8, "K4:Back");
    OLED_Update();
}
