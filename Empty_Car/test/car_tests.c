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

    OLED_Clear();
    OLED_Printf(0, 0, OLED_8X16, "Gray Test");
    OLED_Printf(0, 16, OLED_6X8, "L:%02X R:%02X S:%u",
                (unsigned int)state->grayLeftRaw,
                (unsigned int)state->grayRightRaw,
                (unsigned int)state->grayStatus);
    OLED_Printf(0, 24, OLED_6X8, "1-8 :%c%c%c%c%c%c%c%c",
                bits[0], bits[1], bits[2], bits[3],
                bits[4], bits[5], bits[6], bits[7]);
    OLED_Printf(0, 32, OLED_6X8, "9-16:%c%c%c%c%c%c%c%c",
                bits[8], bits[9], bits[10], bits[11],
                bits[12], bits[13], bits[14], bits[15]);
    OLED_Printf(0, 48, OLED_6X8, "1=Black 0=White");
    OLED_Printf(0, 56, OLED_6X8, "K4:Back");
    OLED_Update();
}

void CarTest_CameraEnter(void)
{
    (void)SerialMaixCam_SendCommand("start communication");
}

void CarTest_RenderCamera(void)
{
    const char *cmd;

    (void)SerialMaixCam_Process();
    cmd = SerialMaixCam_GetCommand();

    OLED_Clear();
    OLED_Printf(0, 0, OLED_8X16, "Camera Test");
    OLED_Printf(0, 16, OLED_6X8, "Send:start comm");
    OLED_Printf(0, 24, OLED_6X8, "RX:");
    OLED_Printf(18, 24, OLED_6X8, "%s", cmd);
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
