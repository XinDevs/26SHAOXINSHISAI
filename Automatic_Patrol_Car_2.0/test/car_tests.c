#include "car_tests.h"

#include "Flash.h"
#include "OLED.h"
#include "grayscale_sensor.h"
#include "serial_maixcam.h"
#include "pid.h"
#include "dc_motor.h"
#include "encoder.h"
#include "main.h"

#define BASE_SPEED_TEST        (0.5f)
#define PWM_TEST_DUTY          (30)

static const char *s_cameraLastCmdName = "";
static uint8_t s_cameraLastTxPayload = 0U;
static uint8_t s_cameraLastTxOk = 0U;

static void CarTest_CameraSendRequest(uint8_t payload, const char *cmdName)
{
    s_cameraLastCmdName = cmdName;
    s_cameraLastTxPayload = payload;

    if (payload == SERIAL_MAIXCAM_CMD_START_REQUEST) {
        s_cameraLastTxOk = SerialMaixCam_SendStartRequest();
    } else if (payload == SERIAL_MAIXCAM_CMD_STOP_REQUEST) {
        s_cameraLastTxOk = SerialMaixCam_SendStopRequest();
    } else {
        s_cameraLastTxOk = 0U;
    }
}

/**
 * @brief  速度环测试
 * @details 左右轮以目标速度 0.5m/s 运行，用于测试速度环PID
 */
void CarTest_SpeedLoop(void)
{
    PID_GoalSpeedPair_Set(BASE_SPEED_TEST, BASE_SPEED_TEST);
    PID_ExecuteSpeedInnerLoop();
}

/**
 * @brief  PWM测试
 * @details 右电机以固定占空比(30%)输出，用于测试电机驱动
 */
void CarTest_PwmTest(void)
{
    DCMotor_SetDuty(0, PWM_TEST_DUTY);
}

void CarTest_RenderSpeedLoop(void)
{
    OLED_Clear();
    OLED_Printf(0, 0,  OLED_8X16, "Speed Loop");
    OLED_Printf(0, 16, OLED_6X8, "Target L/R %.2fm/s", (double)BASE_SPEED_TEST);
    OLED_Printf(0, 24, OLED_6X8, "Spd L%+.2f R%+.2f",
                (double)encoder_get_left_speed_mps(),
                (double)encoder_get_right_speed_mps());
    OLED_Printf(0, 32, OLED_6X8, "Dist %.3f m",
                (double)encoder_get_center_distance_m());
    OLED_Printf(0, 56, OLED_6X8, "K4:Stop & Back");
    OLED_Update();
}

void CarTest_RenderPwm(void)
{
    DCMotor_Status_t status;

    DCMotor_GetStatus(&status);

    OLED_Clear();
    OLED_Printf(0, 0,  OLED_8X16, "PWM Test");
    OLED_Printf(0, 16, OLED_6X8, "Cmd L0%% R%d%%", PWM_TEST_DUTY);
    OLED_Printf(0, 24, OLED_6X8, "Duty L%+3d%% R%+3d%%",
                status.left_duty_percent,
                status.right_duty_percent);
    OLED_Printf(0, 32, OLED_6X8, "Spd L%+.2f R%+.2f",
                (double)encoder_get_left_speed_mps(),
                (double)encoder_get_right_speed_mps());
    OLED_Printf(0, 56, OLED_6X8, "K4:Stop & Back");
    OLED_Update();
}

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
    OLED_Printf(0, 16, OLED_6X8, "4C:%02X(%u) 4D:%02X(%u)",
                (unsigned int)state->grayLeftRaw,
                (unsigned int)leftOk,
                (unsigned int)state->grayRightRaw,
                (unsigned int)rightOk);
    OLED_Printf(0, 24, OLED_6X8, "4C:0-7  4D:8-15");
    OLED_Printf(0, 32, OLED_6X8, "%s", bitsLine);
    OLED_Printf(0, 48, OLED_6X8, "0=OK 3=I2C ERR");
    OLED_Printf(0, 56, OLED_6X8, "K4:Back");
    OLED_Update();
}

void CarTest_CameraEnter(void)
{
    CarTest_CameraSendRequest(SERIAL_MAIXCAM_CMD_START_REQUEST, "start");
}

void CarTest_CameraLook(void)
{
    CarTest_CameraSendRequest(SERIAL_MAIXCAM_CMD_START_REQUEST, "start");
}

void CarTest_CameraExit(void)
{
    CarTest_CameraSendRequest(SERIAL_MAIXCAM_CMD_STOP_REQUEST, "stop");
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
    OLED_Printf(0, 32, OLED_6X8, "RX:FF 01 %02X FE", (unsigned int)resultCode);
    OLED_Printf(0, 40, OLED_6X8, "TX:%u FF01%02XFE",
                (unsigned int)s_cameraLastTxOk,
                (unsigned int)s_cameraLastTxPayload);
    OLED_Printf(0, 48, OLED_6X8, "Cmd:%s", s_cameraLastCmdName);
    OLED_Printf(84, 48, OLED_6X8, "D:%lu",
                (unsigned long)SerialMaixCam_GetTxDropCount());
    OLED_Printf(0, 56, OLED_6X8, "K2:Start K4:Stop");
    OLED_Update();
}

void CarTest_RenderOledChinese(void)
{
    OLED_Clear();
    OLED_Printf(0, 0,  OLED_8X16, "一二三四五");
    OLED_Printf(104, 0, OLED_6X8, "16");
    OLED_Printf(0, 16, OLED_8X16, "红绿圆方弃");
    OLED_Printf(104, 16, OLED_6X8, "16");
    OLED_Printf(0, 32, OLED_12X12, "一二三四五");
    OLED_Printf(104, 32, OLED_6X8, "12");
    OLED_Printf(0, 44, OLED_12X12, "红绿圆方弃");
    OLED_Printf(104, 44, OLED_6X8, "12");
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
