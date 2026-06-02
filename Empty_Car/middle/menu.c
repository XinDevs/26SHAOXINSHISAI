/**
 * @file    menu.c
 * @brief   OLED 树状菜单系统实现
 */
#include "menu.h"
#include "main.h"
#include "OLED.h"
#include "pid.h"
#include "dc_motor.h"
#include "encoder.h"
#include "grayscale_sensor.h"
#include "icm42688_driver.h"
#include "buzzer_led.h"
#include "Flash.h"
#include "oled_ui.h"
#include "test/car_tests.h"
#include "ti_msp_dl_config.h"
#include <string.h>

/* ===== 菜单树定义 =====
 * 索引布局:
 *  [0]  Tasks           (NODE, 5 项)
 *  [1]  Speed 0.5m/s    (LEAF)
 *  [2]  Gray Line       (LEAF)
 *  [3]  Yaw Straight    (LEAF)
 *  [4]  PWM Test        (LEAF)
 *  [5]  Reserved        (LEAF)
 *  [6]  Status          (LEAF)
 *  [7]  Tests           (NODE, 5 项)
 *  [8]  Camera Test     (LEAF)
 *  [9]  Gray Test       (LEAF)
 *  [10] Flash Test      (LEAF)
 *  [11] OLED CN Test    (LEAF)
 *  [12] Buzzer LED Test (LEAF)
 *  [13] Settings        (NODE, 1 项)
 *  [14] PID Tune        (NODE, 4 项)
 *  [15] Left Wheel PID  (LEAF)
 *  [16] Right Wheel PID (LEAF)
 *  [17] Gray PID        (LEAF)
 *  [18] Yaw PID         (LEAF)
 */

#define MENU_STACK_MAX   4
#define MENU_VISIBLE_ROWS 6

/* ===== 内部状态 ===== */
static SystemMode_t  s_mode         = SYS_MENU;
static uint8_t       s_stack[MENU_STACK_MAX];
static uint8_t       s_depth        = 0U;
static uint8_t       s_cursor       = 0U;
static uint8_t       s_scroll       = 0U;
static uint8_t       s_currentParent = 0U;

/* PID 编辑模式独立处理 */
static PIDEditPage_t s_pidEditPage  = PID_EDIT_LEFT;
static uint8_t       s_pidParamIdx  = 0U;
static float         s_pidEditKp    = 0.0f;
static float         s_pidEditKi    = 0.0f;
static float         s_pidEditKd    = 0.0f;
static uint8_t       s_pidDirty     = 0U;

static CarTestState_t s_testState = {0};

/* Action functions */
static void actionSpeedLoop(void) { g_taskId = 1; Menu_SetMode(SYS_TASK_RUN); }
static void actionGrayLine(void) { g_taskId = 2; Menu_SetMode(SYS_TASK_RUN); }
static void actionYawStraight(void) { g_taskId = 3; target_straight_yaw = g_currentYaw; Menu_SetMode(SYS_TASK_RUN); }
static void actionPwmTest(void) { g_taskId = 4; Menu_SetMode(SYS_TASK_RUN); }
static void actionMonitor(void) { Menu_SetMode(SYS_MONITOR); }
static void actionCameraTest(void) { CarTest_CameraEnter(); Menu_SetMode(SYS_CAMERA_TEST); }
static void actionGrayTest(void) { Menu_SetMode(SYS_GRAY_TEST); }
static void actionFlashTest(void) { CarTest_FlashRun(&s_testState); Menu_SetMode(SYS_FLASH_TEST); }
static void actionOledCnTest(void) { Menu_SetMode(SYS_OLED_CN_TEST); }
static void actionBuzzerLedTest(void) { Menu_SetMode(SYS_BUZZER_LED_TEST); }
static void actionPIDEditLeft(void) { Menu_SetPIDEditPage(PID_EDIT_LEFT); s_pidParamIdx = 0U; s_pidDirty = 0U; Menu_SetMode(SYS_PID_EDIT); }
static void actionPIDEditRight(void) { Menu_SetPIDEditPage(PID_EDIT_RIGHT); s_pidParamIdx = 0U; s_pidDirty = 0U; Menu_SetMode(SYS_PID_EDIT); }
static void actionPIDEditGray(void) { Menu_SetPIDEditPage(PID_EDIT_GRAY); s_pidParamIdx = 0U; s_pidDirty = 0U; Menu_SetMode(SYS_PID_EDIT); }
static void actionPIDEditYaw(void) { Menu_SetPIDEditPage(PID_EDIT_YAW); s_pidParamIdx = 0U; s_pidDirty = 0U; Menu_SetMode(SYS_PID_EDIT); }

static const MenuItem_t menuItems[] = {
    /* [0] 任务 */
    { "Tasks",           MENU_NODE,  NULL,               1, 5 },
    /* [1] 速度环测试 */
    { "Speed 0.5m/s",    MENU_LEAF,  actionSpeedLoop,    0, 0 },
    /* [2] 灰度循迹 */
    { "Gray Line",       MENU_LEAF,  actionGrayLine,     0, 0 },
    /* [3] 航向保持直行 */
    { "Yaw Straight",    MENU_LEAF,  actionYawStraight,  0, 0 },
    /* [4] PWM test */
    { "PWM Test",        MENU_LEAF,  actionPwmTest,      0, 0 },
    /* [5] Reserved task */
    { "Reserved",        MENU_LEAF,  NULL,               0, 0 },
    /* [6] 状态监测 */
    { "Status",          MENU_LEAF,  actionMonitor,      0, 0 },
    /* [7] Tests */
    { "Tests",           MENU_NODE,  NULL,               8, 5 },
    /* [8] Camera Test */
    { "Camera Test",     MENU_LEAF,  actionCameraTest,   0, 0 },
    /* [9] Gray Test */
    { "Gray Test",       MENU_LEAF,  actionGrayTest,     0, 0 },
    /* [10] Flash Test */
    { "Flash Test",      MENU_LEAF,  actionFlashTest,    0, 0 },
    /* [11] OLED CN Test */
    { "OLED CN Test",    MENU_LEAF,  actionOledCnTest,   0, 0 },
    /* [12] 蜂鸣器和 LED 测试 */
    { "Buzzer LED Test", MENU_LEAF,  actionBuzzerLedTest, 0, 0 },
    /* [13] 设置 */
    { "Settings",        MENU_NODE,  NULL,               14, 1 },
    /* [14] PID 管理 */
    { "PID Tune",        MENU_NODE,  NULL,               15, 4 },
    /* [15] 左轮 PID */
    { "Left Wheel PID",  MENU_LEAF,  actionPIDEditLeft,  0, 0 },
    /* [16] 右轮 PID */
    { "Right Wheel PID", MENU_LEAF,  actionPIDEditRight, 0, 0 },
    /* [17] 灰度 PID */
    { "Gray PID",        MENU_LEAF,  actionPIDEditGray,  0, 0 },
    /* [18] Yaw PID */
    { "Yaw PID",         MENU_LEAF,  actionPIDEditYaw,   0, 0 },
};

#define MENU_ITEM_COUNT  (sizeof(menuItems) / sizeof(menuItems[0]))

/* ===== 内部状态 ===== */

static uint8_t Menu_GetStart(void)
{
    if (s_depth == 0U) {
        return 0U;
    }
    return menuItems[s_currentParent].childStart;
}

static uint8_t Menu_GetCount(void)
{
    if (s_depth == 0U) {
        /* 主菜单: Tasks(0), Tests(7), Status(6), Settings(13) */
        uint8_t cnt = 0U;
        uint8_t i;
        for (i = 0U; i < MENU_ITEM_COUNT; i++) {
            if (i == 0U || i == 6U || i == 7U || i == 13U) {
                cnt++;
            }
        }
        return cnt;
    }
    return menuItems[s_currentParent].childCount;
}

static uint8_t Menu_GetItemIndex(uint8_t relPos)
{
    uint8_t start = Menu_GetStart();
    if (s_depth == 0U) {
        /* 椤跺眰鑿滃崟绱㈠紩鏄犲皠 */
        static const uint8_t topLevel[] = { 0, 7, 6, 13 };
        if (relPos < sizeof(topLevel)) {
            return topLevel[relPos];
        }
        return 0U;
    }
    return (uint8_t)(start + relPos);
}

static void Menu_AdjustScroll(void)
{
    uint8_t count = Menu_GetCount();
    if (s_cursor >= count) {
        s_cursor = (count > 0U) ? (uint8_t)(count - 1U) : 0U;
    }
    if (s_cursor < s_scroll) {
        s_scroll = s_cursor;
    }
    if (s_cursor >= (uint8_t)(s_scroll + MENU_VISIBLE_ROWS)) {
        s_scroll = (uint8_t)(s_cursor - MENU_VISIBLE_ROWS + 1U);
    }
}

/* ===== 内部状态 ===== */

void Menu_Init(void)
{
    s_mode = SYS_MENU;
    s_depth = 0U;
    s_cursor = 0U;
    s_scroll = 0U;
    s_currentParent = 0U;
}

SystemMode_t Menu_GetMode(void)
{
    return s_mode;
}

PIDEditPage_t Menu_GetPIDEditPage(void)
{
    return s_pidEditPage;
}

void Menu_SetMode(SystemMode_t mode)
{
    s_mode = mode;
}

void Menu_SetPIDEditPage(PIDEditPage_t page)
{
    s_pidEditPage = page;
}

void Menu_ExitToMenu(void)
{
    s_mode = SYS_MENU;
    s_depth = 0U;
    s_cursor = 0U;
    s_scroll = 0U;
    s_currentParent = 0U;
}

void Menu_ProcessKey(uint8_t key)
{
    uint8_t count;
    uint8_t idx;
    const MenuItem_t *item;

    if (key == 0U) {
        return;
    }

    /* PID 编辑模式独立处理 */
    if (s_mode == SYS_PID_EDIT) {
        switch (key) {
            case 1: /* K1: - */
                if (s_pidParamIdx == 0U) s_pidEditKp -= 0.01f;
                else if (s_pidParamIdx == 1U) s_pidEditKi -= 0.01f;
                else s_pidEditKd -= 0.01f;
                s_pidDirty = 1U;
                break;
            case 3: /* K3: + */
                if (s_pidParamIdx == 0U) s_pidEditKp += 0.01f;
                else if (s_pidParamIdx == 1U) s_pidEditKi += 0.01f;
                else s_pidEditKd += 0.01f;
                s_pidDirty = 1U;
                break;
            case 2: /* K2: next parameter */
                s_pidParamIdx = (uint8_t)((s_pidParamIdx + 1U) % 3U);
                break;
            case 4: /* K4: Save & Back */
                if (s_pidDirty != 0U) {
                    PID_ITEM pidItem;
                    switch (s_pidEditPage) {
                        case PID_EDIT_LEFT:  pidItem = MOTOR_LEFT;  break;
                        case PID_EDIT_RIGHT: pidItem = MOTOR_RIGHT; break;
                        case PID_EDIT_GRAY:  pidItem = GRAYSCALE;   break;
                        default:             pidItem = YAW;         break;
                    }
                    PID_SetParameters(pidItem, s_pidEditKp, s_pidEditKi, s_pidEditKd);
                }
                s_mode = SYS_MENU;
                break;
            default:
                break;
        }
        return;
    }

    /* 监测模式: K4 返回 */
    if (s_mode == SYS_MONITOR) {
        if (key == 4U) {
            s_mode = SYS_MENU;
        }
        return;
    }

    if (s_mode == SYS_FLASH_TEST) {
        if (key == 4U) {
            s_mode = SYS_MENU;
        }
        return;
    }

    if (s_mode == SYS_GRAY_TEST) {
        if (key == 4U) {
            s_mode = SYS_MENU;
        }
        return;
    }

    if (s_mode == SYS_CAMERA_TEST) {
        if (key == 2U) {
            CarTest_CameraLook();
        } else if (key == 4U) {
            CarTest_CameraExit();
            s_mode = SYS_MENU;
        }
        return;
    }

    if (s_mode == SYS_OLED_CN_TEST) {
        if (key == 4U) {
            s_mode = SYS_MENU;
        }
        return;
    }

    if (s_mode == SYS_BUZZER_LED_TEST) {
        switch (key) {
            case 1U:
                BuzzerLed_StartRedAlert();
                break;
            case 2U:
                BuzzerLed_StartGreenAlert();
                break;
            case 3U:
                BuzzerLed_AllOff();
                break;
            case 4U:
                BuzzerLed_AllOff();
                s_mode = SYS_MENU;
                break;
            default:
                break;
        }
        return;
    }

    /* 任务运行模式: K4 返回 */
    if (s_mode == SYS_TASK_RUN) {
        if (key == 4U) {
            DCMotor_SetDuty(0, 0);
            PID_ResetAll();
            g_taskId = 0U; /* 清除任务编号 */
            s_mode = SYS_MENU;
        }
        return;
    }

    /* ===== 内部状态 ===== */
    count = Menu_GetCount();

    switch (key) {
        case 1: /* K1: Up */
            if (count > 0U) {
                if (s_cursor == 0U) {
                    s_cursor = (uint8_t)(count - 1U);
                } else {
                    s_cursor--;
                }
            }
            break;

        case 3: /* K3: Down */
            if (count > 0U) {
                s_cursor++;
                if (s_cursor >= count) {
                    s_cursor = 0U;
                }
            }
            break;

        case 2: /* K2: Enter */
            if (count == 0U) break;
            idx = Menu_GetItemIndex(s_cursor);
            item = &menuItems[idx];
            if (item->type == MENU_NODE) {
                if (s_depth < MENU_STACK_MAX) {
                    s_stack[s_depth] = s_currentParent;
                    s_depth++;
                    s_currentParent = idx;
                    s_cursor = 0U;
                    s_scroll = 0U;
                }
            } else {
                if (item->action != NULL) {
                    item->action();
                }
            }
            break;

        case 4: /* K4: Back */
            if (s_depth > 0U) {
                s_depth--;
                s_currentParent = s_stack[s_depth];
                s_cursor = 0U;
                s_scroll = 0U;
            }
            break;

        default:
            break;
    }

    Menu_AdjustScroll();
}

/* ===== 菜单渲染 ===== */
static void Menu_RenderMenu(void)
{
    uint8_t count = Menu_GetCount();
    uint8_t i;
    uint8_t idx;
    const char *title;

    OLED_Clear();

    if (s_depth == 0U) {
        title = "Main Menu";
    } else {
        title = menuItems[s_currentParent].name;
    }
    OLED_Printf(0, 0, OLED_8X16, "%s", title);

    for (i = 0U; i < MENU_VISIBLE_ROWS; i++) {
        uint8_t itemPos = (uint8_t)(s_scroll + i);
        if (itemPos >= count) break;

        idx = Menu_GetItemIndex(itemPos);
        uint8_t y = (uint8_t)(16U + i * 8U);

        if (menuItems[idx].type == MENU_NODE) {
            OLED_Printf(0, y, OLED_6X8, " %s>", menuItems[idx].name);
        } else {
            OLED_Printf(0, y, OLED_6X8, " %s", menuItems[idx].name);
        }

        if (itemPos == s_cursor) {
            OLED_ReverseArea(0, y, 128U, 8U);
        }
    }

    if (count > MENU_VISIBLE_ROWS) {
        uint8_t barH = (uint8_t)((MENU_VISIBLE_ROWS * 8U) / count);
        uint8_t barY = (uint8_t)(16U + (uint8_t)((s_scroll * 8U * MENU_VISIBLE_ROWS) / count));
        OLED_DrawRectangle(126, 16, 2U, 48U, OLED_UNFILLED);
        OLED_DrawRectangle(126, barY, 2U, barH, OLED_FILLED);
    }

    OLED_Update();
}

/* ===== 监测页面渲染 ===== */
static void Menu_RenderMonitor(void)
{
    float grayDiff, yawDiff;
    DCMotor_Status_t ms;
    int16_t ld, rd;
    uint8_t i;
    char bits[GW_GRAY_CHANNEL_COUNT + 1U];

    PID_GetOuterDiffs(&grayDiff, &yawDiff);
    DCMotor_GetStatus(&ms);
    PID_GetDutyCmd(&ld, &rd);

    for (i = 0U; i < GW_GRAY_CHANNEL_COUNT; i++) {
        bits[i] = (sensor[i] != 0U) ? '1' : '0';
    }
    bits[GW_GRAY_CHANNEL_COUNT] = '\0';

    OLED_Clear();
    OLED_Printf(0, 0,  OLED_6X8, "Y%6.1f G%+.2f Y%+.2f",
                (double)g_currentYaw, (double)grayDiff, (double)yawDiff);
    OLED_Printf(0, 8,  OLED_6X8, "Spd L%+.2f R%+.2f",
                (double)encoder_get_left_speed_mps(),
                (double)encoder_get_right_speed_mps());
    OLED_Printf(0, 16, OLED_6X8, "Dist %.3f m",
                (double)encoder_get_center_distance_m());
    OLED_Printf(0, 24, OLED_6X8, "G:%s", bits);
    OLED_Printf(0, 32, OLED_6X8, "Duty L%+3d%% R%+3d%%",
                ms.left_duty_percent, ms.right_duty_percent);
    OLED_Printf(0, 40, OLED_6X8, "Dir  L:%s R:%s",
                DCMotor_DirectionString(ms.left_dir),
                DCMotor_DirectionString(ms.right_dir));
    OLED_Printf(0, 48, OLED_6X8, "Cmd  L%+4d R%+4d", (int)ld, (int)rd);
    OLED_Printf(0, 56, OLED_6X8, "K4:Back");
    OLED_Update();
}

/* ===== 测试和 PID 页面渲染 ===== */
static void Menu_RenderFlashTest(void)
{
    CarTest_RenderFlash(&s_testState);
}

static void Menu_RenderGrayTest(void)
{
    CarTest_RenderGray(&s_testState);
}

static void Menu_RenderCameraTest(void)
{
    CarTest_RenderCamera();
}

static void Menu_RenderOledCnTest(void)
{
    CarTest_RenderOledChinese();
}

static void Menu_RenderBuzzerLedTest(void)
{
    CarTest_RenderBuzzerLed();
}

static void Menu_RenderPIDEdit(void)
{
    const char *title;
    float kp, ki, kd;

    PID_GetParameters(
        (s_pidEditPage == PID_EDIT_LEFT)  ? MOTOR_LEFT :
        (s_pidEditPage == PID_EDIT_RIGHT) ? MOTOR_RIGHT :
        (s_pidEditPage == PID_EDIT_GRAY)  ? GRAYSCALE : YAW,
        &kp, &ki, &kd);

    if (s_pidDirty == 0U && s_pidParamIdx == 0U) {
        s_pidEditKp = kp;
        s_pidEditKi = ki;
        s_pidEditKd = kd;
    }

    switch (s_pidEditPage) {
        case PID_EDIT_LEFT:  title = "Left Wheel PID"; break;
        case PID_EDIT_RIGHT: title = "Right Wheel PID"; break;
        case PID_EDIT_GRAY:  title = "Gray PID"; break;
        default:             title = "Yaw PID"; break;
    }

    OLED_Clear();
    OLED_Printf(0, 0, OLED_6X8, "--- %s ---", title);

    OLED_Printf(0, 8, OLED_6X8, " Kp = %8.3f", (double)s_pidEditKp);
    if (s_pidParamIdx == 0U) {
        OLED_ReverseArea(0, 8, 128U, 8U);
    }

    OLED_Printf(0, 16, OLED_6X8, " Ki = %8.3f", (double)s_pidEditKi);
    if (s_pidParamIdx == 1U) {
        OLED_ReverseArea(0, 16, 128U, 8U);
    }

    OLED_Printf(0, 24, OLED_6X8, " Kd = %8.3f", (double)s_pidEditKd);
    if (s_pidParamIdx == 2U) {
        OLED_ReverseArea(0, 24, 128U, 8U);
    }

    OLED_Printf(0, 40, OLED_6X8, "K1:- K2:next K3:+");
    OLED_Printf(0, 48, OLED_6X8, "K4: Save & Back");

    if (s_pidDirty != 0U) {
        OLED_Printf(0, 56, OLED_6X8, "*modified*");
    }

    OLED_Update();
}

/* ===== 统一渲染入口 ===== */
void Menu_Render(void)
{
    switch (s_mode) {
        case SYS_MENU:
            Menu_RenderMenu();
            break;
        case SYS_MONITOR:
            Menu_RenderMonitor();
            break;
        case SYS_PID_EDIT:
            Menu_RenderPIDEdit();
            break;
        case SYS_FLASH_TEST:
            Menu_RenderFlashTest();
            break;
        case SYS_GRAY_TEST:
            Menu_RenderGrayTest();
            break;
        case SYS_CAMERA_TEST:
            Menu_RenderCameraTest();
            break;
        case SYS_OLED_CN_TEST:
            Menu_RenderOledCnTest();
            break;
        case SYS_BUZZER_LED_TEST:
            Menu_RenderBuzzerLedTest();
            break;
        default:
            break;
    }
}
