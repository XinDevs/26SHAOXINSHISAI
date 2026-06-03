/**
 * @file    menu.c
 * @brief   OLED 树状菜单系统实现
 * @details 基于 4 个按键的层级菜单导航，支持节点菜单和叶子动作。
 *          K1=上移，K3=下移，K2=确认，K4=返回。
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
#include "patrol_info.h"
#include "serial_maixcam.h"
#include "car_turn.h"
#include "logic.h"
#include "test/car_tests.h"
#include "ti_msp_dl_config.h"
#include <string.h>

/* ===== 菜单树定义 =====
 * 索引布局:
 *  [0]  Tasks           (NODE, 5 项)
 *  [1]  Line Auto       (LEAF)
 *  [2]  Line Random     (LEAF)
 *  [3]  Gray Line       (LEAF)
 *  [4]  Junction        (LEAF)
 *  [5]  Reserved 5      (LEAF)
 *  [6]  Patrol Info     (LEAF)
 *  [7]  Status          (LEAF)
 *  [8]  Tests           (NODE, 7 项)
 *  [9]  Speed Loop Test (LEAF)
 *  [10] PWM Test        (LEAF)
 *  [11] Camera Test     (LEAF)
 *  [12] Gray Test       (LEAF)
 *  [13] Flash Test      (LEAF)
 *  [14] OLED CN Test    (LEAF)
 *  [15] Buzzer LED Test (LEAF)
 *  [16] Settings        (NODE, 1 项)
 *  [17] PID Tune        (NODE, 4 项)
 *  [18] Left Wheel PID  (LEAF)
 *  [19] Right Wheel PID (LEAF)
 *  [20] Gray PID        (LEAF)
 *  [21] Yaw PID         (LEAF)
 */

#define MENU_STACK_MAX   4    /* 菜单栈最大深度 */
#define MENU_VISIBLE_ROWS 6   /* OLED 可见行数 */
#define PATROL_INFO_PAGE_SIZE 6U

/* ===== 菜单导航状态 ===== */
static SystemMode_t  s_mode         = SYS_MENU;  /* 当前系统模式 */
static uint8_t       s_stack[MENU_STACK_MAX];     /* 菜单栈 */
static uint8_t       s_depth        = 0U;         /* 当前深度 */
static uint8_t       s_cursor       = 0U;         /* 光标位置 */
static uint8_t       s_scroll       = 0U;         /* 滚动偏移 */
static uint8_t       s_currentParent = 0U;        /* 当前父菜单索引 */
static uint8_t       s_patrolInfoPage = 0U;

/* ===== PID 编辑状态 ===== */
static PIDEditPage_t s_pidEditPage  = PID_EDIT_LEFT;  /* 当前编辑页面 */
static uint8_t       s_pidParamIdx  = 0U;   /* 当前参数索引: 0=Kp, 1=Ki, 2=Kd */
static float         s_pidEditKp    = 0.0f;  /* 编辑中的 Kp */
static float         s_pidEditKi    = 0.0f;  /* 编辑中的 Ki */
static float         s_pidEditKd    = 0.0f;  /* 编辑中的 Kd */
static uint8_t       s_pidDirty     = 0U;    /* 参数是否被修改 */

static CarTestState_t s_testState = {0};  /* 测试状态 */

/* ===== 启动任务 ===== */
static void Menu_StartTask(uint8_t taskId)
{
    TaskId = taskId;
    if (taskId == 1U || taskId == 2U) {
        Logic_Reset();
    }
    PID_Gray_ResetYJunctionState();
    PatrolInfo_Start(SysMs);
    (void)SerialMaixCam_SendStopRequest();
    SerialMaixCam_ClearPending();
    Menu_SetMode(SYS_TASK_RUN);
}

/* ===== 菜单动作回调 ===== */
static void actionAutoGoHome(void)   { Menu_StartTask(1U); }   /* 任务1：自主算法，优先回家 */
static void actionAutoFindAll(void)  { Menu_StartTask(2U); }   /* 任务2：自主算法，查找所有色块 */
static void actionLineRandom(void)   { Menu_StartTask(3U); }   /* 任务3：循迹，路口随机转向 */
static void actionLineTest(void)     { TargetYaw = CurrentYaw; Menu_StartTask(4U); } /* 任务4：循迹测试 */
static void actionPatrolInfo(void) { s_patrolInfoPage = 0U; Menu_SetMode(SYS_PATROL_INFO); }
static void actionMonitor(void) { Menu_SetMode(SYS_MONITOR); }
static void actionSpeedLoopTest(void) { PID_ResetRuntimeState(CurrentYaw); Menu_SetMode(SYS_SPEED_LOOP_TEST); }
static void actionPwmTest(void) { DCMotor_SetDuty(0, 0); Menu_SetMode(SYS_PWM_TEST); }
static void actionCameraTest(void) { CarTest_CameraEnter(); Menu_SetMode(SYS_CAMERA_TEST); }
static void actionGrayTest(void) { Menu_SetMode(SYS_GRAY_TEST); }
static void actionFlashTest(void) { CarTest_FlashRun(&s_testState); Menu_SetMode(SYS_FLASH_TEST); }
static void actionOledCnTest(void) { Menu_SetMode(SYS_OLED_CN_TEST); }
static void actionBuzzerLedTest(void) { Menu_SetMode(SYS_BUZZER_LED_TEST); }
static void actionPIDEditLeft(void) { Menu_SetPIDEditPage(PID_EDIT_LEFT); s_pidParamIdx = 0U; s_pidDirty = 0U; Menu_SetMode(SYS_PID_EDIT); }
static void actionPIDEditRight(void) { Menu_SetPIDEditPage(PID_EDIT_RIGHT); s_pidParamIdx = 0U; s_pidDirty = 0U; Menu_SetMode(SYS_PID_EDIT); }
static void actionPIDEditGray(void) { Menu_SetPIDEditPage(PID_EDIT_GRAY); s_pidParamIdx = 0U; s_pidDirty = 0U; Menu_SetMode(SYS_PID_EDIT); }
static void actionPIDEditYaw(void) { Menu_SetPIDEditPage(PID_EDIT_YAW); s_pidParamIdx = 0U; s_pidDirty = 0U; Menu_SetMode(SYS_PID_EDIT); }

/* ===== 菜单项数组 ===== */
static const MenuItem_t menuItems[] = {
    /* [0] 任务 */
    { "Tasks",           MENU_NODE,  NULL,               1, 5 },
    /* [1] 自主算法：优先回家 */
    { "Auto: Go Home",   MENU_LEAF,  actionAutoGoHome,   0, 0 },
    /* [2] 自主算法：查找所有色块 */
    { "Auto: Find All",  MENU_LEAF,  actionAutoFindAll,  0, 0 },
    /* [3] 循迹：路口随机转向 */
    { "Line: Random",    MENU_LEAF,  actionLineRandom,   0, 0 },
    /* [4] 循迹测试 */
    { "Line: Test",      MENU_LEAF,  actionLineTest,     0, 0 },
    /* [5] Reserved task 5 */
    { "Reserved 5",      MENU_LEAF,  NULL,               0, 0 },
    /* [6] 巡检信息 */
    { "Patrol Info",     MENU_LEAF,  actionPatrolInfo,   0, 0 },
    /* [7] 状态监测 */
    { "Status",          MENU_LEAF,  actionMonitor,      0, 0 },
    /* [8] 测试 */
    { "Tests",           MENU_NODE,  NULL,               9, 7 },
    /* [9] 速度环测试 */
    { "Speed Loop Test", MENU_LEAF,  actionSpeedLoopTest, 0, 0 },
    /* [10] PWM 测试 */
    { "PWM Test",        MENU_LEAF,  actionPwmTest,      0, 0 },
    /* [11] 摄像头测试 */
    { "Camera Test",     MENU_LEAF,  actionCameraTest,   0, 0 },
    /* [12] 灰度测试 */
    { "Gray Test",       MENU_LEAF,  actionGrayTest,     0, 0 },
    /* [13] Flash 测试 */
    { "Flash Test",      MENU_LEAF,  actionFlashTest,    0, 0 },
    /* [14] OLED 中文测试 */
    { "OLED CN Test",    MENU_LEAF,  actionOledCnTest,   0, 0 },
    /* [15] 蜂鸣器 LED 测试 */
    { "Buzzer LED Test", MENU_LEAF,  actionBuzzerLedTest, 0, 0 },
    /* [16] 设置 */
    { "Settings",        MENU_NODE,  NULL,               17, 1 },
    /* [17] PID 调节 */
    { "PID Tune",        MENU_NODE,  NULL,               18, 4 },
    /* [18] 左轮 PID */
    { "Left Wheel PID",  MENU_LEAF,  actionPIDEditLeft,  0, 0 },
    /* [19] 右轮 PID */
    { "Right Wheel PID", MENU_LEAF,  actionPIDEditRight, 0, 0 },
    /* [20] 灰度 PID */
    { "Gray PID",        MENU_LEAF,  actionPIDEditGray,  0, 0 },
    /* [21] 航向 PID */
    { "Yaw PID",         MENU_LEAF,  actionPIDEditYaw,   0, 0 },
};

#define MENU_ITEM_COUNT  (sizeof(menuItems) / sizeof(menuItems[0]))

/* 主菜单顶层项索引: Tasks, Patrol Info, Tests, Status, Settings */
static const uint8_t s_topLevelItems[] = { 0U, 6U, 8U, 7U, 16U };

/* ===== 内部辅助函数 ===== */

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
        return (uint8_t)(sizeof(s_topLevelItems) / sizeof(s_topLevelItems[0]));
    }
    return menuItems[s_currentParent].childCount;
}

static uint8_t Menu_GetItemIndex(uint8_t relPos)
{
    uint8_t start = Menu_GetStart();
    if (s_depth == 0U) {
        if (relPos < sizeof(s_topLevelItems)) {
            return s_topLevelItems[relPos];
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

/* ===== 公共接口 ===== */

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

/* ===== 按键处理 ===== */

void Menu_ProcessKey(uint8_t key)
{
    uint8_t count;
    uint8_t idx;
    const MenuItem_t *item;

    if (key == 0U) {
        return;
    }

    /* PID 编辑模式 */
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

    /* 巡检信息页面：任意键返回 */
    if (s_mode == SYS_PATROL_INFO) {
        PatrolInfoSnapshot_t info;
        uint8_t pageCount;

        PatrolInfo_GetSnapshot(&info);
        pageCount = (info.count > 0U) ?
                    (uint8_t)((info.count + PATROL_INFO_PAGE_SIZE - 1U) / PATROL_INFO_PAGE_SIZE) :
                    1U;

        if ((key == 1U) && (s_patrolInfoPage > 0U)) {
            s_patrolInfoPage--;
        } else if ((key == 3U) && ((uint8_t)(s_patrolInfoPage + 1U) < pageCount)) {
            s_patrolInfoPage++;
        } else if (key == 2U) {
            s_patrolInfoPage++;
            if (s_patrolInfoPage >= pageCount) {
                s_patrolInfoPage = 0U;
            }
        } else if (key == 4U) {
            s_mode = SYS_MENU;
        }
        return;
    }

    /* 状态监测：K4 返回 */
    if (s_mode == SYS_MONITOR) {
        if (key == 4U) {
            s_mode = SYS_MENU;
        }
        return;
    }

    /* Flash 测试：K4 返回 */
    if (s_mode == SYS_FLASH_TEST) {
        if (key == 4U) {
            s_mode = SYS_MENU;
        }
        return;
    }

    /* 灰度测试：K4 返回 */
    if (s_mode == SYS_GRAY_TEST) {
        if (key == 4U) {
            s_mode = SYS_MENU;
        }
        return;
    }

    /* 速度环/PWM 测试：K4 返回 */
    if ((s_mode == SYS_SPEED_LOOP_TEST) || (s_mode == SYS_PWM_TEST)) {
        if (key == 4U) {
            DCMotor_SetDuty(0, 0);
            PID_ResetAll();
            s_mode = SYS_MENU;
        }
        return;
    }

    /* 摄像头测试：K2 查看，K4 返回 */
    if (s_mode == SYS_CAMERA_TEST) {
        if (key == 2U) {
            CarTest_CameraLook();
        } else if (key == 4U) {
            CarTest_CameraExit();
            s_mode = SYS_MENU;
        }
        return;
    }

    /* OLED 中文测试：K4 返回 */
    if (s_mode == SYS_OLED_CN_TEST) {
        if (key == 4U) {
            s_mode = SYS_MENU;
        }
        return;
    }

    /* 蜂鸣器 LED 测试 */
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

    /* 任务运行模式：K4 放弃任务 */
    if (s_mode == SYS_TASK_RUN) {
        if (key == 4U) {
            DCMotor_SetDuty(0, 0);
            PID_ResetAll();
            PID_Gray_ResetYJunctionState();
            PatrolInfo_Cancel();
            (void)SerialMaixCam_SendStopRequest();
            TaskId = 0U;
            s_mode = SYS_MENU;
        }
        return;
    }

    /* 菜单导航模式 */
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
    uint8_t titleX;

    OLED_Clear();

    /* 标题 */
    if (s_depth == 0U) {
        title = "Main";
        titleX = 40U;  /* 居中显示 */
    } else {
        title = menuItems[s_currentParent].name;
        titleX = 0U;
    }
    OLED_Printf(titleX, 0, OLED_8X16, "%s", title);

    /* 菜单项列表 */
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

        /* 反显选中项 */
        if (itemPos == s_cursor) {
            OLED_ReverseArea(0, y, 128U, 8U);
        }
    }

    /* 滚动条 */
    if (count > MENU_VISIBLE_ROWS) {
        uint8_t barH = (uint8_t)((MENU_VISIBLE_ROWS * 8U) / count);
        uint8_t barY = (uint8_t)(16U + (uint8_t)((s_scroll * 8U * MENU_VISIBLE_ROWS) / count));
        OLED_DrawRectangle(126, 16, 2U, 48U, OLED_UNFILLED);
        OLED_DrawRectangle(126, barY, 2U, barH, OLED_FILLED);
    }

    OLED_Update();
}

/* ===== 状态监测渲染 ===== */

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
                (double)CurrentYaw, (double)grayDiff, (double)yawDiff);
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

/* ===== 巡检信息渲染 ===== */

static const char *Menu_PatrolOrderText(uint8_t index)
{
    static const char *orders[PATROL_INFO_MAX_RESULTS] = {
        "一", "二", "三", "四", "五", "六", "七", "八", "九", "十"
    };

    return (index < PATROL_INFO_MAX_RESULTS) ? orders[index] : "";
}

static const char *Menu_PatrolResultText(uint8_t resultCode)
{
    switch (resultCode) {
        case SERIAL_MAIXCAM_RESULT_RED_CIRCLE:
            return "红圆";
        case SERIAL_MAIXCAM_RESULT_RED_SQUARE:
            return "红方";
        case SERIAL_MAIXCAM_RESULT_GREEN_CIRCLE:
            return "绿圆";
        case SERIAL_MAIXCAM_RESULT_GREEN_SQUARE:
            return "绿方";
        default:
            return "None";
    }
}

static void Menu_DrawPatrolRecord(uint8_t x, uint8_t y, uint8_t index, uint8_t resultCode)
{
    OLED_ShowString(x, y, Menu_PatrolOrderText(index), OLED_12X12);
    OLED_ShowString((int16_t)(x + 12U), y, Menu_PatrolResultText(resultCode), OLED_12X12);
}

static void Menu_RenderPatrolInfo(void)
{
    PatrolInfoSnapshot_t info;
    uint8_t line;
    uint8_t pageCount;
    uint8_t firstIndex;

    PatrolInfo_GetSnapshot(&info);

    OLED_Clear();

    if (info.storageValid == 0U) {
        OLED_ShowString(28, 0, "巡检", OLED_12X12);
        OLED_ShowString(28, 24, "None", OLED_12X12);
        OLED_Printf(0, 56, OLED_6X8, "K4:Back");
        OLED_Update();
        return;
    }

    pageCount = (info.count > 0U) ?
                (uint8_t)((info.count + PATROL_INFO_PAGE_SIZE - 1U) / PATROL_INFO_PAGE_SIZE) :
                1U;
    if (s_patrolInfoPage >= pageCount) {
        s_patrolInfoPage = (uint8_t)(pageCount - 1U);
    }
    firstIndex = (uint8_t)(s_patrolInfoPage * PATROL_INFO_PAGE_SIZE);

    OLED_ShowString(0, 0, "巡检用时", OLED_12X12);
    OLED_Printf(54, 2, OLED_6X8, "%lus", (unsigned long)info.elapsedSeconds);

    for (line = 0U; line < 3U; line++) {
        uint8_t first = (uint8_t)(firstIndex + line * 2U);
        uint8_t second = (uint8_t)(first + 1U);
        uint8_t y = (uint8_t)(16U + line * 13U);

        if (first >= info.count) {
            break;
        }

        Menu_DrawPatrolRecord(0U, y, first, info.results[first]);
        if (second < info.count) {
            Menu_DrawPatrolRecord(64U, y, second, info.results[second]);
        }
    }

    if (info.count == 0U) {
        OLED_ShowString(0, 24, "None", OLED_12X12);
    }

    if (pageCount > 1U) {
        OLED_Printf(0, 56, OLED_6X8, "K1/K3 Pg %u/%u K4",
                    (unsigned int)(s_patrolInfoPage + 1U),
                    (unsigned int)pageCount);
    } else {
        OLED_Printf(0, 56, OLED_6X8, "K4:Back");
    }
    OLED_Update();
}

/* ===== 测试页面渲染 ===== */

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

static void Menu_RenderSpeedLoopTest(void)
{
    CarTest_RenderSpeedLoop();
}

static void Menu_RenderPwmTest(void)
{
    CarTest_RenderPwm();
}

static void Menu_RenderOledCnTest(void)
{
    CarTest_RenderOledChinese();
}

static void Menu_RenderBuzzerLedTest(void)
{
    CarTest_RenderBuzzerLed();
}

/* ===== PID 编辑页面渲染 ===== */

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
        case SYS_PATROL_INFO:
            Menu_RenderPatrolInfo();
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
        case SYS_SPEED_LOOP_TEST:
            Menu_RenderSpeedLoopTest();
            break;
        case SYS_PWM_TEST:
            Menu_RenderPwmTest();
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

/* ===== 任务停止/完成 ===== */

/**
 * @brief  放弃任务（滑行停止，不保存巡检）
 */
void Task_GiveUp(void)
{
    DCMotor_SetDuty(0, 0);
    PID_ResetAll();
    PID_Gray_ResetYJunctionState();
    PatrolInfo_Cancel();
    (void)SerialMaixCam_SendStopRequest();
    TaskId = 0U;
    Turn_Reset();
    Menu_SetMode(SYS_MENU);
    OledFlag = 1U;
}

/**
 * @brief  完成任务（刹车停止，保存巡检）
 */
void Task_Finish(void)
{
    DCMotor_Brake();
    PID_ResetAll();
    PID_Gray_ResetYJunctionState();
    PatrolInfo_Finish(SysMs);
    (void)SerialMaixCam_SendStopRequest();
    TaskId = 0U;
    Turn_Reset();
    s_patrolInfoPage = 0U;
    Menu_SetMode(SYS_PATROL_INFO);
    OledFlag = 1U;
}
