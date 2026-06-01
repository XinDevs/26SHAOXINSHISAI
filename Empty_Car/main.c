/**
 * @file    main.c
 * @brief   通用智能巡线小车主程序
 * @details 基于TI MSPM0G3507的双轮差速小车，包含以下通用任务模式：
 *          - 任务1: 灰度巡线（灰度外环 + 速度内环串级PID）
 *          - 任务2: 航向保持直行（Yaw外环 + 速度内环串级PID）
 *          - 任务3: 电机测试（固定占空比前进）
 *          定时器1ms中断驱动：10ms IMU更新、20ms PID触发、100ms OLED刷新。
 *          支持串口在线调参（PID参数、灰度权重、目标速度等）。
 */

#include "ti_msp_dl_config.h"
#include "Serial.h"
#include "key.h"
#include "Flash.h"
#include "dc_motor.h"
#include "encoder.h"
#include "pid.h"
#include "serial_cmd.h"
#include "serial_maixcam.h"
#include "oled_ui.h"
#include "OLED.h"
#include "grayscale_sensor.h"
#include "MahonyAHRS.h"
#include "icm42688_driver.h"
#include "main.h"
#include "menu.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ===== ISR写/主循环读 的时序标志 ===== */
static volatile uint8_t  g_oledRefreshFlag      = 1U;
static volatile uint16_t g_speedPidPendingCount = 0U;
static volatile uint32_t g_sysTickMs            = 0U;

/* ===== 共享状态变量(菜单等模块可访问) ===== */
uint8_t  g_taskId            = 0U;   /* 当前任务编号: 0=待机, 1=巡线, 2=航向直行, 3=电机测试 */
float    target_straight_yaw = 0.0f; /* 任务2航向直行目标航向角(deg) */
volatile uint8_t g_buzzerRequestFlag = 0U; /* 蜂鸣器请求标志 */

/* ===== 蜂鸣器非阻塞控制 ===== */
static volatile uint16_t g_buzzerRemainMs       = 0U;

/* ===== IMU航向角反馈量(全局, ISR写, 主循环+pid.c读) ===== */
volatile float g_currentYaw = 0.0f;

/* 陀螺仪方向修正系数 */
static float GYRO_DIR_X = 1.0f;
static float GYRO_DIR_Y = 1.0f;
static float GYRO_DIR_Z = -1.0f;  /* Z轴反转, 适配安装方向 */

/**
 * @brief  系统主入口(超级循环)
 */
int main(void)
{
    int16_t leftDutyCmdNow, rightDutyCmdNow;
    uint8_t keyNum;
    uint8_t imuId;

    /* ===== 外设初始化 ===== */
    SYSCFG_DL_init();
    Serial0_Init();
    Serial1_Init();
    SerialMaixCam_Init();
    Flash_Init();
    Serial0_SendString("@BOOT:Serial Ready\r\n");
    imuId = Init_ICM42688();

    DCMotor_Init();
    encoder_init();
    PID_Init();
    PID_UpdateYawFeedback(g_currentYaw);

    DCMotor_Enable(1U);
    DCMotor_SetDuty(0, 0);

    OLEDUI_InitStatus(imuId);

    IMU_Calibrate();
    Mahony_Init(100.0f);
    Get_Acc_ICM42688();
    Get_Gyro_ICM42688();
    MahonyAHRSinit(icm42688_acc_x, icm42688_acc_y, icm42688_acc_z, 0.0f, 0.0f, 0.0f);

    NVIC_ClearPendingIRQ(TIMER_FOR_1MS_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_FOR_1MS_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_FOR_1MS_INST);

    Menu_Init();

    /* ===== 主循环 ===== */
    while (1) {
        /* 1) 按键处理 */
        keyNum = Key_GetNum();
        if (keyNum != 0U) {
            Menu_ProcessKey(keyNum);
            g_oledRefreshFlag = 1U;
        }

        if ((Menu_GetMode() == SYS_TASK_RUN) && (g_taskId == 3U)) {
            DCMotor_SetDuty(30, 30);
        }

        /* 2) 20ms PID 控制事件 */
        if (g_speedPidPendingCount > 0U) {
            g_speedPidPendingCount = 0U;

            (void)grayscale_update_sensor_array();

            /* 仅在任务运行模式下执行PID控制 */
            if (Menu_GetMode() != SYS_TASK_RUN) {
                goto pid_done;
            }

            if (g_taskId == 3U) {
                goto pid_done;
            }

            switch (g_taskId) {
                case 1U:
                    /* 灰度巡线: 灰度外环 + 速度内环串级PID */
                    PID_ExecuteGrayCascade(BASE_LINE_SPEED,
                                           0.0f,
                                           MAX_LINE_SPEED_DIFF,
                                           g_currentYaw);
                    break;

                case 2U:
                    /* 航向保持直行: Yaw外环 + 速度内环串级PID */
                    PID_ExecuteYawCascade(target_straight_yaw,
                                          BASE_STRAIGHT_SPEED,
                                          MAX_YAW_SPEED_DIFF);
                    break;

                case 4U:
                    /* 中文字模测试: 停车，仅显示 */
                    DCMotor_SetDuty(0, 0);
                    break;

                default:
                    PID_ResetRuntimeState(g_currentYaw);
                    break;
            }

            PID_GetDutyCmd(&leftDutyCmdNow, &rightDutyCmdNow);
            DCMotor_SetDuty(leftDutyCmdNow, rightDutyCmdNow);
            pid_done:;
        }

        /* 3) 串口命令处理 */
        if (SerialCmd_Process(g_currentYaw) != 0U) {
            continue;
        }

        /* 4) MaixCam 通信 */
        (void)SerialMaixCam_Process();

        /* 5) OLED刷新(每100ms) */
        if (g_oledRefreshFlag != 0U) {
            g_oledRefreshFlag = 0U;

            switch (Menu_GetMode()) {
                case SYS_MENU:
                case SYS_MONITOR:
                case SYS_PID_EDIT:
                case SYS_FLASH_TEST:
                case SYS_GRAY_TEST:
                case SYS_CAMERA_TEST:
                case SYS_OLED_CN_TEST:
                    Menu_Render();
                    break;

                case SYS_TASK_RUN:
                    OLED_Clear();
                    if (g_taskId == 4U) {
                        /* 任务4: 中文字模测试 */
                        OLED_Printf(0, 0,  OLED_8X16, "一二三四五");
                        OLED_Printf(0, 16,  OLED_8X16, "红绿圆方弃");
                        OLED_Printf(0, 32,  OLED_8X16, "巡检用时秒");
                    } 
                    else {
                        /* 通用任务运行显示 */
                        OLED_Printf(0, 0,  OLED_8X16, "Task %u Running", (unsigned int)g_taskId);
                        OLED_Printf(0, 16, OLED_6X8, "Yaw: %6.1f", (double)g_currentYaw);
                        OLED_Printf(0, 24, OLED_6X8, "Spd L%+.2f R%+.2f",
                                    (double)encoder_get_left_speed_mps(),
                                    (double)encoder_get_right_speed_mps());
                        OLED_Printf(0, 32, OLED_6X8, "Dist %.3f m",
                                    (double)encoder_get_center_distance_m());
                        OLED_Printf(0, 56, OLED_6X8, "K4:Stop & Back");
                    }
                    OLED_Update();
                    break;

            }
        }
    }
}

/**
 * @brief  1ms 定时中断服务函数
 */
void TIMER_FOR_1MS_INST_IRQHandler(void)
{
    DL_TimerA_clearInterruptStatus(TIMER_FOR_1MS_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    static uint16_t tick10ms  = 0U;
    static uint16_t tick20ms  = 0U;
    static uint16_t tick100ms = 0U;

    /* [1ms] 基础心跳 */
    g_sysTickMs++;
    Key_Tick();

    tick20ms++;
    tick100ms++;
    tick10ms++;

    /* [20ms] 控制事件触发 */
    if (tick20ms >= 20U) {
        tick20ms = 0U;

        if (g_speedPidPendingCount < PID_PENDING_COUNT_MAX) {
            g_speedPidPendingCount++;
        }
    }

    /* [100ms] OLED 刷新事件 */
    if (tick100ms >= 100U) {
        tick100ms = 0U;
        g_oledRefreshFlag = 1U;
    }

    /* [10ms] IMU 更新与 Yaw 解算 */
    if (tick10ms >= 10U) {
        tick10ms = 0U;
        Get_Acc_ICM42688();
        Get_Gyro_ICM42688();
        {
            const float DEG_TO_RAD = 0.01745329252f;
            float gx = icm42688_gyro_x * DEG_TO_RAD * GYRO_DIR_X;
            float gy = icm42688_gyro_y * DEG_TO_RAD * GYRO_DIR_Y;
            float gz = icm42688_gyro_z * DEG_TO_RAD * GYRO_DIR_Z;
            MahonyAHRSupdateIMU(gx, gy, gz, icm42688_acc_x, icm42688_acc_y, icm42688_acc_z);
            g_currentYaw = getYaw();
            PID_UpdateYawFeedback(g_currentYaw);
        }
    }

    /* [1ms] 蜂鸣器 */
    if (g_buzzerRequestFlag != 0U) {
        g_buzzerRequestFlag = 0U;
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN_0_PIN);
        g_buzzerRemainMs = BUZZER_BEEP_MS;
    } else if (g_buzzerRemainMs > 0U) {
        g_buzzerRemainMs--;
        if (g_buzzerRemainMs == 0U) {
            DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN_0_PIN);
        }
    }
}
