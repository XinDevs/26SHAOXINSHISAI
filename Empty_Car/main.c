/**
 * @file    main.c
 * @brief   智能巡线小车主程序
 * @details 基于 TI MSPM0G3507 的双轮差速小车，包含以下运行任务：
 *          - 任务1：速度环测试，左右轮目标速度 0.5m/s
 *          - 任务2：灰度循迹，灰度外环 + 速度内环串级 PID
 *          - 任务3：航向保持直行，Yaw 外环 + 速度内环串级 PID
 *          - 任务4：左右电机 PWM 测试，固定占空比输出
 *          定时器 1ms 中断驱动：10ms IMU 更新、20ms PID 触发、100ms OLED 刷新。
 *          支持串口在线调参和 OLED 菜单测试页。
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
#include "serial_report.h"
#include "oled_ui.h"
#include "OLED.h"
#include "grayscale_sensor.h"
#include "MahonyAHRS.h"
#include "icm42688_driver.h"
#include "buzzer_led.h"
#include "main.h"
#include "menu.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ===== ISR 写入、主循环读取的时序标志 ===== */
static volatile uint8_t  g_oledRefreshFlag      = 1U;
static volatile uint16_t g_speedPidPendingCount = 0U;
static volatile uint32_t g_sysTickMs            = 0U;

/* ===== 共享状态变量，供菜单等模块访问 ===== */
uint8_t  g_taskId            = 0U;   /* 当前任务编号: 0=待机, 1=速度环, 2=灰度, 3=航向角, 4=PWM测试 */
float    target_straight_yaw = 0.0f; /* 任务3航向直行的目标航向角(deg) */

/* ===== IMU 航向角反馈量，全局共享 ===== */
volatile float g_currentYaw = 0.0f;

/* 陀螺仪方向修正系数 */
static float GYRO_DIR_X = 1.0f;
static float GYRO_DIR_Y = 1.0f;
static float GYRO_DIR_Z = -1.0f;  /* Z 轴反向，适配安装方向 */

/**
 * @brief  系统主入口
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
    OLEDUI_InitStatus(imuId);

    DCMotor_Init();
    encoder_init();
    PID_Init();
    PID_UpdateYawFeedback(g_currentYaw);

    DCMotor_Enable(1U);
    DCMotor_SetDuty(0, 0);

   // IMU_Calibrate();
   // Mahony_Init(100.0f);
    //Get_Acc_ICM42688();
   // Get_Gyro_ICM42688();
    //MahonyAHRSinit(icm42688_acc_x, icm42688_acc_y, icm42688_acc_z, 0.0f, 0.0f, 0.0f);
    //ICM42688_ResetYawZero();

    OLEDUI_InitStatus(imuId);

    NVIC_ClearPendingIRQ(TIMER_FOR_1MS_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_FOR_1MS_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_FOR_1MS_INST);

    Menu_Init();
    /* ===== PID 参数初始化 ===== */
    PID_SPEED_INIT(0.2f, 0.15f, 0.08f,
                   0.2f, 0.15f, 0.08f);
    PID_Grayscale_Init(0.35f, 0.0f, 1.5f);
    PID_Yaw_Init(0.03f, 0.0f, 0.2f);
    /* ===== 主循环 ===== */
    while (1) {
        // DCMotor_SetDuty(0, PWM_TEST_DUTY);
        /* 1) 按键处理 */
        keyNum = Key_GetNum();
        if (keyNum != 0U) {
            Menu_ProcessKey(keyNum);
            g_oledRefreshFlag = 1U;
        }

        if ((Menu_GetMode() == SYS_TASK_RUN) && (g_taskId == 4U)) {
            DCMotor_SetDuty(0, PWM_TEST_DUTY);
        }

        /* 2) 20ms PID 控制事件 */
        if (g_speedPidPendingCount > 0U) {
            g_speedPidPendingCount = 0U;

            (void)grayscale_update_sensor_array();

            /* 仅在任务运行模式下执行 PID 控制 */
            if (Menu_GetMode() != SYS_TASK_RUN) {
                goto pid_done;
            }
            if (g_taskId == 4U) {
                goto pid_done;
            }

            switch (g_taskId) {
                case 1U:
                    /* 速度环测试：左右轮目标速度 0.5m/s */
                    PID_GoalSpeedPair_Set(BASE_SPEED_TEST, BASE_SPEED_TEST);
                    PID_ExecuteSpeedInnerLoop();
                    break;

                case 2U:
                    /* 灰度循迹：灰度外环 + 速度内环串级 PID */
                    PID_ExecuteGrayCascade(BASE_LINE_SPEED,
                                           0.0f,
                                           MAX_LINE_SPEED_DIFF,
                                           g_currentYaw);
                    break;

                case 3U:
                    /* 航向保持直行：Yaw 外环 + 速度内环串级 PID */
                    PID_ExecuteYawCascade(BASE_STRAIGHT_SPEED,
                                          target_straight_yaw,
                                          MAX_YAW_SPEED_DIFF);
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

        /* 5) OLED 刷新(每 100ms) */
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
                case SYS_BUZZER_LED_TEST:
                    Menu_Render();
                    break;

                case SYS_TASK_RUN:
                    OLED_Clear();
                    OLED_Printf(0, 0,  OLED_8X16, "Task %u Running", (unsigned int)g_taskId);
                    OLED_Printf(0, 16, OLED_6X8, "Yaw: %6.1f", (double)g_currentYaw);
                    OLED_Printf(0, 24, OLED_6X8, "Spd L%+.2f R%+.2f",
                                (double)encoder_get_left_speed_mps(),
                                (double)encoder_get_right_speed_mps());
                    OLED_Printf(0, 32, OLED_6X8, "Dist %.3f m",
                                (double)encoder_get_center_distance_m());
                    OLED_Printf(0, 56, OLED_6X8, "K4:Stop & Back");
                    OLED_Update();
                    if (g_taskId == 1U) {
                        SerialReport_Task1Speed();
                    }
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
            g_currentYaw = ICM42688_GetYawZeroedDeg();
            PID_UpdateYawFeedback(g_currentYaw);
        }
    }

    /* [1ms] Buzzer/LED non-blocking service */
    BuzzerLed_Tick1ms();
}
