/**
 * @file    main.c
 * @brief   智能巡线小车主程序
 * @details 基于 TI MSPM0G3507 的双轮差速小车，包含以下运行任务：
 *          - 任务1：循迹，遇到 Y 路口后停车等待 MaixCam 识别并转向
 *          - 任务2：循迹，遇到 Y 路口后随机左/右转
 *          - 任务3：灰度循迹，灰度外环 + 速度内环串级 PID
 *          - 任务4：灰度循迹，遇到 Y 型岔路口停车
 *          - 任务5：预留
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
#include "pid_gray.h"
#include "serial_cmd.h"
#include "serial_maixcam.h"
#include "serial_report.h"
#include "patrol_info.h"
#include "oled_ui.h"
#include "OLED.h"
#include "grayscale_sensor.h"
#include "MahonyAHRS.h"
#include "icm42688_driver.h"
#include "buzzer_led.h"
#include "test/car_tests.h"
#include "car_turn.h"
#include "main.h"
#include "main_helpers.h"
#include "menu.h"
#include "logic.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ===== ISR 写入、主循环读取的时序标志 ===== */
volatile uint8_t  OledFlag              = 1U;
static volatile uint8_t  ReportFlag     = 0U;
static volatile uint16_t PidPending = 0U;
volatile uint32_t SysMs                   = 0U;

/* ===== 共享状态变量，供菜单等模块访问 ===== */
uint8_t  TaskId            = 0U;
float    TargetYaw = 0.0f;

/* ===== IMU 航向角反馈量，全局共享 ===== */
volatile float CurrentYaw = 0.0f;

/* 陀螺仪方向修正系数 */
static float GyroDirX = 1.0f;
static float GyroDirY = 1.0f;
static float GyroDirZ = -1.0f;  /* Z 轴反向，适配安装方向 */

typedef enum {
    TASK_TRACE = 0,
    TASK_CAMERA,
    TASK_JUNCTION_PAUSE,
    TASK_TURN,
    TASK_FINISH
} TaskState_t;

static TaskState_t TaskState = TASK_TRACE;
static uint32_t CameraStartMs = 0U;
static uint32_t JunctionPauseStartMs = 0U;
static uint8_t CameraReady = 0U;
static uint8_t CameraCode = SERIAL_MAIXCAM_RESULT_NONE;
static uint8_t CameraRecorded = 0U;

static uint8_t g_grayParamProfile = 0U;

/**
 * @brief  系统主入口
 */
int main(void)
{
    int16_t LeftDuty, RightDuty;
    uint8_t TurnDir;
    uint8_t Key;
    uint8_t ImuId;
    uint8_t DutyReady;
    uint8_t LineState;
    SystemMode_t Mode;
    SerialReportSnapshot_t Report;

    /* ===== 外设初始化 ===== */
    SYSCFG_DL_init();
    Serial0_Init();
    Serial1_Init();
    SerialMaixCam_Init();
    Flash_Init();
    PatrolInfo_Init();
    Init_Map();
    Serial0_SendString("@BOOT:Serial Ready\r\n");

    DCMotor_Init();
    encoder_init();
    PID_Init();
    PID_UpdateYawFeedback(CurrentYaw);
    DCMotor_Enable(1U);
    DCMotor_SetDuty(0, 0);
    /* ===== ICM42688 初始化 ===== */
    ImuId = Init_ICM42688();
    OLEDUI_InitStatus(ImuId);
    // IMU_Calibrate();
    // Mahony_Init(100.0f);
    // Get_Acc_ICM42688();
    // Get_Gyro_ICM42688();
    // MahonyAHRSinit(icm42688_acc_x, icm42688_acc_y, icm42688_acc_z, 0.0f, 0.0f, 0.0f);
    // ICM42688_ResetYawZero();

    NVIC_ClearPendingIRQ(TIMER_FOR_1MS_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_FOR_1MS_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_FOR_1MS_INST);

    Menu_Init();
    /* ===== PID 参数初始化 ===== */
    PID_SPEED_INIT(0.2f, 0.15f, 0.08f,
                   0.2f, 0.15f, 0.08f);
    PID_Grayscale_Init(0.5f, 0.0f, 0.15f);
    PID_SetGrayscaleLeftWeights6(0.35f, 0.30f, 0.30f,
                                 0.20f, 0.08f, 0.05f);
    g_grayParamProfile = 1U;
    PID_Yaw_Init(0.03f, 0.0f, 0.2f);
    /* ===== 主循环 ===== */
    while (1) {
        /* 1) 按键处理 */
        Key = Key_GetNum();
        if (Key != 0U) {
            Menu_ProcessKey(Key);
            OledFlag = 1U;
        }

        /* 2) 20ms PID 控制事件 */
        if (PidPending > 0U) {
            PidPending = 0U;
            DutyReady = 0U;
            Mode = Menu_GetMode();

            (void)grayscale_update_sensor_array();

            if ((Mode != SYS_TASK_RUN) || ((TaskId != 1U) && (TaskId != 2U))) {
                TaskState = TASK_TRACE;
                CameraReady = 0U;
                CameraCode = SERIAL_MAIXCAM_RESULT_NONE;
                CameraRecorded = 0U;
                JunctionPauseStartMs = 0U;
                Turn_Reset();
                Main_ResetStartFinishLineState();
            }

            /* 正式任务选择 */
            if (Mode == SYS_TASK_RUN) {
                switch (TaskId) {
                    case 1U:
                        if (g_grayParamProfile != 1U) {
                            PID_Grayscale_Init(0.5f, 0.0f, 0.15f);
                            PID_SetGrayscaleLeftWeights6(0.35f, 0.30f, 0.30f,
                                                         0.20f, 0.08f, 0.05f);
                            PID_Reset(GRAYSCALE);
                            g_grayParamProfile = 1U;
                        }
                        /* 任务1：循迹行驶，遇到 Y 路口时停车等待摄像头给出转向。 */
                        switch (TaskState) {
                            case TASK_TRACE:
                                /*
                                 * 循迹状态：
                                 * - 每 20ms 检查一次横线；
                                 * - 第一次横线只记录为起点，车辆继续循迹；
                                 * - 第二次横线才认为到达终点，切到 TASK_FINISH；
                                 * - 只有未压在横线上时才检测 Y 路口，避免横线全亮误判为路口。
                                 */
                                LineState = Main_CheckStartFinishLineCrossing();
                                if (LineState >= 2U) {
                                    /* 第二次经过起终点线：准备结束任务。 */
                                    TaskState = TASK_FINISH;
                                } 
                                else if ((LineState == 0U) &&
                                           (PID_Gray_IsYJunction() != 0U)) {
                                    /* 检测到 Y 路口：先刹停，再请求 MaixCam 识别红/绿方向。 */
                                    DCMotor_Brake();
                                    PID_ResetAll();
                                    CameraReady = 0U;
                                    CameraCode = SERIAL_MAIXCAM_RESULT_NONE;
                                    CameraRecorded = 0U;
                                    CameraStartMs = SysMs;
                                    SerialMaixCam_ClearPending();
                                    (void)SerialMaixCam_SendStartRequest();//给摄像头发送开始识别的命令 使得CameraReady在收到结果后被置位
                                    TaskState = TASK_CAMERA;
                                }
                                 else {
                                    /* 未到终点、未到路口：正常灰度循迹。 */
                                    PID_ExecuteGrayCascade(BASE_LINE_SPEED,
                                                           0.0f,
                                                           LINE_SPEED_DIFF_SCALE);
                                    DutyReady = 1U;
                                }
                                break;
                            case TASK_CAMERA:
                                /*
                                 * 摄像头等待状态：
                                 * - 收到结果后按 CameraDirection() 转成左右转命令；
                                 * - 超时仍无结果时随机选择方向，防止一直停在路口。
                                 */
                                if (CameraReady != 0U) {
                                    /* MaixCam 返回有效结果按识别结果转向；未识别到标识则随机转向。 */
                                    if (CameraCode == SERIAL_MAIXCAM_RESULT_NONE) {
                                        TurnDir = RandomDirection();
                                    } else {
                                        if (CameraRecorded == 0U) {
                                            PatrolInfo_RecordResult(CameraCode);
                                            CameraRecorded = 1U;
                                        }
                                        TurnDir = CameraDirection(CameraCode);
                                    }
                                    CameraReady = 0U;
                                    CameraCode = SERIAL_MAIXCAM_RESULT_NONE;
                                    (void)SerialMaixCam_SendStopRequest();
                                    SerialMaixCam_ClearPending();
                                    PID_ResetAll();
                                    Turn_Start(TurnDir, SPIN_TO_LINE_SPEED_MPS, SysMs);
                                    TaskState = TASK_TURN;
                                } 
                                else if ((uint32_t)(SysMs - CameraStartMs) >= CAMERA_TURN_TIMEOUT_MS) {
                                    /* 摄像头超时：随机选择方向继续任务。 */
                                    TurnDir = RandomDirection();
                                    (void)SerialMaixCam_SendStopRequest();
                                    SerialMaixCam_ClearPending();
                                    PID_ResetAll();
                                    Turn_Start(TurnDir, SPIN_TO_LINE_SPEED_MPS, SysMs);
                                    TaskState = TASK_TURN;
                                }
                                break;

                            case TASK_TURN:
                                /*
                                 * 转弯状态：
                                 * Turn_Run() 持续执行原地/差速转向；
                                 * Turn_IsDone() 在延时后检测回到线，确认完成后回到循迹。
                                 */
                                Turn_Run();
                                if (Turn_IsDone(SysMs, TURN_LINE_DETECT_DELAY_MS) != 0U) {
                                    /* 已重新压上线：清掉路口锁存和 PID 历史，恢复循迹。 */
                                    Turn_Reset();
                                    PID_ResetAll();
                                    PID_Gray_ResetYJunctionState();
                                    TaskState = TASK_TRACE;
                                    PID_ExecuteGrayCascade(BASE_LINE_SPEED,
                                                           0.0f,
                                                           LINE_SPEED_DIFF_SCALE);
                                    DutyReady = 1U;
                                } else {
                                    /* 还在转弯过程中：继续输出 Turn_Run() 写入的电机指令。 */
                                    DutyReady = 1U;
                                }
                                break;

                            case TASK_FINISH:
                                /* 终点状态：复位横线计数，主动刹车，记录完成信息并返回菜单。 */
                                TaskState = TASK_TRACE;
                                Main_ResetStartFinishLineState();
                                Task_Finish();
                                break;

                            default:
                                /* 异常状态保护：回到循迹并清理转向/PID 状态。 */
                                TaskState = TASK_TRACE;
                                Turn_Reset();
                                PID_ResetAll();
                                break;
                        }
                        break;
                    case 2U:       //任务2 实现自主算法循迹
                        if (g_grayParamProfile != 1U) {
                            PID_Grayscale_Init(0.5f, 0.0f, 0.15f);
                            PID_SetGrayscaleLeftWeights6(0.35f, 0.30f, 0.30f,
                                                         0.20f, 0.08f, 0.05f);
                            PID_Reset(GRAYSCALE);
                            g_grayParamProfile = 1U;
                        }
                        switch (TaskState) {
                            case TASK_TRACE:
                                /*
                                 * 循迹状态：
                                 * - 每 20ms 检查一次横线；
                                 * - 第一次横线只记录为起点，车辆继续循迹；
                                 * - 第二次横线才认为到达终点，切到 TASK_FINISH；
                                 * - 只有未压在横线上时才检测 Y 路口，避免横线全亮误判为路口。
                                 */
                                LineState = Main_CheckStartFinishLineCrossing();
                                if (LineState >= 2U) {
                                    /* 第二次经过起终点线：准备结束任务。 */
                                    TaskState = TASK_FINISH;
                                } 
                                else if ((LineState == 0U) &&
                                           (PID_Gray_IsYJunction() != 0U)) {
                                    /* 检测到 Y 路口：先刹停，再请求 MaixCam 识别红/绿方向。 */
                                    DCMotor_Brake();
                                    PID_ResetAll();
                                    CameraReady = 0U;       // 摄像头不使用，直接进入盲走决策
                                    CameraCode = SERIAL_MAIXCAM_RESULT_NONE;    // 摄像头结果无效
                                    CameraRecorded = 0U;        // 摄像头结果不记录
                                    CameraStartMs = SysMs;      // 记录进入摄像头等待状态的时间戳，用于盲走决策的超时判断
                                    SerialMaixCam_ClearPending();   //
                                    (void)SerialMaixCam_SendStartRequest();
                                    TaskState = TASK_CAMERA;
                                }
                                else {
                                    /* 未到终点、未到路口：正常灰度循迹。 */
                                    PID_ExecuteGrayCascade(BASE_LINE_SPEED,
                                                           0.0f,
                                                           LINE_SPEED_DIFF_SCALE);
                                    DutyReady = 1U;
                                }
                                break;
                            case TASK_CAMERA:
                                /*
                                 * 摄像头等待状态：
                                 * - 收到结果后按 CameraDirection() 转成左右转命令；
                                 * - 超时仍无结果时随机选择方向，防止一直停在路口。
                                 */
                                if (CameraReady != 0U) {
                                    int prev_node = curr_node;
                                    int prev_last = last_node;
                                    char reportBuf[64];
                                    /* MaixCam 返回有效结果按识别结果转向；未识别到标识则随机转向。 */
                                    if (CameraCode == SERIAL_MAIXCAM_RESULT_NONE) {
                                        TurnDir = Logic_Get_Turn_Direction(COLOR_NONE);     //首次默认右转 为0
                                        int lt = (int)map_forward_letter[prev_node][prev_last];
                                        char lc = (lt >= 1 && lt <= 12) ? (char)('A' + lt - 1) : 'N';
                                        snprintf(reportBuf, sizeof(reportBuf),
                                                 "node=%d last=%d color=%c letter=%c found=%d\r\n",
                                                 prev_node, prev_last, 'N', lc, found_count);
                                        Serial0_SendString(reportBuf);
                                    } else {    //这个else里写的是有看到色块的情况
                                        if (CameraRecorded == 0U) {     // 记录摄像头结果是否存储到flash里  0说明没有被存到，1 说明已经存到
                                            PatrolInfo_RecordResult(CameraCode);        //存储到flash
                                            CameraRecorded = 1U;
                                        }
                                        //超绝色块分入到只有颜色的写法 666
                                        uint8_t camColor;
                                        switch(CameraCode) {
                                            case SERIAL_MAIXCAM_RESULT_RED_CIRCLE:
                                            case SERIAL_MAIXCAM_RESULT_RED_SQUARE:
                                                camColor = COLOR_RED;
                                                TurnDir = Logic_Get_Turn_Direction(COLOR_RED);
                                                break;
                                            case SERIAL_MAIXCAM_RESULT_GREEN_CIRCLE:
                                            case SERIAL_MAIXCAM_RESULT_GREEN_SQUARE:
                                                camColor = COLOR_GREEN;
                                                TurnDir = Logic_Get_Turn_Direction(COLOR_GREEN);
                                                break;
                                            default:
                                                camColor = COLOR_NONE;
                                                TurnDir = Logic_Get_Turn_Direction(COLOR_NONE);
                                                break;
                                        }
                                        int lt = (int)map_forward_letter[prev_node][prev_last];
                                        char lc = (lt >= 1 && lt <= 12) ? (char)('A' + lt - 1) : 'N';
                                        char cc = (camColor == COLOR_RED) ? 'R' : (camColor == COLOR_GREEN) ? 'G' : 'N';
                                        snprintf(reportBuf, sizeof(reportBuf),
                                                 "node=%d last=%d color=%c letter=%c found=%d\r\n",
                                                 prev_node, prev_last, cc, lc, found_count);
                                        Serial0_SendString(reportBuf);
                                    }
                                    CameraReady = 0U;
                                    CameraCode = SERIAL_MAIXCAM_RESULT_NONE;
                                    (void)SerialMaixCam_SendStopRequest();
                                    SerialMaixCam_ClearPending();
                                    PID_ResetAll();
                                    Turn_Start(TurnDir, SPIN_TO_LINE_SPEED_MPS, SysMs);
                                    TaskState = TASK_TURN;
                                } 
                                else if ((uint32_t)(SysMs - CameraStartMs) >= CAMERA_TURN_TIMEOUT_MS) {
                                    /* 摄像头超时：随机选择方向继续任务。 */
                                    // TurnDir = Logic_Get_Turn_Direction(COLOR_NONE);
                                    // (void)SerialMaixCam_SendStopRequest();
                                    // SerialMaixCam_ClearPending();
                                    // PID_ResetAll();
                                    // Turn_Start(TurnDir, SPIN_TO_LINE_SPEED_MPS, SysMs);
                                    // TaskState = TASK_TURN;
                                    LED_Red_On();
                                    LED_Green_On();
                                    Motor_Stop();   //测试，如果超时则不动，红绿灯都亮
                                }
                                break;

                            case TASK_TURN:
                                /*
                                 * 转弯状态：
                                 * Turn_Run() 持续执行原地/差速转向；
                                 * Turn_IsDone() 在延时后检测回到线，确认完成后回到循迹。
                                 */
                                Turn_Run();
                                if (Turn_IsDone(SysMs, TURN_LINE_DETECT_DELAY_MS) != 0U) {
                                    /* 已重新压上线：清掉路口锁存和 PID 历史，恢复循迹。 */
                                    Turn_Reset();
                                    PID_ResetAll();
                                    PID_Gray_ResetYJunctionState();
                                    TaskState = TASK_TRACE;
                                    PID_ExecuteGrayCascade(BASE_LINE_SPEED,
                                                           0.0f,
                                                           LINE_SPEED_DIFF_SCALE);
                                    DutyReady = 1U;
                                } else {
                                    /* 还在转弯过程中：继续输出 Turn_Run() 写入的电机指令。 */
                                    DutyReady = 1U;
                                }
                                break;

                            case TASK_FINISH:
                                /* 终点状态：复位横线计数，主动刹车，记录完成信息并返回菜单。 */
                                TaskState = TASK_TRACE;
                                Main_ResetStartFinishLineState();
                                Task_Finish();
                                break;

                            default:
                                /* 异常状态保护：回到循迹并清理转向/PID 状态。 */
                                TaskState = TASK_TRACE;
                                Turn_Reset();
                                PID_ResetAll();
                                break;
                        }
                        break;    

                    case 3U:
                      if (g_grayParamProfile != 3U) {
                            PID_Grayscale_Init(0.8f, 0.0f, 0.8f);
                            PID_SetGrayscaleLeftWeights6(0.35f, 0.30f, 0.30f,
                                                         0.20f, 0.08f, 0.05f);
                            PID_Reset(GRAYSCALE);
                            g_grayParamProfile = 3U;
                        }
                        /* 任务3：灰度陀螺仪循迹，遇到 Y 路口刹停一下再随机转向。 */
                        switch (TaskState) {
                            case TASK_TRACE:
                                LineState = Main_CheckStartFinishLineCrossing();
                                if (LineState >= 2U) {
                                    TaskState = TASK_FINISH;
                                } else if ((LineState == 0U) &&
                                           (PID_Gray_IsYJunction() != 0U)) {
                                    DCMotor_Brake();
                                    PID_ResetAll();
                                    JunctionPauseStartMs = SysMs;
                                    TaskState = TASK_JUNCTION_PAUSE;
                                } else {
                                    PID_ExecuteGrayGyroCascade(0.5f,
                                                               0.0f,
                                                               LINE_SPEED_DIFF_SCALE,
                                                               0.00005f);
                                    DutyReady = 1U;
                                }
                                break;

                            case TASK_JUNCTION_PAUSE:
                                DCMotor_Brake();
                                if ((uint32_t)(SysMs - JunctionPauseStartMs) >= CAMERA_TURN_TIMEOUT_MS) {
                                    TurnDir = RandomDirection();
                                    Turn_Start(TurnDir, SPIN_TO_LINE_SPEED_MPS, SysMs);
                                    TaskState = TASK_TURN;
                                }
                                break;

                            case TASK_TURN:
                                Turn_Run();
                                if (Turn_IsDone(SysMs, TURN_LINE_DETECT_DELAY_MS) != 0U) {
                                    Turn_Reset();
                                    PID_ResetAll();
                                    PID_Gray_ResetYJunctionState();
                                    TaskState = TASK_TRACE;
                                    PID_ExecuteGrayGyroCascade(0.5f,
                                                               0.0f,
                                                               LINE_SPEED_DIFF_SCALE,
                                                               0.0002f);
                                    DutyReady = 1U;
                                } else {
                                    DutyReady = 1U;
                                }
                                break;

                            case TASK_FINISH:
                                TaskState = TASK_TRACE;
                                Main_ResetStartFinishLineState();
                                Task_Finish();
                                break;

                            case TASK_CAMERA:
                            default:
                                TaskState = TASK_TRACE;
                                Turn_Reset();
                                PID_ResetAll();
                                JunctionPauseStartMs = 0U;
                                break;
                        }
                        break;
                    case 4U:
                        if (g_grayParamProfile != 1U) {
                            PID_Grayscale_Init(0.5f, 0.0f, 0.15f);
                            PID_SetGrayscaleLeftWeights6(0.35f, 0.30f, 0.30f,
                                                         0.20f, 0.08f, 0.05f);
                            PID_Reset(GRAYSCALE);
                            g_grayParamProfile = 1U;
                        }
                        /* 灰度循迹，遇到 Y 型岔路口停车。 */
                        if (PID_Gray_IsYJunction() != 0U) {
                            Task_GiveUp();
                        } else {
                            PID_ExecuteGrayCascade(BASE_LINE_SPEED,
                                                   0.0f,
                                                   LINE_SPEED_DIFF_SCALE);
                            DutyReady = 1U;
                        }
                        break;

                    default:
                        PID_ResetRuntimeState(CurrentYaw);
                        TaskState = TASK_TRACE;
                        Turn_Reset();
                        break;
                }
            }

            /* 测试模式选择 */
            switch (Mode) {
                case SYS_SPEED_LOOP_TEST:
                    CarTest_SpeedLoop();
                    DutyReady = 1U;
                    break;

                case SYS_PWM_TEST:
                    CarTest_PwmTest();
                    break;

                default:
                    break;
            }

            if (DutyReady != 0U) {
                PID_GetDutyCmd(&LeftDuty, &RightDuty);
                DCMotor_SetDuty(LeftDuty, RightDuty);
            }
        }

        /* 3) 串口命令处理 */
        if (SerialCmd_Process(CurrentYaw) != 0U) {
            continue;
        }

        /* 4) MaixCam 通信 */
        if (SerialMaixCam_Process() != 0U) {
            if ((Mode == SYS_TASK_RUN) &&
                (TaskId == 1U || TaskId == 2U) &&
                (TaskState == TASK_CAMERA)) {
                CameraCode = SerialMaixCam_GetResultCode(); //返回的值有效时会被 SerialMaixCam_Process 处理并存入 CameraCode，同时将 CameraReady 置位
                CameraReady = 1U;
            }
        }

        /* 5) 蓝牙状态上报(每 1s) */
        Report.taskId = TaskId;
        Report.currentYawDeg = CurrentYaw;
        Report.pidTrigIntervalMs = 20U;
        Report.pidHandleIntervalMs = 20U;
        Report.pidPendingCount = PidPending;
        Report.pidTriggerCount = SysMs / 20U;
        Report.pidOverwriteCount = 0U;
       // (void)SerialReport_Process(&ReportFlag, &Report);

        /* 6) OLED 刷新(每 100ms) */
        if (OledFlag != 0U) {
            OledFlag = 0U;

            switch (Menu_GetMode()) {
                case SYS_MENU:
                case SYS_MONITOR:
                case SYS_PATROL_INFO:
                case SYS_PID_EDIT:
                case SYS_FLASH_TEST:
                case SYS_GRAY_TEST:
                case SYS_CAMERA_TEST:
                case SYS_SPEED_LOOP_TEST:
                case SYS_PWM_TEST:
                case SYS_OLED_CN_TEST:
                case SYS_BUZZER_LED_TEST:
                    Menu_Render();
                    break;

                case SYS_TASK_RUN:
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
    static uint16_t Tick10  = 0U;
    static uint16_t Tick20  = 0U;
    static uint16_t Tick100 = 0U;
    static uint16_t Tick1000 = 0U;

    /* [1ms] 基础心跳 */
    SysMs++;
    Key_Tick();

    Tick20++;
    Tick100++;
    Tick1000++;
    Tick10++;

    /* [20ms] 控制事件触发 */
    if (Tick20 >= 20U) {
        Tick20 = 0U;

        if (PidPending < PID_PENDING_COUNT_MAX) {
            PidPending++;
        }
    }

    /* [100ms] OLED 刷新事件 */
    if (Tick100 >= 100U) {
        Tick100 = 0U;
        OledFlag = 1U;
    }

    /* [1000ms] 蓝牙状态上报事件 */
    if (Tick1000 >= 1000U) {
        Tick1000 = 0U;
        ReportFlag = 1U;
    }

    /* [10ms] IMU 更新与 Yaw 解算 */
    if (Tick10 >= 10U) {
        Tick10 = 0U;
        Get_Acc_ICM42688();
        Get_Gyro_ICM42688();
        {
            const float DegToRad = 0.01745329252f;
            float Gx = icm42688_gyro_x * DegToRad * GyroDirX;
            float Gy = icm42688_gyro_y * DegToRad * GyroDirY;
            float Gz = icm42688_gyro_z * DegToRad * GyroDirZ;
            MahonyAHRSupdateIMU(Gx, Gy, Gz, icm42688_acc_x, icm42688_acc_y, icm42688_acc_z);
            CurrentYaw = ICM42688_GetYawZeroedDeg();
            PID_UpdateYawFeedback(CurrentYaw);
            PID_UpdateGyroZFeedback(icm42688_gyro_z * GyroDirZ);
        }
    }

    /* [1ms] Buzzer/LED non-blocking service */
    BuzzerLed_Tick1ms();
}





