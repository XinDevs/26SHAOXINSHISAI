/**
 * @file    serial_cmd.c
 * @brief   串口命令解析与在线调参实现
 * @details 支持以下命令路径:
 *          1) 20参数帧: 灰度权重 + 灰度PID + 左右速度PID + Yaw PID
 *          2) 11参数帧: 灰度权重 + 灰度PID
 *          3) 字符命令: TS/LP/RP/ALL/RST/?
 *          协议格式统一为 @payload\r\n，底层收发通过 Serial 驱动完成。
 */

#include "serial_cmd.h"
#include "ti_msp_dl_config.h"
#include "Serial.h"
#include "pid.h"
#include "dc_motor.h"
#include "encoder.h"
#include <string.h>
#include <stdio.h>

/* ===== 模块内缓冲与解析临时变量 =====
 * 所有串口命令解析过程中用到的临时变量均放在本模块内，
 * 避免 main 维护大量命令解析状态。
 */
static char s_pidRspLine[96];
static char s_rxLocal[SERIAL_PACKET_SIZE];
static char s_cmd[8];

static float s_leftTargetNew;
static float s_rightTargetNew;
static float s_leftKpNew;
static float s_leftKiNew;
static float s_leftKdNew;
static float s_rightKpNew;
static float s_rightKiNew;
static float s_rightKdNew;
static float s_grayWeightsNew[2];
static float s_grayKpNew;
static float s_grayKiNew;
static float s_grayKdNew;
static float s_yawKpNew;
static float s_yawKiNew;
static float s_yawKdNew;

static float s_leftKpCur;
static float s_leftKiCur;
static float s_leftKdCur;
static float s_rightKpCur;
static float s_rightKiCur;
static float s_rightKdCur;
static float s_grayKpCur;
static float s_grayKiCur;
static float s_grayKdCur;
static float s_yawKpCur;
static float s_yawKiCur;
static float s_yawKdCur;

/**
 * @brief  串口命令处理入口
 * @param  currentYawDeg 当前航向角(deg)，供 RST 命令复位运行态使用
 * @return 1 表示命令已处理且调用者应跳过本轮剩余逻辑，0 表示继续主循环
 */
uint8_t SerialCmd_Process(float currentYawDeg)
{
	int parseCount;
	int16_t leftDutyCmdNow;
	int16_t rightDutyCmdNow;
	char *rxCmdLine;

	/* 无新帧时直接返回，主循环无需额外处理。 */
	if (Serial_RxFlag[SERIAL_UART0] == 0U) {
		return 0U;
	}

	/* 原子读取一帧数据到本地缓冲，避免 DMA/中断并发改写。 */
	__disable_irq();
	Serial_RxFlag[SERIAL_UART0] = 0U;
	(void)strncpy(s_rxLocal, Serial_RxPacket[SERIAL_UART0], SERIAL_PACKET_SIZE - 1U);
	s_rxLocal[SERIAL_PACKET_SIZE - 1U] = '\0';
	__enable_irq();

	rxCmdLine = s_rxLocal;
	/* 兼容 @payload 与 payload 两种输入，统一去掉可选前缀 @。 */
	if (rxCmdLine[0] == '@') {
		rxCmdLine++;
	}

	/* 先尝试按 14 浮点格式解析，匹配完整参数下发命令(2权重+3灰度PID+3左速PID+3右速PID+3YawPID)。 */
	parseCount = sscanf(rxCmdLine,
						"%f %f %f %f %f %f %f %f %f %f %f %f %f %f",
						&s_grayWeightsNew[0],
						&s_grayWeightsNew[1],
						&s_grayKpNew,
						&s_grayKiNew,
						&s_grayKdNew,
						&s_leftKpNew,
						&s_leftKiNew,
						&s_leftKdNew,
						&s_rightKpNew,
						&s_rightKiNew,
						&s_rightKdNew,
						&s_yawKpNew,
						&s_yawKiNew,
						&s_yawKdNew);

	if (parseCount == 14) {
		/* 14 参数命令: 全链路参数更新 + 全环复位。 */
		PID_SetGrayscaleWeights2(s_grayWeightsNew);
		PID_Grayscale_Init(s_grayKpNew, s_grayKiNew, s_grayKdNew);
		PID_SPEED_INIT(s_rightKpNew, s_rightKiNew, s_rightKdNew,
					   s_leftKpNew, s_leftKiNew, s_leftKdNew);
		PID_Yaw_Init(s_yawKpNew, s_yawKiNew, s_yawKdNew);

		PID_Reset(GRAYSCALE);
		PID_Reset(MOTOR_LEFT);
		PID_Reset(MOTOR_RIGHT);
		PID_Reset(YAW);

		PID_GetParameters(GRAYSCALE, &s_grayKpCur, &s_grayKiCur, &s_grayKdCur);
		PID_GetParameters(MOTOR_LEFT, &s_leftKpCur, &s_leftKiCur, &s_leftKdCur);
		PID_GetParameters(MOTOR_RIGHT, &s_rightKpCur, &s_rightKiCur, &s_rightKdCur);
		PID_GetParameters(YAW, &s_yawKpCur, &s_yawKiCur, &s_yawKdCur);

		(void)snprintf(s_pidRspLine,
					   sizeof(s_pidRspLine),
					   "@PID20:OK GL(%.2f,%.2f,%.2f) L(%.2f,%.2f,%.2f) R(%.2f,%.2f,%.2f) Y(%.2f,%.2f,%.2f)\r\n",
					   (float)s_grayKpCur,
					   (float)s_grayKiCur,
					   (float)s_grayKdCur,
					   (float)s_leftKpCur,
					   (float)s_leftKiCur,
					   (float)s_leftKdCur,
					   (float)s_rightKpCur,
					   (float)s_rightKiCur,
					   (float)s_rightKdCur,
					   (float)s_yawKpCur,
					   (float)s_yawKiCur,
					   (float)s_yawKdCur);
		(void)Serial0_SendStringTry(s_pidRspLine);
		return 1U;
	}

	if (parseCount == 5) {
		/* 5 参数命令: 仅更新灰度权重(2路)与灰度外环 PID。 */
		PID_SetGrayscaleWeights2(s_grayWeightsNew);
		PID_Grayscale_Init(s_grayKpNew, s_grayKiNew, s_grayKdNew);
		PID_Reset(GRAYSCALE);

		PID_GetParameters(GRAYSCALE, &s_grayKpCur, &s_grayKiCur, &s_grayKdCur);

		(void)snprintf(s_pidRspLine,
					   sizeof(s_pidRspLine),
					   "@GRAY:OK KP=%.3f KI=%.3f KD=%.3f\r\n",
					   (float)s_grayKpCur,
					   (float)s_grayKiCur,
					   (float)s_grayKdCur);
		(void)Serial0_SendStringTry(s_pidRspLine);
		return 1U;
	}

	/* 非纯参数帧则按字符串命令处理。 */
	parseCount = sscanf(rxCmdLine, "%7s", s_cmd);
	if (parseCount != 1) {
		return 0U;
	}

	if (strcmp(s_cmd, "TS") == 0) {
		/* TS: 直接设置左右目标速度并立即计算占空比。 */
		parseCount = sscanf(rxCmdLine, "%*s %f %f", &s_leftTargetNew, &s_rightTargetNew);
		if (parseCount == 2) {
			PID_EnableSpeedOverride(s_leftTargetNew, s_rightTargetNew);
			PID_ExecuteSpeedInnerLoop();
			PID_GetDutyCmd(&leftDutyCmdNow, &rightDutyCmdNow);
			DCMotor_SetDuty(leftDutyCmdNow, rightDutyCmdNow);
			PID_GetTargetSpeeds(&s_leftTargetNew, &s_rightTargetNew);
			(void)snprintf(s_pidRspLine,
						   sizeof(s_pidRspLine),
						   "@TS:OK L=%.3f R=%.3f\r\n",
						   (float)s_leftTargetNew,
						   (float)s_rightTargetNew);
			(void)Serial0_SendStringTry(s_pidRspLine);
		} else {
			(void)Serial0_SendStringTry("@TS:ERR fmt=TS leftTarget rightTarget\\r\\n");
		}
	} else if (strcmp(s_cmd, "LP") == 0) {
		/* LP: 更新左轮速度 PID 参数，右轮保持当前参数。 */
		parseCount = sscanf(rxCmdLine, "%*s %f %f %f", &s_leftKpNew, &s_leftKiNew, &s_leftKdNew);
		if (parseCount == 3) {
			PID_GetParameters(MOTOR_RIGHT, &s_rightKpCur, &s_rightKiCur, &s_rightKdCur);
			PID_SPEED_INIT(s_rightKpCur, s_rightKiCur, s_rightKdCur,
						   s_leftKpNew, s_leftKiNew, s_leftKdNew);
			PID_GetParameters(MOTOR_LEFT, &s_leftKpCur, &s_leftKiCur, &s_leftKdCur);
			(void)snprintf(s_pidRspLine,
						   sizeof(s_pidRspLine),
						   "@LP:OK (%.3f,%.3f,%.3f)\r\n",
						   (float)s_leftKpCur,
						   (float)s_leftKiCur,
						   (float)s_leftKdCur);
			(void)Serial0_SendStringTry(s_pidRspLine);
		} else {
			(void)Serial0_SendStringTry("@LP:ERR fmt=LP kp ki kd\\r\\n");
		}
	} else if (strcmp(s_cmd, "RP") == 0) {
		/* RP: 更新右轮速度 PID 参数，左轮保持当前参数。 */
		parseCount = sscanf(rxCmdLine, "%*s %f %f %f", &s_rightKpNew, &s_rightKiNew, &s_rightKdNew);
		if (parseCount == 3) {
			PID_GetParameters(MOTOR_LEFT, &s_leftKpCur, &s_leftKiCur, &s_leftKdCur);
			PID_SPEED_INIT(s_rightKpNew, s_rightKiNew, s_rightKdNew,
						   s_leftKpCur, s_leftKiCur, s_leftKdCur);
			PID_GetParameters(MOTOR_RIGHT, &s_rightKpCur, &s_rightKiCur, &s_rightKdCur);
			(void)snprintf(s_pidRspLine,
						   sizeof(s_pidRspLine),
						   "@RP:OK (%.3f,%.3f,%.3f)\r\n",
						   (float)s_rightKpCur,
						   (float)s_rightKiCur,
						   (float)s_rightKdCur);
			(void)Serial0_SendStringTry(s_pidRspLine);
		} else {
			(void)Serial0_SendStringTry("@RP:ERR fmt=RP kp ki kd\\r\\n");
		}
	} else if (strcmp(s_cmd, "ALL") == 0) {
		/* ALL: 同时更新左右速度PID与左右目标速度。 */
		parseCount = sscanf(rxCmdLine,
							"%*s %f %f %f %f %f %f %f %f",
							&s_leftTargetNew,
							&s_rightTargetNew,
							&s_leftKpNew,
							&s_leftKiNew,
							&s_leftKdNew,
							&s_rightKpNew,
							&s_rightKiNew,
							&s_rightKdNew);
		if (parseCount == 8) {
			PID_SPEED_INIT(s_rightKpNew, s_rightKiNew, s_rightKdNew,
						   s_leftKpNew, s_leftKiNew, s_leftKdNew);
			PID_EnableSpeedOverride(s_leftTargetNew, s_rightTargetNew);
			PID_ExecuteSpeedInnerLoop();
			PID_GetDutyCmd(&leftDutyCmdNow, &rightDutyCmdNow);
			DCMotor_SetDuty(leftDutyCmdNow, rightDutyCmdNow);
			(void)Serial0_SendStringTry("@ALL:OK\\r\\n");
		} else {
			(void)Serial0_SendStringTry("@ALL:ERR fmt=ALL lt rt lkp lki lkd rkp rki rkd\\r\\n");
		}
	} else if (strcmp(s_cmd, "RST") == 0) {
		/* RST: 清覆盖、清速度环历史并清空运行态占空比命令，同时清零里程计。 */
		PID_ClearSpeedOverride();
		PID_Reset(MOTOR_LEFT);
		PID_Reset(MOTOR_RIGHT);
		PID_ResetRuntimeState(currentYawDeg);
		encoder_reset_odometry();
		DCMotor_SetDuty(0, 0);
		(void)Serial0_SendStringTry("@RST:OK\\r\\n");
	} else if (strcmp(s_cmd, "?") == 0) {
		/* 帮助命令: 返回可用格式说明。 */
		(void)Serial0_SendStringTry("@CMD fmt1=@w0..w7 gkp gki gkd | fmt2=@w0..w7 gkp gki gkd lkp lki lkd rkp rki rkd ykp yki ykd\\r\\n");
	} else {
		/* 未知命令统一回错误提示。 */
		(void)Serial0_SendStringTry("@ERR unknown cmd\\r\\n");
	}

	return 0U;
}


