/**
 * @file    dc_motor.c
 * @brief   鍙岃矾鐩存祦鐢垫満PWM椹卞姩
 * @details 鍩轰簬MSPM0瀹氭椂鍣≒WM杈撳嚭锛屾帶鍒跺乏鍙宠疆鐩存祦鐢垫満鐨勫崰绌烘瘮涓庢柟鍚戙€?
 *          鏀寔姝ｈ浆/鍙嶈浆/鍋滄锛屽彲閰嶇疆宸﹀彸杞柟鍚戝弽鐩稿紑鍏充互閫傞厤瀹為檯鎺ョ嚎銆?
 */

#include "dc_motor.h"

#include "ti_msp_dl_config.h"

/*
 * 鏂瑰悜鍙嶇浉寮€鍏筹紙鎸夊疄闄呮帴绾胯皟鏁达級
 * 0: 姝ｅ父
 * 1: 璇ヤ晶鍓嶈繘/鍚庨€€閫昏緫浜掓崲
 */
#ifndef DCMOTOR_LEFT_REVERSE
#define DCMOTOR_LEFT_REVERSE   (1)
#endif

#ifndef DCMOTOR_RIGHT_REVERSE
#define DCMOTOR_RIGHT_REVERSE  (1)
#endif

static DCMotor_Status_t s_motor = {
    .left_duty_percent = 0,
    .right_duty_percent = 0,
    .left_dir = DCMOTOR_DIR_STOP,
    .right_dir = DCMOTOR_DIR_STOP,
    .enabled = 0
};

/**
 * @brief  鍗犵┖姣旈檺骞呭埌 [-100, 100]
 * @param  duty 杈撳叆鍗犵┖姣旂櫨鍒嗘瘮
 * @retval 闄愬箙鍚庣殑鍗犵┖姣?
 */
static int16_t DCMotor_ClampDuty(int16_t duty)
{
    if (duty > 100) return 100;
    if (duty < -100) return -100;
    return duty;
}

/**
 * @brief  鎸夊弽鐩搁厤缃槧灏勭數鏈烘柟鍚?
 * @param  dir 鍘熷鏂瑰悜
 * @param  reverse 鍙嶇浉寮€鍏?0:涓嶅弽鐩? 1:鍙嶇浉)
 * @retval 鏄犲皠鍚庣殑瀹為檯鏂瑰悜
 */
static DCMotor_Direction_t DCMotor_ApplyReverse(
    DCMotor_Direction_t dir, uint8_t reverse)
{
    if (!reverse) {
        return dir;
    }
    if (dir == DCMOTOR_DIR_FORWARD) {
        return DCMOTOR_DIR_BACKWARD;
    }
    if (dir == DCMOTOR_DIR_BACKWARD) {
        return DCMOTOR_DIR_FORWARD;
    }
    return DCMOTOR_DIR_STOP;
}

/**
 * @brief  璁剧疆宸︾數鏈烘柟鍚戝紩鑴?
 * @param  dir 鐩爣鏂瑰悜
 */
static void DCMotor_SetLeftDirection(DCMotor_Direction_t dir)
{
    DCMotor_Direction_t realDir = DCMotor_ApplyReverse(dir, DCMOTOR_LEFT_REVERSE);

    switch (realDir)
    {
        case DCMOTOR_DIR_FORWARD:
            DL_GPIO_setPins(DC_Motor_BIN1_PORT, DC_Motor_BIN1_PIN);
            DL_GPIO_clearPins(DC_Motor_BIN2_PORT, DC_Motor_BIN2_PIN);
            break;
        case DCMOTOR_DIR_BACKWARD:
            DL_GPIO_clearPins(DC_Motor_BIN1_PORT, DC_Motor_BIN1_PIN);
            DL_GPIO_setPins(DC_Motor_BIN2_PORT, DC_Motor_BIN2_PIN);
            break;
        case DCMOTOR_DIR_STOP:
        default:
            DL_GPIO_clearPins(DC_Motor_BIN1_PORT, DC_Motor_BIN1_PIN);
            DL_GPIO_clearPins(DC_Motor_BIN2_PORT, DC_Motor_BIN2_PIN);
            break;
    }
}

/**
 * @brief  璁剧疆鍙崇數鏈烘柟鍚戝紩鑴?
 * @param  dir 鐩爣鏂瑰悜
 */
static void DCMotor_SetRightDirection(DCMotor_Direction_t dir)
{
    DCMotor_Direction_t realDir = DCMotor_ApplyReverse(dir, DCMOTOR_RIGHT_REVERSE);

    switch (realDir)
    {
        case DCMOTOR_DIR_FORWARD:
            DL_GPIO_setPins(DC_Motor_AIN1_PORT, DC_Motor_AIN1_PIN);
            DL_GPIO_clearPins(DC_Motor_AIN2_PORT, DC_Motor_AIN2_PIN);
            break;
        case DCMOTOR_DIR_BACKWARD:
            DL_GPIO_clearPins(DC_Motor_AIN1_PORT, DC_Motor_AIN1_PIN);
            DL_GPIO_setPins(DC_Motor_AIN2_PORT, DC_Motor_AIN2_PIN);
            break;
        case DCMOTOR_DIR_STOP:
        default:
            DL_GPIO_clearPins(DC_Motor_AIN1_PORT, DC_Motor_AIN1_PIN);
            DL_GPIO_clearPins(DC_Motor_AIN2_PORT, DC_Motor_AIN2_PIN);
            break;
    }
}

/**
 * @brief  璁剧疆鎸囧畾 PWM 閫氶亾鍗犵┖姣旂粷瀵瑰€?
 * @param  ccIndex 閫氶亾绱㈠紩
 * @param  dutyAbs 鍗犵┖姣旂粷瀵瑰€?0~100)
 */
static void DCMotor_SetPwmAbs(DL_TIMER_CC_INDEX ccIndex, uint16_t dutyAbs)
{
    uint32_t load;
    uint32_t compare;

    if (dutyAbs > 100U) {
        dutyAbs = 100U;
    }

    load = DL_Timer_getLoadValue(DC_Motor_PWM_INST);
    compare = ((load + 1U) * dutyAbs) / 100U;
    if (compare > load) {
        compare = load;
    }

    DL_Timer_setCaptureCompareValue(DC_Motor_PWM_INST, compare, ccIndex);
}

/**
 * @brief  璁剧疆宸︾數鏈?PWM 鍗犵┖姣旂粷瀵瑰€?
 * @param  dutyAbs 鍗犵┖姣旂粷瀵瑰€?0~100)
 */
static void DCMotor_SetLeftPwmAbs(uint16_t dutyAbs)
{
    DCMotor_SetPwmAbs(GPIO_DC_Motor_PWM_C1_IDX, dutyAbs);
}

/**
 * @brief  璁剧疆鍙崇數鏈?PWM 鍗犵┖姣旂粷瀵瑰€?
 * @param  dutyAbs 鍗犵┖姣旂粷瀵瑰€?0~100)
 */
static void DCMotor_SetRightPwmAbs(uint16_t dutyAbs)
{
    DCMotor_SetPwmAbs(GPIO_DC_Motor_PWM_C0_IDX, dutyAbs);
}

/**
 * @brief  鐩存祦鐢垫満椹卞姩鍒濆鍖?
 * @note   鍒濆鍖?PWM 骞剁疆闆惰緭鍑恒€?
 */
void DCMotor_Init(void)
{
    /* 鐢?SysConfig 鐢熸垚鐨勭數鏈?PWM 鍒濆鍖?*/
    SYSCFG_DL_DC_Motor_PWM_init();

    /* 纭繚璁℃暟鍣ㄨ繍琛岋紝PWM杈撳嚭鐢熸晥 */
    DL_Timer_startCounter(DC_Motor_PWM_INST);

    s_motor.enabled = 1U;
    DCMotor_SetDuty(0, 0);
}

/**
 * @brief  浣胯兘鎴栧叧闂數鏈鸿緭鍑?
 * @param  enable 0: 鍏抽棴, 闈?: 浣胯兘
 */
void DCMotor_Enable(uint8_t enable)
{
    s_motor.enabled = (enable != 0U) ? 1U : 0U;

    if (!s_motor.enabled)
    {
        DCMotor_SetDuty(0, 0);
        DL_Timer_stopCounter(DC_Motor_PWM_INST);
    }
    else
    {
        DL_Timer_startCounter(DC_Motor_PWM_INST);
    }
}

/**
 * @brief  璁剧疆宸﹀彸鐢垫満鍗犵┖姣?
 * @param  left_duty_percent 宸﹁疆鍗犵┖姣?-100~100)
 * @param  right_duty_percent 鍙宠疆鍗犵┖姣?-100~100)
 * @note   姝ｅ€煎墠杩? 璐熷€煎悗閫€, 0 涓哄仠姝€?
 */
void DCMotor_SetDuty(int16_t left_duty_percent, int16_t right_duty_percent)
{
    left_duty_percent = DCMotor_ClampDuty(left_duty_percent);
    right_duty_percent = DCMotor_ClampDuty(right_duty_percent);

    s_motor.left_duty_percent = left_duty_percent;
    s_motor.right_duty_percent = right_duty_percent;

    if (!s_motor.enabled)
    {
        s_motor.left_dir = DCMOTOR_DIR_STOP;
        s_motor.right_dir = DCMOTOR_DIR_STOP;
        DCMotor_SetLeftDirection(DCMOTOR_DIR_STOP);
        DCMotor_SetRightDirection(DCMOTOR_DIR_STOP);
        DCMotor_SetLeftPwmAbs(0);
        DCMotor_SetRightPwmAbs(0);
        return;
    }

    if (left_duty_percent > 0)
    {
        s_motor.left_dir = DCMOTOR_DIR_FORWARD;
        DCMotor_SetLeftDirection(DCMOTOR_DIR_FORWARD);
        DCMotor_SetLeftPwmAbs((uint16_t)left_duty_percent);
    }
    else if (left_duty_percent < 0)
    {
        s_motor.left_dir = DCMOTOR_DIR_BACKWARD;
        DCMotor_SetLeftDirection(DCMOTOR_DIR_BACKWARD);
        DCMotor_SetLeftPwmAbs((uint16_t)(-left_duty_percent));
    }
    else
    {
        s_motor.left_dir = DCMOTOR_DIR_STOP;
        DCMotor_SetLeftDirection(DCMOTOR_DIR_STOP);
        DCMotor_SetLeftPwmAbs(0);
    }

    if (right_duty_percent > 0)
    {
        s_motor.right_dir = DCMOTOR_DIR_FORWARD;
        DCMotor_SetRightDirection(DCMOTOR_DIR_FORWARD);
        DCMotor_SetRightPwmAbs((uint16_t)right_duty_percent);
    }
    else if (right_duty_percent < 0)
    {
        s_motor.right_dir = DCMOTOR_DIR_BACKWARD;
        DCMotor_SetRightDirection(DCMOTOR_DIR_BACKWARD);
        DCMotor_SetRightPwmAbs((uint16_t)(-right_duty_percent));
    }
    else
    {
        s_motor.right_dir = DCMOTOR_DIR_STOP;
        DCMotor_SetRightDirection(DCMOTOR_DIR_STOP);
        DCMotor_SetRightPwmAbs(0);
    }
}

/**
 * @brief  鍋滄鍙岀數鏈鸿緭鍑?
 */
void DCMotor_Stop(void)
{
    DCMotor_SetDuty(0, 0);
}
/**
 * @brief  刹车双电机，短接 H 桥输出以减小惯性滑行
 */
void DCMotor_Brake(void)
{
    s_motor.left_duty_percent = 0;
    s_motor.right_duty_percent = 0;
    s_motor.left_dir = DCMOTOR_DIR_STOP;
    s_motor.right_dir = DCMOTOR_DIR_STOP;

    if (!s_motor.enabled) {
        DCMotor_SetDuty(0, 0);
        return;
    }

    DL_GPIO_setPins(DC_Motor_BIN1_PORT, DC_Motor_BIN1_PIN);
    DL_GPIO_setPins(DC_Motor_BIN2_PORT, DC_Motor_BIN2_PIN);
    DL_GPIO_setPins(DC_Motor_AIN1_PORT, DC_Motor_AIN1_PIN);
    DL_GPIO_setPins(DC_Motor_AIN2_PORT, DC_Motor_AIN2_PIN);
    DCMotor_SetLeftPwmAbs(100U);
    DCMotor_SetRightPwmAbs(100U);
}

/**
 * @brief  鑾峰彇鐢垫満鐘舵€佸揩鐓?
 * @param  status 杈撳嚭鐘舵€佺粨鏋勪綋鎸囬拡
 */
void DCMotor_GetStatus(DCMotor_Status_t *status)
{
    if (status == 0) {
        return;
    }
    *status = s_motor;
}

/**
 * @brief  灏嗘柟鍚戞灇涓捐浆涓哄瓧绗︿覆
 * @param  dir 鏂瑰悜鏋氫妇
 * @retval 鏂瑰悜瀛楃涓?FWD/BWD/STP)
 */
const char *DCMotor_DirectionString(DCMotor_Direction_t dir)
{
    switch (dir)
    {
        case DCMOTOR_DIR_FORWARD:  return "FWD";
        case DCMOTOR_DIR_BACKWARD: return "BWD";
        case DCMOTOR_DIR_STOP:
        default:                   return "STP";
    }
}


