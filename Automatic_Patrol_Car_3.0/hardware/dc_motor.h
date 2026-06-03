/**
 * @file    dc_motor.h
 * @brief   鍙岃矾鐩存祦鐢垫満PWM椹卞姩 鈥?澶存枃浠?
 * @details 澹版槑鐢垫満鍒濆鍖栥€佷娇鑳姐€佸崰绌烘瘮璁剧疆銆佸仠姝㈠強鐘舵€佹煡璇㈡帴鍙ｃ€?
 */
#ifndef ICODE_DC_MOTOR_H_
#define ICODE_DC_MOTOR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DCMOTOR_DIR_STOP = 0,
    DCMOTOR_DIR_FORWARD,
    DCMOTOR_DIR_BACKWARD
} DCMotor_Direction_t;

typedef struct {
    int16_t left_duty_percent;
    int16_t right_duty_percent;
    DCMotor_Direction_t left_dir;
    DCMotor_Direction_t right_dir;
    uint8_t enabled;
} DCMotor_Status_t;

/**
 * @brief  鐩存祦鐢垫満椹卞姩鍒濆鍖?
 */
void DCMotor_Init(void);
/**
 * @brief  浣胯兘鎴栧叧闂數鏈鸿緭鍑?
 * @param  enable 0: 鍏抽棴, 闈?: 浣胯兘
 */
void DCMotor_Enable(uint8_t enable);
/**
 * @brief  璁剧疆宸﹀彸鐢垫満鍗犵┖姣?
 * @param  left_duty_percent 宸﹁疆鍗犵┖姣?-100~100)
 * @param  right_duty_percent 鍙宠疆鍗犵┖姣?-100~100)
 */
void DCMotor_SetDuty(int16_t left_duty_percent, int16_t right_duty_percent);
/**
 * @brief  鍋滄鍙岀數鏈鸿緭鍑?
 */
void DCMotor_Stop(void);
/**
 * @brief  刹车双电机，短接 H 桥输出以减小惯性滑行。
 */
void DCMotor_Brake(void);
/**
 * @brief  鑾峰彇鐢垫満鐘舵€佸揩鐓?
 * @param  status 杈撳嚭鐘舵€佺粨鏋勪綋鎸囬拡
 */
void DCMotor_GetStatus(DCMotor_Status_t *status);
/**
 * @brief  灏嗘柟鍚戞灇涓捐浆涓哄瓧绗︿覆
 * @param  dir 鏂瑰悜鏋氫妇
 * @retval 鏂瑰悜瀛楃涓?FWD/BWD/STP)
 */
const char *DCMotor_DirectionString(DCMotor_Direction_t dir);

#ifdef __cplusplus
}
#endif

#endif /* ICODE_DC_MOTOR_H_ */

