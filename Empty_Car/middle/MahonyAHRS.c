/**
 * @file MahonyAHRS.c
 * @brief Mahony 互补滤波姿态估计算法 —— 实现文件
 * @details 基于 Madgwick 的 Mahony AHRS 开源实现，利用 6 轴 IMU（加速度计+
 *          陀螺仪）或 9 轴（+磁力计）数据，通过互补滤波融合计算四元数姿态，
 *          并输出 Roll / Pitch / Yaw 欧拉角。本项目用于 ICM-42688 姿态解算，
 *          为航向角(Yaw)闭环 PID 提供反馈。算法以可调参数 twoKp / twoKi
 *          控制比例与积分增益，调用频率需与 IMU 采样频率一致（本项目 10ms/100Hz）。
 * @note 原始作者 SOH Madgwick，2011；后经"哆啦a梦"魔改适配。
 */

//-------------------------------------------------------------------------------------------
// 头文件

#include "MahonyAHRS.h"


//#include "dsp/fast_math_functions.h"    
//#include "arm_math.h"
#include <math.h>

//-------------------------------------------------------------------------------------------
// 定义
// 这里都是可以调整的参数
//---------------------------------------***********************************
float twoKp;        // <--- 新增这行：定义 Kp 变量
float twoKi;		// 2 * 积分增益 (Ki)
float q0, q1, q2, q3;	// 传感器坐标系相对于辅助坐标系的四元数
float integralFBx, integralFBy, integralFBz;  // 由 Ki 缩放的积分误差项
float invSampleFreq;
float roll_mahony, pitch_mahony, yaw_mahony;
char anglesComputed;
//*-*-*-**-----------------------------------------------------------------------------
/**
 * @brief  快速平方根倒数
 * @param  x 输入值
 * @retval 1/sqrt(x) 的近似值
 */
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


#define twoKpDef	(2.0f * 0.50f)	// 2 * 比例增益	这个不用了
#define twoKiDef	(2.0f * 0.0f)	// 2 * 积分增益
/**
 * @brief  Mahony 姿态解算初始化
 * @param  sampleFrequency 采样频率(Hz)
 */
void Mahony_Init(float sampleFrequency)
{
	twoKp = 2.0f * 0.5f; // 给个默认值
	//twoKi = twoKiDef;	// 2 * 积分增益 (Ki) （原）
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	integralFBx = 0.0f;
	integralFBy = 0.0f;
	integralFBz = 0.0f;
	anglesComputed = 0;
	invSampleFreq = 1.0f / sampleFrequency;
}

/**
 * @brief  设置比例反馈系数 twoKp
 * @param  kp 比例反馈系数
 */
void Mahony_SetKp(float kp)
{
    twoKp = kp;
}

/**
 * @brief  快速平方根倒数(两次牛顿迭代版本)
 * @param  x 输入值
 * @retval 1/sqrt(x) 的近似值
 */
float Mahony_invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**
 * @brief  依据初始加速度/磁力计估计初始四元数
 * @param  ax 加速度 X
 * @param  ay 加速度 Y
 * @param  az 加速度 Z
 * @param  mx 磁场 X
 * @param  my 磁场 Y
 * @param  mz 磁场 Z
 */
void MahonyAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float init_yaw, init_pitch, init_roll;
    float cr2, cp2, cy2, sr2, sp2, sy2;
    float sin_roll, cos_roll, sin_pitch, cos_pitch;
    float magX, magY;

    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) 
    {
	    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
	    mx *= recipNorm;
	    my *= recipNorm;
	    mz *= recipNorm;
	}

    init_pitch = atan2f(-ax, az);
    init_roll = atan2f(ay, az);

    sin_roll  = sinf(init_roll);
    cos_roll  = cosf(init_roll);
    cos_pitch = cosf(init_pitch);
    sin_pitch = sinf(init_pitch);

    if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
    {
    	magX = mx * cos_pitch + my * sin_pitch * sin_roll + mz * sin_pitch * cos_roll;
    	magY = my * cos_roll - mz * sin_roll;
    	init_yaw  = atan2f(-magY, magX);
    }
    else
    {
    	init_yaw=0.0f;
    }

    cr2 = cosf(init_roll * 0.5f);
    cp2 = cosf(init_pitch * 0.5f);
    cy2 = cosf(init_yaw * 0.5f);
    sr2 = sinf(init_roll * 0.5f);
    sp2 = sinf(init_pitch * 0.5f);
    sy2 = sinf(init_yaw * 0.5f);

    q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    q1= sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    q3= cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

    // 归一化四元数
    recipNorm = Mahony_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
/**
 * @brief  Mahony 姿态更新(9轴)
 * @param  gx 陀螺仪 X
 * @param  gy 陀螺仪 Y
 * @param  gz 陀螺仪 Z
 * @param  ax 加速度 X
 * @param  ay 加速度 Y
 * @param  az 加速度 Z
 * @param  mx 磁场 X
 * @param  my 磁场 Y
 * @param  mz 磁场 Z
 */
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	// 将陀螺仪角速度从 度/秒 转换为 弧度/秒
//	gx *= 0.0174533f;
//	gy *= 0.0174533f;
//	gz *= 0.0174533f;

    // 若磁力计数据无效，则使用纯 IMU 算法（避免磁力计归一化时产生 NaN）
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

	// 仅在加速度计测量有效时计算反馈
	//（避免加速度计归一化时产生 NaN）
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// 归一化加速度计测量值
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// 归一化磁力计测量值
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// 辅助变量，避免重复算术运算
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// 地球磁场的参考方向
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrtf(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// 重力和磁场的估计方向
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// 误差为估计方向与测量方向的叉积之和
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// 若启用积分反馈，则计算并应用
		if(twoKi > 0.0f) {
			// 由 Ki 缩放的积分误差
			integralFBx += twoKi * halfex * invSampleFreq;
			integralFBy += twoKi * halfey * invSampleFreq;
			integralFBz += twoKi * halfez * invSampleFreq;
			gx += integralFBx;	// 应用积分反馈
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;	// 防止积分饱和
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// 应用比例反馈
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// 积分四元数变化率
	gx *= (0.5f * invSampleFreq);		// 预乘公因子
	gy *= (0.5f * invSampleFreq);
	gz *= (0.5f * invSampleFreq);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// 归一化四元数
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}
//---------------------------------------------------------------------------------------------------
/**
 * @brief  Mahony 姿态更新(6轴, 无磁力计)
 * @param  gx 陀螺仪 X
 * @param  gy 陀螺仪 Y
 * @param  gz 陀螺仪 Z
 * @param  ax 加速度 X
 * @param  ay 加速度 Y
 * @param  az 加速度 Z
 */
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 仅在加速度计测量有效时计算反馈（避免加速度计归一化时产生 NaN）
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 归一化加速度计测量值
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 重力估计方向及垂直于磁通量的向量
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 误差为估计与测量的重力方向的叉积
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 若启用积分反馈，则计算并应用
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex  * invSampleFreq;	// 由 Ki 缩放的积分误差
            integralFBy += twoKi * halfey  * invSampleFreq;
            integralFBz += twoKi * halfez  * invSampleFreq;
            gx += integralFBx;	// 应用积分反馈
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	// 防止积分饱和
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 应用比例反馈
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
    }

    // 积分四元数变化率
    gx *= (0.5f *   invSampleFreq);		// 预乘公因子
    gy *= (0.5f  * invSampleFreq);
    gz *= (0.5f  * invSampleFreq);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
	anglesComputed = 0;
}

/**
 * @brief  由当前四元数计算欧拉角(角度制)
 */
void Mahony_computeAngles()
{
    // 使用标准 math.h 库
    roll_mahony  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578f;
    pitch_mahony = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;
    yaw_mahony   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 57.29578f;
	anglesComputed = 1;
}
/**
 * @brief  获取 Roll 角(度)
 * @retval Roll 角
 */
float getRoll() {
	if (!anglesComputed) Mahony_computeAngles();
	return roll_mahony;
}
/**
 * @brief  获取 Pitch 角(度)
 * @retval Pitch 角
 */
float getPitch() {
	if (!anglesComputed) Mahony_computeAngles();
	return pitch_mahony;
}
/**
 * @brief  获取 Yaw 角(度)
 * @retval Yaw 角
 */
float getYaw() {
	if (!anglesComputed) Mahony_computeAngles();
	return yaw_mahony;
}
//============================================================================================
// 代码结束
//============================================================================================
