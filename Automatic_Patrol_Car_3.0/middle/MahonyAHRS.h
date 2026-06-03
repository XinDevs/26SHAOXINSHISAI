/**
 * @file MahonyAHRS.h
 * @brief Mahony 互补滤波姿态估计算法 —— 头文件
 * @details 基于 Madgwick 的 Mahony AHRS 开源实现，利用 6 轴 IMU（加速度计+
 *          陀螺仪）或 9 轴（+磁力计）数据，通过互补滤波融合计算四元数姿态，
 *          并输出 Roll / Pitch / Yaw 欧拉角。本项目用于 ICM-42688 姿态解算，
 *          为航向角(Yaw)闭环 PID 提供反馈。
 * @note 原始作者 SOH Madgwick，2011；后经"哆啦a梦"魔改适配。
 *       http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 */
#ifndef MahonyAHRS_h
#define MahonyAHRS_h
/**
 * @brief  Mahony 姿态更新(9轴)
 */
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
/**
 * @brief  Mahony 姿态解算初始化
 */
void Mahony_Init(float sampleFrequency);
/**
 * @brief  Mahony 姿态更新(6轴, 无磁力计)
 */
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
/**
 * @brief  由四元数计算欧拉角(角度制)
 */
void Mahony_computeAngles(void);
/**
 * @brief  依据初始加速度/磁力计估计初始四元数
 */
void MahonyAHRSinit(float ax, float ay, float az, float mx, float my, float mz);
/**
 * @brief  获取 Roll 角(度)
 */
float getRoll(void);
/**
 * @brief  获取 Pitch 角(度)
 */
float getPitch(void);
/**
 * @brief  获取 Yaw 角(度)
 */
float getYaw(void);
/**
 * @brief  获取 Roll 角(弧度)
 */
float getRollRadians(void);
/**
 * @brief  获取 Pitch 角(弧度)
 */
float getPitchRadians(void);
/**
 * @brief  获取 Yaw 角(弧度)
 */
float getYawRadians(void);
/**
 * @brief  设置比例反馈系数
 */
void Mahony_SetKp(float kp);

extern float roll_mahony, pitch_mahony, yaw_mahony;
#endif
