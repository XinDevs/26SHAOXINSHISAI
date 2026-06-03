/**
 * @file    encoder.h
 * @brief   双轮编码器测速驱动 — 头文件
 * @details 提供左右轮线速度(m/s)、转速(RPM)查询接口，以及编码器物理参数宏定义。
 */
#ifndef ICODE_ENCODER_H_
#define ICODE_ENCODER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 编码器参数（按你的电机/减速箱实际参数修改） ===== */
#define ENCODER_PPR              (13.0f)     /* 电机轴每圈脉冲数（单沿） */
#define ENCODER_GEAR_RATIO       (20.0f)     /* 减速比 */
#define ENCODER_EDGE_FACTOR      (2.0f)      /* 单相上下沿计数 -> 2X */
#define ENCODER_WHEEL_DIAMETER_M (0.065f)    /* 轮径（m） */
#define ENCODER_WHEEL_TRACK_M    (0.150f)    /* 左右轮中心距（m），用于差速里程计 */
#define ENCODER_SAMPLE_PERIOD_S  (0.020f)    /* 统计窗口（s），当前是20ms */

typedef struct {
	float left_distance_m;    /* 左轮累计位移（m），前进为正 */
	float right_distance_m;   /* 右轮累计位移（m），前进为正 */
	float center_distance_m;  /* 车体中心累计位移（m），由左右轮均值得到 */
	float total_distance_m;   /* 累计路程（m），按 |中心增量| 叠加 */
	float x_m;                /* 里程计 x 位置（m） */
	float y_m;                /* 里程计 y 位置（m） */
	float heading_rad;        /* 里程计航向（rad） */
} EncoderOdometry_t;

/**
 * @brief 编码器驱动初始化
 */
void encoder_init(void);

/**
 * @brief 获取左轮线速度
 * @retval 左轮速度 (m/s)
 */
float encoder_get_left_speed_mps(void);
/**
 * @brief 获取右轮线速度
 * @retval 右轮速度 (m/s)
 */
float encoder_get_right_speed_mps(void);
/**
 * @brief 获取左轮转速
 * @retval 左轮转速 (RPM)
 */
float encoder_get_left_rpm(void);
/**
 * @brief 获取右轮转速
 * @retval 右轮转速 (RPM)
 */
float encoder_get_right_rpm(void);

/**
 * @brief 清零里程计累计量（位移与位姿）
 */
void encoder_reset_odometry(void);
/**
 * @brief 获取左轮累计位移
 * @retval 左轮累计位移 (m)
 */
float encoder_get_left_distance_m(void);
/**
 * @brief 获取右轮累计位移
 * @retval 右轮累计位移 (m)
 */
float encoder_get_right_distance_m(void);
/**
 * @brief 获取车体中心累计位移
 * @retval 车体中心累计位移 (m)
 */
float encoder_get_center_distance_m(void);
/**
 * @brief 获取累计路程
 * @retval 累计路程 (m)
 */
float encoder_get_total_distance_m(void);
/**
 * @brief 获取里程计完整快照
 * @param odomOut 输出快照指针
 */
void encoder_get_odometry_snapshot(EncoderOdometry_t *odomOut);

/**
 * @brief 查询速度是否有新采样
 * @retval 1: 有新数据
 * @retval 0: 无新数据
 */
uint8_t encoder_is_updated(void);
/**
 * @brief 清除速度更新标志
 */
void encoder_clear_update_flag(void);

#ifdef __cplusplus
}
#endif

#endif /* ICODE_ENCODER_H_ */
