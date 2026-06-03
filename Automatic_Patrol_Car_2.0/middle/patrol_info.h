/**
 * @file    patrol_info.h
 * @brief   巡检信息记录模块 — 头文件
 * @details 提供巡检任务的信息记录功能，包括识别结果记录、用时统计和
 *          Flash持久化存储。用于记录小车在巡检过程中识别到的各类标志物
 *          （红圆、蓝圆、红方、绿方等），并在OLED上显示巡检报告。
 *
 *          数据存储格式（18字节）：
 *          [0-3]  Magic Number (0x50494E46 = "PINF")
 *          [4]    版本号
 *          [5]    结果数量
 *          [6-9]  用时（秒）
 *          [10-15] 识别结果数组（最多6个）
 *          [16-17] 校验和
 */

#ifndef PATROL_INFO_H_
#define PATROL_INFO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 最大记录结果数量 */
#define PATROL_INFO_MAX_RESULTS 10U

/**
 * @brief  巡检信息快照结构体
 * @note   用于保存当前巡检状态的副本，供外部读取
 */
typedef struct {
    uint8_t results[PATROL_INFO_MAX_RESULTS];  /**< 识别结果数组，存储识别码 */
    uint8_t count;                              /**< 已记录的结果数量 */
    uint32_t elapsedSeconds;                    /**< 巡检用时（秒） */
    uint8_t running;                            /**< 是否正在运行中 (0:停止, 1:运行) */
    uint8_t storageValid;                       /**< 存储是否有效 (0:无效, 1:有效) */
} PatrolInfoSnapshot_t;

/**
 * @brief  初始化巡检信息模块
 * @details 从Flash读取已存储的巡检数据，校验Magic Number、版本号和校验和。
 *          若数据有效则恢复历史记录，否则清零初始化。
 * @note   应在系统启动时调用
 */
void PatrolInfo_Init(void);

/**
 * @brief  开始一次新的巡检任务
 * @param  nowMs 当前时间戳（毫秒），通常来自系统tick
 * @details 重置运行时状态，记录起始时间，标记为运行中。
 */
void PatrolInfo_Start(uint32_t nowMs);

/**
 * @brief  记录一次识别结果
 * @param  resultCode 识别结果代码，取值范围见 serial_maixcam.h:
 *         - SERIAL_MAIXCAM_RESULT_RED_CIRCLE   (红圆)
 *         - SERIAL_MAIXCAM_RESULT_BLUE_CIRCLE  (蓝圆)
 *         - SERIAL_MAIXCAM_RESULT_RED_SQUARE   (红方)
 *         - SERIAL_MAIXCAM_RESULT_GREEN_SQUARE (绿方)
 * @details 将识别结果追加到结果数组，最多记录6个。
 *          若未在运行中或结果码无效则忽略。
 */
void PatrolInfo_RecordResult(uint8_t resultCode);

/**
 * @brief  结束巡检任务
 * @param  nowMs 当前时间戳（毫秒）
 * @details 计算总用时，标记为停止，将数据保存到Flash。
 *          若未在运行中则忽略。
 */
void PatrolInfo_Finish(uint32_t nowMs);

/**
 * @brief  取消当前巡检任务
 * @details 手动退出时调用，不保存本次数据，也不覆盖上一次成功记录。
 */
void PatrolInfo_Cancel(void);

/**
 * @brief  获取当前巡检信息快照
 * @param  snapshot 输出参数，指向 PatrolInfoSnapshot_t 结构体
 * @details 将当前巡检状态复制到传入的结构体中，供外部读取显示。
 *          传入NULL指针时直接返回。
 */
void PatrolInfo_GetSnapshot(PatrolInfoSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* PATROL_INFO_H_ */
