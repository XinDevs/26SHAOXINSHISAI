/**
 * @file    patrol_info.c
 * @brief   巡检信息记录模块 — 实现文件
 * @details 实现巡检任务的识别结果记录、用时统计和Flash持久化存储功能。
 *          使用Little-Endian格式存储数据，支持数据校验和版本管理。
 *
 *          典型使用流程：
 *          1. 系统启动时调用 PatrolInfo_Init() 读取历史数据
 *          2. 巡检开始时调用 PatrolInfo_Start(nowMs)
 *          3. 识别到标志物时调用 PatrolInfo_RecordResult(code)
 *          4. 巡检结束时调用 PatrolInfo_Finish(nowMs) 保存数据
 *          5. 显示时调用 PatrolInfo_GetSnapshot() 获取快照
 */

#include "patrol_info.h"

#include "Flash.h"
#include "serial_maixcam.h"

#include <string.h>

/* ===== Flash存储配置 ===== */

/** @brief Flash存储起始地址（第60KB位置） */
#define PATROL_INFO_FLASH_ADDR      0x000F1000UL

/** @brief Magic Number，用于标识有效数据 ("PINF" 的ASCII值) */
#define PATROL_INFO_MAGIC           0x50494E46UL

/** @brief 数据格式版本号 */
#define PATROL_INFO_VERSION         1U

/** @brief 存储总大小（字节） */
#define PATROL_INFO_STORAGE_SIZE    18U

/** @brief 校验和在数据中的偏移量 */
#define PATROL_INFO_CHECKSUM_OFFSET 16U

/* ===== 内部状态变量 ===== */

/** @brief 巡检信息快照（运行时数据） */
static PatrolInfoSnapshot_t s_info;

/** @brief 巡检开始时间戳（毫秒） */
static uint32_t s_startMs;

/* ===== 内部辅助函数 ===== */

/**
 * @brief  重置运行时状态
 * @details 清零巡检信息结构体和开始时间戳。
 */
static void PatrolInfo_ResetRuntime(void)
{
    memset(&s_info, 0, sizeof(s_info));
    s_startMs = 0U;
}

/**
 * @brief  计算校验和
 * @param  data 数据缓冲区
 * @param  length 计算校验和的字节数
 * @retval 16位校验和（所有字节累加和）
 */
static uint16_t PatrolInfo_Checksum(const uint8_t *data, uint16_t length)
{
    uint16_t sum = 0U;
    uint16_t i;

    for (i = 0U; i < length; i++) {
        sum = (uint16_t)(sum + data[i]);
    }

    return sum;
}

/**
 * @brief  写入32位Little-Endian值
 * @param  data 目标缓冲区（至少4字节）
 * @param  value 要写入的32位值
 */
static void PatrolInfo_WriteLe32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    data[2] = (uint8_t)((value >> 16) & 0xFFU);
    data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

/**
 * @brief  读取32位Little-Endian值
 * @param  data 源缓冲区（至少4字节）
 * @retval 读取到的32位值
 */
static uint32_t PatrolInfo_ReadLe32(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

/**
 * @brief  保存巡检数据到Flash
 * @details 将当前巡检信息打包为18字节数据：
 *          [0-3]  Magic Number
 *          [4]    版本号
 *          [5]    结果数量
 *          [6-9]  用时（秒）
 *          [10-15] 识别结果（最多6个）
 *          [16-17] 校验和
 *          然后擦除Flash扇区并写入数据。
 */
static void PatrolInfo_SaveToFlash(void)
{
    uint8_t data[PATROL_INFO_STORAGE_SIZE];
    uint16_t checksum;
    uint8_t eraseRet;
    uint8_t writeRet;

    /* 初始化为0xFF（Flash擦除后的默认值） */
    memset(data, 0xFF, sizeof(data));

    /* 打包数据 */
    PatrolInfo_WriteLe32(&data[0], PATROL_INFO_MAGIC);
    data[4] = PATROL_INFO_VERSION;
    data[5] = s_info.count;
    PatrolInfo_WriteLe32(&data[6], s_info.elapsedSeconds);
    memcpy(&data[10], s_info.results, PATROL_INFO_MAX_RESULTS);

    /* 计算并写入校验和 */
    checksum = PatrolInfo_Checksum(data, PATROL_INFO_CHECKSUM_OFFSET);
    data[16] = (uint8_t)(checksum & 0xFFU);
    data[17] = (uint8_t)((checksum >> 8) & 0xFFU);

    /* 擦除Flash扇区 */
    eraseRet = Flash_EraseSector(PATROL_INFO_FLASH_ADDR);
    if (eraseRet == 0U || eraseRet == 2U) {
        return;  /* 擦除失败 */
    }

    /* 写入Flash */
    writeRet = Flash_WritePage(PATROL_INFO_FLASH_ADDR, data, sizeof(data));
    if (writeRet == 0U || writeRet == 2U) {
        return;  /* 写入失败 */
    }

    s_info.storageValid = 1U;
}

/* ===== 公共接口函数 ===== */

/**
 * @brief  初始化巡检信息模块
 * @details 从Flash读取已存储的巡检数据，依次校验：
 *          1. Flash读取是否成功
 *          2. Magic Number是否正确
 *          3. 版本号是否匹配
 *          4. 结果数量是否有效
 *          5. 结果码是否在合法范围
 *          6. 校验和是否正确
 *          若任一校验失败则清零初始化。
 */
void PatrolInfo_Init(void)
{
    uint8_t data[PATROL_INFO_STORAGE_SIZE];
    uint16_t checksum;
    uint16_t storedChecksum;
    uint8_t count;
    uint8_t i;

    PatrolInfo_ResetRuntime();

    /* 读取Flash数据 */
    if (Flash_ReadData(PATROL_INFO_FLASH_ADDR, data, sizeof(data)) == 0U) {
        return;
    }

    /* 校验Magic Number */
    if (PatrolInfo_ReadLe32(&data[0]) != PATROL_INFO_MAGIC) {
        return;
    }

    /* 校验版本号 */
    if (data[4] != PATROL_INFO_VERSION) {
        return;
    }

    /* 校验结果数量 */
    count = data[5];
    if (count > PATROL_INFO_MAX_RESULTS) {
        return;
    }

    /* 校验每个结果码是否合法 */
    for (i = 0U; i < count; i++) {
        if (data[10U + i] < SERIAL_MAIXCAM_RESULT_RED_CIRCLE ||
            data[10U + i] > SERIAL_MAIXCAM_RESULT_GREEN_SQUARE) {
            return;
        }
    }

    /* 校验和 */
    checksum = PatrolInfo_Checksum(data, PATROL_INFO_CHECKSUM_OFFSET);
    storedChecksum = (uint16_t)data[16] | ((uint16_t)data[17] << 8);
    if (checksum != storedChecksum) {
        return;
    }

    /* 所有校验通过，恢复历史数据 */
    s_info.count = count;
    s_info.elapsedSeconds = PatrolInfo_ReadLe32(&data[6]);
    memcpy(s_info.results, &data[10], PATROL_INFO_MAX_RESULTS);
    s_info.running = 0U;
    s_info.storageValid = 1U;
}

/**
 * @brief  开始一次新的巡检任务
 * @param  nowMs 当前时间戳（毫秒）
 */
void PatrolInfo_Start(uint32_t nowMs)
{
    PatrolInfo_ResetRuntime();
    s_startMs = nowMs;
    s_info.running = 1U;
}

/**
 * @brief  记录一次识别结果
 * @param  resultCode 识别结果代码
 * @details 仅在运行中、结果码合法、未满时记录。
 */
void PatrolInfo_RecordResult(uint8_t resultCode)
{
    /* 检查是否在运行中 */
    if (s_info.running == 0U) {
        return;
    }

    /* 校验结果码范围 */
    if (resultCode < SERIAL_MAIXCAM_RESULT_RED_CIRCLE ||
        resultCode > SERIAL_MAIXCAM_RESULT_GREEN_SQUARE) {
        return;
    }

    /* 检查是否已满 */
    if (s_info.count >= PATROL_INFO_MAX_RESULTS) {
        return;
    }

    /* 记录结果 */
    s_info.results[s_info.count] = resultCode;
    s_info.count++;
}

/**
 * @brief  结束巡检任务
 * @param  nowMs 当前时间戳（毫秒）
 * @details 计算用时（向上取整到秒），保存到Flash。
 */
void PatrolInfo_Finish(uint32_t nowMs)
{
    uint32_t elapsedMs;

    /* 检查是否在运行中 */
    if (s_info.running == 0U) {
        return;
    }

    /* 计算用时（毫秒转秒，向上取整） */
    elapsedMs = nowMs - s_startMs;
    s_info.elapsedSeconds = (elapsedMs + 999U) / 1000U;
    s_info.running = 0U;
    s_info.storageValid = 0U;

    /* 保存到Flash */
    PatrolInfo_SaveToFlash();
}

/**
 * @brief  获取当前巡检信息快照
 * @param  snapshot 输出参数
 */
void PatrolInfo_GetSnapshot(PatrolInfoSnapshot_t *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    *snapshot = s_info;
}
