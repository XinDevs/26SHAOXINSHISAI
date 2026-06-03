/**
 * @file    emm_v5_status.c
 * @brief   Emm_V5 运动状态查询与解析实现
 */
#include "Emm_V5_status.h"
#include "Emm_V5.h"

#define EMM_V5_STATUS_FRAME_MAX_LEN  (16U)
#define EMM_V5_CMD_READ_VEL          (0x35U)
#define EMM_V5_CMD_READ_FLAG         (0x3AU)
#define EMM_V5_CMD_READ_OFLAG        (0x3BU)
#define EMM_V5_MIN_CMD_GAP_MS        (10U)

/* 修复1：将掩码从 0x01U 改为 0x02U，匹配闭环步进“到位标志位” (Bit 1) */
#define EMM_V5_FLAG_IN_POS_MASK      (0x02U)
#define EMM_V5_OFLAG_BUSY_MASK       (0x04U)

typedef struct {
    uint8_t addr;
    uint8_t hasVel;
    uint8_t hasFlag;
    uint8_t hasOFlag;
    int16_t velRpm;
    uint8_t flagRaw;
    uint8_t oflagRaw;
} EmmV5StatusSnapshot_t;

static EmmV5StatusSnapshot_t s_snapshot = {0U, 0U, 0U, 0U, 0, 0U, 0U};
static uint8_t s_frameBuf[EMM_V5_STATUS_FRAME_MAX_LEN];
static uint8_t s_frameLen = 0U;
static uint32_t s_nextCmdAllowedMs = 0U;

static void EmmV5Status_ParseFrame(const uint8_t *frame, uint8_t len)
{
    uint8_t addr;
    uint8_t cmd;

    if ((frame == 0) || (len < 3U)) {
        return;
    }

    addr = frame[0];
    cmd = frame[1];

    /* 修复2：速度帧长必须为 6，修正数据位索引，并加入方向判断符号 */
    if ((cmd == EMM_V5_CMD_READ_VEL) && (len >= 6U)) {
        // frame[2] 是方向(0是正转，非0反转)，frame[3] 是高8位，frame[4] 是低8位
        uint16_t raw = ((uint16_t) frame[3] << 8) | (uint16_t) frame[4];
        s_snapshot.addr = addr;
        s_snapshot.velRpm = (frame[2] == 0U) ? (int16_t)raw : -(int16_t)raw;
        s_snapshot.hasVel = 1U;
    } 
    /* 标志帧长为 4 */
    else if ((cmd == EMM_V5_CMD_READ_FLAG) && (len >= 4U)) {
        s_snapshot.addr = addr;
        s_snapshot.flagRaw = frame[2];
        s_snapshot.hasFlag = 1U;
    }
    else if ((cmd == EMM_V5_CMD_READ_OFLAG) && (len >= 4U)) {
        s_snapshot.addr = addr;
        s_snapshot.oflagRaw = frame[2];
        s_snapshot.hasOFlag = 1U;
    }
    else {
        /* 忽略未关注的状态帧 */
    }
}

void EmmV5Status_Reset(void)
{
    s_snapshot.addr = 0U;
    s_snapshot.hasVel = 0U;
    s_snapshot.hasFlag = 0U;
    s_snapshot.hasOFlag = 0U;
    s_snapshot.velRpm = 0;
    s_snapshot.flagRaw = 0U;
    s_snapshot.oflagRaw = 0U;
    s_frameLen = 0U;
    s_nextCmdAllowedMs = 0U;
}

void EmmV5Status_OnUart1Byte(uint8_t byte)
{
    if (s_frameLen >= EMM_V5_STATUS_FRAME_MAX_LEN) {
        s_frameLen = 0U;
    }

    s_frameBuf[s_frameLen++] = byte;

    /* 修复3：加入预期帧长判断，防止数据位恰好是 0x6B 导致提前截断 */
    if (s_frameLen >= 2U) {
        uint8_t expectedLen = 0U;
        
        // 根据张大头协议提取对应指令的固定返回帧长
        if (s_frameBuf[1] == EMM_V5_CMD_READ_VEL) {
            expectedLen = 6U;
        } else if (s_frameBuf[1] == EMM_V5_CMD_READ_FLAG) {
            expectedLen = 4U;
        } else if (s_frameBuf[1] == EMM_V5_CMD_READ_OFLAG) {
            expectedLen = 4U;
        } else {
            // 如果不是我们关注的指令，或者错位了，只要遇到 0x6B 就抛弃并复位
            if (byte == 0x6BU) {
                s_frameLen = 0U;
            }
            return;
        }
        
        // 当达到预期长度时，再校验帧尾是否为 0x6B
        if ((expectedLen > 0U) && (s_frameLen == expectedLen)) {
            if (s_frameBuf[s_frameLen - 1U] == 0x6BU) {
                EmmV5Status_ParseFrame(s_frameBuf, s_frameLen);
            }
            s_frameLen = 0U; // 无论解析成功与否，完成一帧长度即清零
        }
    }
}

void EmmV5Status_RequestMotionState(uint8_t addr, uint32_t nowMs)
{
    /*
     * 根因记录：此前把 0x35(读速度) 与 0x3A(读状态) 连续发送，
     * 电机侧可能出现粘包或收发时序冲突，导致错误回包/错误指令。
     * 厂家要求：相邻两条命令间隔至少 10ms。
     * 处理策略：
     * 1) 每次只发送一条查询命令（速度/状态交替）。
     * 2) 增加全局发送节流，未到 10ms 不发送。
     */
    if (nowMs < s_nextCmdAllowedMs) {
        return;
    }

    /* Motion completion only needs the in-position flag; do not poll speed here. */
    Emm_V5_Read_Sys_Params(addr, S_FLAG);

    s_nextCmdAllowedMs = nowMs + EMM_V5_MIN_CMD_GAP_MS;
}

void EmmV5Status_RequestHomeState(uint8_t addr, uint32_t nowMs)
{
    if (nowMs < s_nextCmdAllowedMs) {
        return;
    }

    Emm_V5_Read_Sys_Params(addr, S_OFLAG);
    s_nextCmdAllowedMs = nowMs + EMM_V5_MIN_CMD_GAP_MS;
}

uint8_t EmmV5Status_IsMotionDone(uint8_t addr)
{
    if (s_snapshot.addr != addr) {
        return 0U;
    }

    /* 修复4：调换优先级。优先判断到位标志位，防止启动瞬间由于速度为0而误判为已完成 */
    if (s_snapshot.hasFlag != 0U) {
        // Bit 1 为 1 表示已到位
        return ((s_snapshot.flagRaw & EMM_V5_FLAG_IN_POS_MASK) != 0U) ? 1U : 0U;
    }

    return 0U;
}

uint8_t EmmV5Status_HasAnyState(uint8_t addr)
{
    if (s_snapshot.addr != addr) {
        return 0U;
    }
    return (uint8_t) ((s_snapshot.hasVel != 0U)
                      || (s_snapshot.hasFlag != 0U)
                      || (s_snapshot.hasOFlag != 0U));
}

int16_t EmmV5Status_GetLastVelRpm(uint8_t addr)
{
    if ((s_snapshot.addr != addr) || (s_snapshot.hasVel == 0U)) {
        return 0;
    }
    return s_snapshot.velRpm;
}

uint8_t EmmV5Status_GetLastFlagRaw(uint8_t addr)
{
    if ((s_snapshot.addr != addr) || (s_snapshot.hasFlag == 0U)) {
        return 0U;
    }
    return s_snapshot.flagRaw;
}

uint8_t EmmV5Status_IsHomeDone(uint8_t addr)
{
    if ((s_snapshot.addr != addr) || (s_snapshot.hasOFlag == 0U)) {
        return 0U;
    }

    /* Bit 2 is Org_SF: 1 means homing in progress (0x07), 0 means homing finished or idle (0x03). */
    return ((s_snapshot.oflagRaw & EMM_V5_OFLAG_BUSY_MASK) == 0U) ? 1U : 0U;
}

uint8_t EmmV5Status_GetLastHomeFlagRaw(uint8_t addr)
{
    if ((s_snapshot.addr != addr) || (s_snapshot.hasOFlag == 0U)) {
        return 0U;
    }
    return s_snapshot.oflagRaw;
}
