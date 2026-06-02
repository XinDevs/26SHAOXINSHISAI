/**
 * @file    serial_maixcam.h
 * @brief   MaixCam 涓插彛閫氫俊妯″潡鎺ュ彛锛圲ART2锛?15200bps锛? * @details 鍗忚鏍煎紡涓?serial_cmd 涓€鑷达細@payload\r\n
 *          閫氳繃 Serial2(UART2) 涓?MaixCam 杩涜闈為樆濉炴敹鍙戙€?=================================================================================
 * MaixCam 瑙嗚璇嗗埆閫氫俊鍗忚鎬昏〃 (UART2)
 * =================================================================================
 *
 * ---------------------------------------------------------------------------------
 * >>> 鍙戦€佹寚浠よ〃 (STM32 -> MaixCam)
 * ---------------------------------------------------------------------------------
 * 鍩虹璋冪敤: SerialMaixCam_SendCommand("鎸囦护");  (搴曞眰鑷姩鎷兼帴 @ 鍜?\r\n)
 *
 * 銆愮郴缁熺姸鎬佹帶鍒躲€? * -> "start communication" : 杩涘叆閫氫俊妯″紡 (鍞ら啋鎽勫儚澶达紝浠庨粯璁ょ敾闈㈣繘鍏ョ瓑寰呮寚浠ょ姸鎬?
 * 閫傜敤鍦烘櫙: 绯荤粺鍒氫笂鐢点€佹垨鎸変笅灏忚溅鈥滀竴閿惎鍔ㄢ€濇寜閿悗璋冪敤銆? * * -> "exit"                : 閫€鍑洪€氫俊妯″紡 (璁╂憚鍍忓ご鎭㈠鍒板垵濮嬬殑 HOME 鐣岄潰)
 * 閫傜敤鍦烘櫙: 灏忚溅璺戝畬涓ゅ湀鍥炲埌鍦嗗績锛屽贰妫€浠诲姟褰诲簳缁撴潫鏃惰皟鐢ㄣ€? *
 * 銆愯瘑鍒樁娈靛垏鎹€? * -> "Period1"             : 鍒囨崲鍒伴樁娈?1 (瀵艰埅妯″紡锛氭壘鈥滆杩涚澶粹€濆拰鈥滈粦鑹插崄瀛楄矾鍙ｂ€?
 * 閫傜敤鍦烘櫙: 灏忚溅鍦ㄨ禌閬撲笂瀵昏抗琛岄┒鏃惰皟鐢ㄣ€? * * -> "Period2"             : 鍒囨崲鍒伴樁娈?2 (鍥惧舰妯″紡锛氭壘绾㈣摑缁裤€佸渾鏂逛笁瑙?
 * 閫傜敤鍦烘櫙: 灏忚溅鍒拌揪 A~G 浠绘剰涓€涓墦鍗＄偣骞跺仠绋冲悗璋冪敤銆? *
 * 銆愯Е鍙戣瘑鍒€? * -> "need look"           : 璇锋眰璇嗗埆缁撴灉 (瑙﹀彂鍗曟鎶撴媿锛屽垎鏋愬綋鍓嶇敾闈㈠苟杩斿洖缁撴灉)
 * 閫傜敤鍦烘櫙: 鍒囨崲瀹?Period 鍚庯紝鎴栧畾鏃跺櫒姣忛殧鍑犲崄姣鍛ㄦ湡璋冪敤銆? *
 * * ---------------------------------------------------------------------------------
 * <<< 鎺ユ敹鍗忚琛?(MaixCam -> STM32)
 * ---------------------------------------------------------------------------------
 * 鍩虹甯ф牸寮? @ + 鏈夋晥杞借嵎(Payload) + \r\n
 *
 * 銆愰樁娈?1: 瀵艰埅涓庡杩规ā寮?(Period1 杩斿洖鍊?銆? * -> 璇嗗埆鐩爣: 琛岃繘绠ご銆侀粦鑹插崄瀛楄矾鍙? * - 妫€娴嬪埌绠ご鏈濆墠       : @forward\r\n
 * - 妫€娴嬪埌绠ご鏈濆悗       : @backward\r\n
 * - 浠呮娴嬪埌鍗佸瓧(鏃犵澶? : @True\r\n
 * - 鐢婚潰涓棤鏈夋晥鐩爣     : @None\r\n
 *
 * 銆愰樁娈?2: 鐜鍥惧舰鎵撳崱妯″紡 (Period2 杩斿洖鍊?銆? * -> 璇嗗埆鐩爣: A~G 鍖哄煙鐨勮壊鍧?(绾?钃?缁? 鍦?鏂?涓夎)
 * - 妫€娴嬪埌鍥惧舰           : @{棰滆壊} {褰㈢姸}\r\n  (渚? @red rectangle\r\n)
 * - 鐢婚潰涓棤鏈夋晥鐩爣     : @None\r\n
 *
 * 銆愬紓甯哥姸鎬佽繑鍥炪€? * - 鏈缃樁娈电洿鎺ヨ姹?  : @error: no period set\r\n
 * ================================================================================= */
#ifndef SERIAL_MAIXCAM_H_
#define SERIAL_MAIXCAM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Current MaixCam frame:
 *   TX/RX: 0xFF, length, payload bytes, 0xFE
 *   STM32 -> MaixCam:
 *     0xFF 0x01 0x05 0xFE: start sending results
 *     0xFF 0x01 0x06 0xFE: stop sending results
 *   MaixCam -> STM32 result payload length is 0x01.
 */
#define SERIAL_MAIXCAM_CMD_START_REQUEST 0x05U
#define SERIAL_MAIXCAM_CMD_STOP_REQUEST  0x06U

#define SERIAL_MAIXCAM_RESULT_NONE          0x00U
#define SERIAL_MAIXCAM_RESULT_RED_CIRCLE    0x01U
#define SERIAL_MAIXCAM_RESULT_RED_SQUARE    0x02U
#define SERIAL_MAIXCAM_RESULT_GREEN_CIRCLE  0x03U
#define SERIAL_MAIXCAM_RESULT_GREEN_SQUARE  0x04U

/**
 * @brief  鍒濆鍖?MaixCam 涓插彛锛圲ART2锛? */
void SerialMaixCam_Init(void);

/**
 * @brief  澶勭悊鏉ヨ嚜 MaixCam 鐨勪竴甯ф暟鎹紙闈為樆濉烇紝搴斿湪涓诲惊鐜腑鍛ㄦ湡璋冪敤锛? * @return 1 琛ㄧず鏈夋柊鍛戒护鍒拌揪锛岃皟鐢ㄨ€呭彲闅忓悗閫氳繃 SerialMaixCam_GetCommand 鑾峰彇锛? *         0 琛ㄧず鏃犳柊鏁版嵁銆? */
uint8_t SerialMaixCam_Process(void);

/**
 * @brief  鑾峰彇鏈€杩戜竴娆¤В鏋愬埌鐨勫懡浠ゅ瓧绗︿覆锛堝幓鎺?@ 鍓嶇紑锛? * @return 鍛戒护瀛楃涓叉寚閽堬紝鎸囧悜妯″潡鍐呴儴闈欐€佺紦鍐诧紱鏃犲懡浠ゆ椂杩斿洖绌轰覆 ""銆? *         涓嬫 Process 璋冪敤鍚庡唴瀹逛細琚鐩栥€? */
const char *SerialMaixCam_GetCommand(void);

uint8_t SerialMaixCam_GetResultCode(void);

uint8_t SerialMaixCam_SendStartRequest(void);
uint8_t SerialMaixCam_SendStopRequest(void);

/**
 * @brief  鍚?MaixCam 鍙戦€佸懡浠わ紙闈為樆濉烇級锛岃嚜鍔ㄦ嫾鎺?@ 鍓嶇紑涓?\r\n 鍚庣紑
 * @param  cmd 鍛戒护瀛楃涓诧紙涓嶅惈 @ 鍜?\r\n锛? * @return 1 鍏ラ槦鎴愬姛锛? 鍙戦€佺紦鍐叉弧锛堜涪寮冿級
 */
uint8_t SerialMaixCam_SendCommand(const char *cmd);

/**
 * @brief  鍚?MaixCam 鍙戦€佹牸寮忓寲鍛戒护锛堥潪闃诲锛? * @note   鐢ㄦ硶鍚?printf锛岃嚜鍔ㄦ嫾鎺?@ 鍓嶇紑涓?\r\n 鍚庣紑
 * @return 1 鍏ラ槦鎴愬姛锛? 鍙戦€佺紦鍐叉弧锛堜涪寮冿級
 */
uint8_t SerialMaixCam_SendCommandf(const char *format, ...);

/**
 * @brief  鍚?MaixCam 鍙戦€佸師濮嬪瓧鑺傦紙闈為樆濉烇級锛岃皟鐢ㄨ€呰嚜琛屽鐞嗗崗璁牸寮? * @return 1 鍏ラ槦鎴愬姛锛? 鍙戦€佺紦鍐叉弧锛堜涪寮冿級
 */
uint8_t SerialMaixCam_SendRaw(const uint8_t *data, uint16_t length);

/**
 * @brief  鑾峰彇 UART2 鍙戦€佷涪鍖呰鏁? */
uint32_t SerialMaixCam_GetTxDropCount(void);

#ifdef __cplusplus
}
#endif

#endif

