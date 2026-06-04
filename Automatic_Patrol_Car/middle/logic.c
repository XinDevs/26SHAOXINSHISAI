#include "logic.h"
#include "dc_motor.h"
#include "buzzer_led.h"
#include "delay.h"
#include "serial_maixcam.h"
#include "main.h"
#include <string.h>

/* ================= 底层适配函数 ================= */
/* 将 logic.h 声明的外部接口映射到项目已有的驱动函数 */

void Motor_Stop(void)          { DCMotor_Brake(); }
void Motor_Brake_And_Lock(void){ /* 由 Logic_Get_Turn_Direction 返回 -1 代替死循环 */ }
void Delay_ms(uint32_t ms)     { delay_ms(ms); }

void Buzzer_Beep(uint32_t ms)              { (void)ms; BuzzerLed_StartRedAlert(); }
void Buzzer_Beep_Intermittent(uint32_t ms) { (void)ms; BuzzerLed_StartGreenAlert(); }
void Buzzer_Off(void)                      { BuzzerLed_AllOff(); }
void LED_Red_On(void)                      { /* 已包含在 BuzzerLed_StartRedAlert 中 */ }
void LED_Green_On(void)                    { /* 已包含在 BuzzerLed_StartGreenAlert 中 */ }
void LED_Off(void)                         { BuzzerLed_AllOff(); }

static MaixCam_Data s_visionCache = {COLOR_NONE, SHAPE_NONE};

MaixCam_Data Get_Vision_Data(void) { return s_visionCache; }

void Execute_Physical_Turn(int dir)
{
    /* 非阻塞版本中不使用，物理转弯由 main.c 的 Turn_Start/Run/IsDone 执行 */
    (void)dir;
}

// ================= 1. 全局状态与地图阵列 =================
int sys_state = MODE_EXPLORE;
int last_node = NODE_O;
int curr_node = NODE_N1;
int found_count = 0;   // 已找到的标识数
int step = 0;          // 已走过的路口数
int target_N = 4;      // 目标寻找数量

// 边访问记录 (用于防死锁 DFS 决策)
int edge_visited[6][6] = {0};
// 标识是否已记录过，防止原地打转时重复记录
bool marker_recorded[256] = {false};

// 回O点最短真实距离表 (Dijkstra: O↔N1=1, N1↔N2=2, N1↔N3=2, N2↔N4=1, N4↔N3=1, N4↔N5=1, N2↔N5=4, N3↔N5=4)
const int dist_to_O[6] = {0, 1, 3, 3, 4, 5};

// 节点间真实物理距离权重矩阵，99代表不直接相连
const int edge_weight[6][6] = {
    /* O,  N1, N2, N3, N4, N5 */
    {  0,  1, 99, 99, 99, 99}, /* O  */
    {  1,  0,  2,  2, 99, 99}, /* N1 */
    { 99,  2,  0, 99,  1,  4}, /* N2 */
    { 99,  2, 99,  0,  1,  4}, /* N3 */
    { 99, 99,  1,  1,  0,  1}, /* N4 */
    { 99, 99,  4,  4,  1,  0}  /* N5 */
};

// 拓扑映射表：[当前节点][从哪来][左/右转] = 下一个节点
int map_next[6][6][2];               
// 视角映射表：[当前节点][从哪来] = 正前方对应的字母位置
LetterType map_forward_letter[6][6]; 

// ================= 2. 核心初始化函数 =================
void Init_Map(void) {
    memset(map_next, -1, sizeof(map_next));
    memset(map_forward_letter, LETTER_NONE, sizeof(map_forward_letter));

    // N1 节点
    map_next[NODE_N1][NODE_O][TURN_LEFT]  = NODE_N3; map_next[NODE_N1][NODE_O][TURN_RIGHT]  = NODE_N2;
    map_next[NODE_N1][NODE_N2][TURN_LEFT] = NODE_O;  map_next[NODE_N1][NODE_N2][TURN_RIGHT] = NODE_N3;
    map_next[NODE_N1][NODE_N3][TURN_LEFT] = NODE_N2; map_next[NODE_N1][NODE_N3][TURN_RIGHT] = NODE_O;
    map_forward_letter[NODE_N1][NODE_O]   = LETTER_A;  

    // N2 节点
    map_next[NODE_N2][NODE_N1][TURN_LEFT] = NODE_N4; map_next[NODE_N2][NODE_N1][TURN_RIGHT] = NODE_N5;
    map_next[NODE_N2][NODE_N4][TURN_LEFT] = NODE_N5; map_next[NODE_N2][NODE_N4][TURN_RIGHT] = NODE_N1;
    map_next[NODE_N2][NODE_N5][TURN_LEFT] = NODE_N1; map_next[NODE_N2][NODE_N5][TURN_RIGHT] = NODE_N4;
    map_forward_letter[NODE_N2][NODE_N1]  = LETTER_J; 
    map_forward_letter[NODE_N2][NODE_N4]  = LETTER_K; 
    map_forward_letter[NODE_N2][NODE_N5]  = LETTER_L; 

    // N3 节点
    map_next[NODE_N3][NODE_N1][TURN_LEFT] = NODE_N5; map_next[NODE_N3][NODE_N1][TURN_RIGHT] = NODE_N4;
    map_next[NODE_N3][NODE_N4][TURN_LEFT] = NODE_N1; map_next[NODE_N3][NODE_N4][TURN_RIGHT] = NODE_N5;
    map_next[NODE_N3][NODE_N5][TURN_LEFT] = NODE_N4; map_next[NODE_N3][NODE_N5][TURN_RIGHT] = NODE_N1;
    map_forward_letter[NODE_N3][NODE_N5]  = LETTER_B; 
    map_forward_letter[NODE_N3][NODE_N4]  = LETTER_C; 
    map_forward_letter[NODE_N3][NODE_N1]  = LETTER_D; 

    // N4 节点
    map_next[NODE_N4][NODE_N2][TURN_LEFT] = NODE_N3; map_next[NODE_N4][NODE_N2][TURN_RIGHT] = NODE_N5;
    map_next[NODE_N4][NODE_N3][TURN_LEFT] = NODE_N5; map_next[NODE_N4][NODE_N3][TURN_RIGHT] = NODE_N2;
    map_next[NODE_N4][NODE_N5][TURN_LEFT] = NODE_N2; map_next[NODE_N4][NODE_N5][TURN_RIGHT] = NODE_N3;
    map_forward_letter[NODE_N4][NODE_N3]  = LETTER_I; 
    map_forward_letter[NODE_N4][NODE_N2]  = LETTER_E; 
    map_forward_letter[NODE_N4][NODE_N5]  = LETTER_NONE; 

    // N5 节点
    map_next[NODE_N5][NODE_N2][TURN_LEFT] = NODE_N4; map_next[NODE_N5][NODE_N2][TURN_RIGHT] = NODE_N3;
    map_next[NODE_N5][NODE_N3][TURN_LEFT] = NODE_N2; map_next[NODE_N5][NODE_N3][TURN_RIGHT] = NODE_N4;
    map_next[NODE_N5][NODE_N4][TURN_LEFT] = NODE_N3; map_next[NODE_N5][NODE_N4][TURN_RIGHT] = NODE_N2;
    map_forward_letter[NODE_N5][NODE_N4]  = LETTER_G; 
    map_forward_letter[NODE_N5][NODE_N3]  = LETTER_H; 
    map_forward_letter[NODE_N5][NODE_N2]  = LETTER_F; 
}

// ================= 3. 路口事件中断/处理函数 =================
void On_Crossroad_Detected(void) 
{
    // 1. 底层控制：刹车停稳，准备拍照与决策
    Motor_Stop();
    Delay_ms(200); 
    step++;

    // 2. 终点判定：如果是在回家模式下回到了O点，彻底结束比赛！
    if (sys_state == MODE_GO_HOME && curr_node == NODE_O) {
        Motor_Brake_And_Lock(); 
        while(1); 
    }

    // 防超时保护：走太多步强制下班回家 
    if (step >= 25 && sys_state == MODE_EXPLORE) {
        sys_state = MODE_GO_HOME;
    }

    // 3. 视觉数据获取与映射分析
    MaixCam_Data vision = Get_Vision_Data(); 
    LetterType view_letter = map_forward_letter[curr_node][last_node];
    uint8_t m_color = vision.color;
    
    int turn_cmd = -1;
    int node_L = map_next[curr_node][last_node][TURN_LEFT];
    int node_R = map_next[curr_node][last_node][TURN_RIGHT];

    // ================== 状态机决策：探索模式 ==================
    if (sys_state == MODE_EXPLORE) 
    {
        // 记录标识得分
        if (m_color != COLOR_NONE && view_letter != LETTER_NONE) {
            if (!marker_recorded[view_letter]) {
                marker_recorded[view_letter] = true;
                found_count++;
            }
        }

        // =========================================================
        // ⭐ 核心策略修改：落袋为安的“保守回家”判断
        // =========================================================
        if (found_count >= target_N) {
            sys_state = MODE_GO_HOME; // 找齐了，完美收工
        } 
        else if (curr_node == NODE_N1 && last_node != NODE_O) {
            // 保守策略：只要不是刚从 O 点开出来的那一步，
            // 只要它绕图时碰巧回到了 N1(家门口)，哪怕没找齐，也强行宣布下班！
            sys_state = MODE_GO_HOME;
        } 
        else 
        {
            // 没到家门口，继续在迷宫里探索寻路
            if (m_color == COLOR_RED && edge_visited[curr_node][node_R] < 2) {
                turn_cmd = TURN_RIGHT;
                Buzzer_Beep(2000);   
                LED_Red_On();        
            } 
            else if (m_color == COLOR_GREEN && edge_visited[curr_node][node_L] < 2) {
                turn_cmd = TURN_LEFT;
                Buzzer_Beep_Intermittent(2000); 
                LED_Green_On();                 
            } 
            else {
                Buzzer_Off(); 
                LED_Off(); 
                
                // 自主 DFS 寻路
                int vL = edge_visited[curr_node][node_L];
                int vR = edge_visited[curr_node][node_R];

                if (vL < vR) {
                    turn_cmd = TURN_LEFT;
                } else if (vR < vL) {
                    turn_cmd = TURN_RIGHT;
                } else {
                    turn_cmd = TURN_RIGHT; // 默认靠右法则
                }
            }
        }
    } 
    
    // ================== 状态机决策：回家模式 ==================
    if (sys_state == MODE_GO_HOME) 
    {
        Buzzer_Off(); 
        LED_Off(); 
        
        // 查表直奔 O 点 (Dijkstra)
        if (curr_node == NODE_N1) {
            // ⭐ 物理转向修复：动态查表找准回 O 点的方向，防止撞墙
            if (map_next[NODE_N1][last_node][TURN_LEFT] == NODE_O) {
                turn_cmd = TURN_LEFT;
            } else {
                turn_cmd = TURN_RIGHT;
            }
            node_L = NODE_O; // 修正引导节点
        } else {
            // 对比左右分支哪个离家更近（加入边的代价）
            int cost_L = edge_weight[curr_node][node_L] + dist_to_O[node_L];
            int cost_R = edge_weight[curr_node][node_R] + dist_to_O[node_R];

            if (cost_L < cost_R) {
                turn_cmd = TURN_LEFT;
            } else {
                turn_cmd = TURN_RIGHT;
            }
        }
    }

    // ================== 执行与图论状态更新 ==================
    int next_node = map_next[curr_node][last_node][turn_cmd];
    
    // 如果在N1节点要回家，强制修正终点为O点
    if (sys_state == MODE_GO_HOME && curr_node == NODE_N1) {
        next_node = NODE_O; 
    }
    
    // 更新走过的路径记忆
    edge_visited[curr_node][next_node]++;
    last_node = curr_node;
    curr_node = next_node;

    // 交给底层控制组：执行原地物理转弯！
    Execute_Physical_Turn(turn_cmd);
}

// ================= 4. 非阻塞决策函数 (供 main.c 状态机调用) =================

/**
 * @brief  设置摄像头识别结果缓存 (main.c 在收到 MaixCam 数据后调用)
 */
void Logic_SetVisionCache(uint8_t resultCode)
{
    s_visionCache.color = COLOR_NONE;
    s_visionCache.shape = SHAPE_NONE;

    switch (resultCode) {
        case SERIAL_MAIXCAM_RESULT_RED_CIRCLE:
            s_visionCache.color = COLOR_RED;
            s_visionCache.shape = SHAPE_CIRCLE;
            break;
        case SERIAL_MAIXCAM_RESULT_RED_SQUARE:
            s_visionCache.color = COLOR_RED;
            s_visionCache.shape = SHAPE_SQUARE;
            break;
        case SERIAL_MAIXCAM_RESULT_GREEN_CIRCLE:
            s_visionCache.color = COLOR_GREEN;
            s_visionCache.shape = SHAPE_CIRCLE;
            break;
        case SERIAL_MAIXCAM_RESULT_GREEN_SQUARE:
            s_visionCache.color = COLOR_GREEN;
            s_visionCache.shape = SHAPE_SQUARE;
            break;
        default:
            break;
    }
}

/**
 * @brief  非阻塞路口决策：根据摄像头颜色 + 拓扑地图决定转向方向
 * @param  cameraColor 摄像头识别的颜色 (COLOR_RED / COLOR_GREEN / COLOR_NONE)
 * @retval TURN_LEFT(1) 或 TURN_RIGHT(0): 正常转向方向
 * @retval -1: 已到达终点(O点)，任务应结束
 * @note   此函数同时更新图论状态 (edge_visited, last_node, curr_node, step 等)
 */
int Logic_Get_Turn_Direction(uint8_t cameraColor)   //如果cameracolor
{
    step++;

    // 终点判定：回家模式下回到 O 点
    if (sys_state == MODE_GO_HOME && curr_node == NODE_O) {
        return -1;
    }

    // 防超时保护
    if (step >= 25 && sys_state == MODE_EXPLORE) {
        sys_state = MODE_GO_HOME;
    }

    LetterType view_letter = map_forward_letter[curr_node][last_node];
    uint8_t m_color = cameraColor;

    int turn_cmd = TURN_RIGHT; // 默认值
    int node_L = map_next[curr_node][last_node][TURN_LEFT];
    int node_R = map_next[curr_node][last_node][TURN_RIGHT];

    // ============ 探索模式决策 ============
    if (sys_state == MODE_EXPLORE)
    {
        // 保守回家判断
        if(found_count < target_N)
        {

            // 颜色引导 + DFS 寻路
            if (m_color == COLOR_RED && edge_visited[curr_node][node_R] < 5) {
                turn_cmd = TURN_RIGHT;
                Buzzer_Beep(2000);
                LED_Red_On();
            }
            else if (m_color == COLOR_GREEN && edge_visited[curr_node][node_L] < 5) {
                turn_cmd = TURN_LEFT;
                Buzzer_Beep_Intermittent(2000);
                LED_Green_On();
            }
            //以下是盲走逻辑：如果颜色信息无效或对应的方向已经走过两次以上，就启用 DFS 寻路决策，选择访问次数更少的方向。   
            else {
                Buzzer_Off();
                LED_Off();

                int vL = edge_visited[curr_node][node_L];
                int vR = edge_visited[curr_node][node_R];
                if (curr_node == NODE_N1 && last_node == NODE_N3) {
                    turn_cmd=TURN_LEFT;
                }
                else if (curr_node == NODE_N1 && last_node == NODE_N2) {
                    turn_cmd=TURN_RIGHT;
                }
                else if (vL < vR) {
                    turn_cmd = TURN_LEFT;
                } 
                else if (vR < vL) {
                    turn_cmd = TURN_RIGHT;
                } 
                else {
                    turn_cmd = TURN_RIGHT; // 默认靠右法则
                }
            }
        }
        else if (found_count >= target_N) {
            if (m_color == COLOR_RED && edge_visited[curr_node][node_R] < 2) {
                turn_cmd = TURN_RIGHT;
                Buzzer_Beep(2000);
                LED_Red_On();
            }
            else if (m_color == COLOR_GREEN && edge_visited[curr_node][node_L] < 2) {
                turn_cmd = TURN_LEFT;
                Buzzer_Beep_Intermittent(2000);
                LED_Green_On();
            }
            sys_state = MODE_GO_HOME;
        }
        // if (curr_node == NODE_N1 && last_node != NODE_O){
        //     sys_state = MODE_EXPLORE; 
        // }
        // 记录标识得分
        if (m_color != COLOR_NONE && view_letter != LETTER_NONE) {
            if (!marker_recorded[view_letter]) {
                marker_recorded[view_letter] = true;
                found_count++;
            }
        }
    }

    // ============ 回家模式决策 ============
    if (sys_state == MODE_GO_HOME)
    {
        // Buzzer_Off();
        // LED_Off();

        if (curr_node == NODE_N1) {
            //例如从N2来 特殊的点 其余的在else里
            if (map_next[NODE_N1][last_node][TURN_LEFT] == NODE_O) {
                turn_cmd = TURN_LEFT;
            } else {
                turn_cmd = TURN_RIGHT;
            }
        } else {
            // 对比左右分支哪个离家更近（加入边的代价）
            int cost_L = edge_weight[curr_node][node_L] + dist_to_O[node_L];
            int cost_R = edge_weight[curr_node][node_R] + dist_to_O[node_R];
            int vL = edge_visited[curr_node][node_L];
            int vR = edge_visited[curr_node][node_R];
            if (m_color == COLOR_RED && edge_visited[curr_node][node_R] < 5) {
                turn_cmd = TURN_RIGHT;
                Buzzer_Beep(2000);
                LED_Red_On();
            }
            else if (m_color == COLOR_GREEN && edge_visited[curr_node][node_L] < 5) {
                turn_cmd = TURN_LEFT;
                Buzzer_Beep_Intermittent(2000);
                LED_Green_On();
            }
            else if (vL < vR) {
                turn_cmd = TURN_LEFT;
            } 
            else if (vR < vL) {
                turn_cmd = TURN_RIGHT;
            } 
            else if (cost_L < cost_R) {
                turn_cmd = TURN_LEFT;
            } else {
                turn_cmd = TURN_RIGHT;
            }
        }
    }

    // ============ 图论状态更新 ============
    int next_node = map_next[curr_node][last_node][turn_cmd];

    if (sys_state == MODE_GO_HOME && curr_node == NODE_N1) {
        next_node = NODE_O;
    }

    edge_visited[curr_node][next_node]++;
    last_node = curr_node;
    curr_node = next_node;

    return turn_cmd;
}

/**
 * @brief  重置 logic 全局状态 (任务开始/重新开始时调用)
 */
void Logic_Reset(void)
{
    sys_state = MODE_EXPLORE;
    last_node = NODE_O;
    curr_node = NODE_N1;
    found_count = 0;
    step = 0;
    memset(edge_visited, 0, sizeof(edge_visited));
    memset(marker_recorded, 0, sizeof(marker_recorded));
    s_visionCache.color = COLOR_NONE;
    s_visionCache.shape = SHAPE_NONE;
}

/**
 * @brief  非阻塞路口决策2：回家优先策略
 * @details 与决策1不同，决策2直接以回家为目标，不依赖 target_N 切换状态。
 * 默认按最短路径(Dijkstra)回家，仅当颜色非常明确且对应方向未走过时才覆盖。
 * 在 N1 出家门口无色块时，默认右转出发。
 * @param  cameraColor 摄像头识别的颜色 (COLOR_RED / COLOR_GREEN / COLOR_NONE)
 * @retval TURN_LEFT(1) 或 TURN_RIGHT(0): 正常转向方向
 * @retval -1: 已到达终点(O点)，任务应结束
 */
int Logic_Get_Turn_Direction2(uint8_t cameraColor)
{
    step++;

    // 终点判定：回到 O 点则任务结束
    if (curr_node == NODE_O) {
        return -1;
    }

    LetterType view_letter = map_forward_letter[curr_node][last_node];
    uint8_t m_color = cameraColor;

    int turn_cmd = TURN_RIGHT; // 默认值
    int node_L = map_next[curr_node][last_node][TURN_LEFT];
    int node_R = map_next[curr_node][last_node][TURN_RIGHT];

    // 记录标识得分（无论什么模式都要记录）
    if (m_color != COLOR_NONE && view_letter != LETTER_NONE) {
        if (!marker_recorded[view_letter]) {
            marker_recorded[view_letter] = true;
            found_count++;
        }
    }

    // ============ 回家优先决策 ============
    // 核心思路：默认走离家更近的方向，颜色仅在"非常明确"时覆盖
    Buzzer_Off();
    LED_Off();

    // 第一优先级：颜色非常明确 → 跟着颜色走（但必须该方向未走过太多次）
    if (m_color == COLOR_RED && edge_visited[curr_node][node_R] < 3) {
        turn_cmd = TURN_RIGHT;
        Buzzer_Beep(2000);
        LED_Red_On();
    }
    else if (m_color == COLOR_GREEN && edge_visited[curr_node][node_L] < 3) {
        turn_cmd = TURN_LEFT;
        Buzzer_Beep_Intermittent(2000);
        LED_Green_On();
    }
    // 第二优先级：无色块或颜色不可信 → 按最短路径回家
    else {
        // N1 出家门口特殊处理：无色块时默认右转
        if (curr_node == NODE_N1 && last_node == NODE_O) {
            turn_cmd = TURN_RIGHT;
        }
        // N1 从其他方向回来，查表找 O 点方向
        else if (curr_node == NODE_N1) {
            if (map_next[NODE_N1][last_node][TURN_LEFT] == NODE_O) {
                turn_cmd = TURN_LEFT;
            } else {
                turn_cmd = TURN_RIGHT;
            }
        }
        // 其他节点：选离 O 点更近的方向（考虑物理距离）
        else {
            int cost_L = edge_weight[curr_node][node_L] + dist_to_O[node_L];
            int cost_R = edge_weight[curr_node][node_R] + dist_to_O[node_R];

            // if (cost_L < cost_R) {
            //     turn_cmd = TURN_LEFT;
            // } else if (cost_R < cost_L) {
            //     turn_cmd = TURN_RIGHT;
            // }
            // // 距离相等时，选走过次数少的方向（探索差异）
            // else {
            //     int vL = edge_visited[curr_node][node_L];
            //     int vR = edge_visited[curr_node][node_R];
            //     if (vL < vR) {
            //         turn_cmd = TURN_LEFT;
            //     } else {
            //         turn_cmd = TURN_RIGHT;
            //     }
            // }
            int vL = edge_visited[curr_node][node_L];
            int vR = edge_visited[curr_node][node_R];
            if (vL < vR) {
                turn_cmd = TURN_LEFT;
            } 
            else if(vL > vR)
            {
                turn_cmd = TURN_RIGHT;
            }
            else if (cost_L < cost_R) {
                turn_cmd = TURN_LEFT;
            } 
            else if (cost_R < cost_L) {
                turn_cmd = TURN_RIGHT;
            }
            else {
                turn_cmd = TURN_RIGHT;
            }
        }
    }

    // ============ 图论状态更新 ============
    int next_node = map_next[curr_node][last_node][turn_cmd];

    // N1 回 O 点修正
    if (curr_node == NODE_N1 && last_node != NODE_O) {
        if (dist_to_O[node_L] == 0) {
            next_node = NODE_O;
        } else if (dist_to_O[node_R] == 0) {
            next_node = NODE_O;
        }
    }

    edge_visited[curr_node][next_node]++;
    last_node = curr_node;
    curr_node = next_node;

    return turn_cmd;
}