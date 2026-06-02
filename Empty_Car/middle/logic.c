#include "logic.h"
#include <string.h>

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

// 回O点最短步数表 (预先算好的 Dijkstra 距离跳数)
const int dist_to_O[6] = {0, 1, 2, 2, 3, 3};

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
                EEPROM_Save_Marker(found_count, view_letter, m_color, vision.shape);
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
            // 对比左右分支哪个离家更近
            if (dist_to_O[node_L] < dist_to_O[node_R]) {
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