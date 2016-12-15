#ifndef _q_learning_h_
#define _q_learning_h_

#ifndef RAND_MAX
#define RAND_MAX 2147483000
#endif

// Maxの迷路のサイズ
#define FLOORSIZE 5
// 設定する迷路のサイズ
#define SIZE 3

// ゴールの位置
#define GOAL_COL 2
#define GOAL_ROW 2

// 行動の数
#define NUM_ACTIONS 4

// 報酬
#define GOAL_REWARD 100.0		/* ゴールにたどりついた場合 */
#define HIT_WALL_PENALTY -10.0	/* 壁にぶつかった場合 */
#define ONE_STEP_PENALTY -1.0	/* 壁にぶつからずに1マス進んだ場合 */

// e-greedy法のパラメータ
#define EPSILON 0.1
// 割引率
#define GAMMA 0.9
// 学習率
#define BETA 0.1


typedef struct { int col; int row; } STATE;
typedef enum { NORTH = 0, EAST, SOUTH, WEST } ACTION;

double qvalues[SIZE][SIZE][NUM_ACTIONS];
int state_action_recency[SIZE][SIZE][NUM_ACTIONS];

typedef enum { STOP = 0, INITPOSITION, ACTIONDECISION, WALK, GETREWARD, NEXTSTEP } RobotState;

#endif