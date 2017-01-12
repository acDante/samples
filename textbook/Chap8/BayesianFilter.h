#ifndef _q_learning_h_
#define _q_learning_h_

#ifndef RAND_MAX
#define RAND_MAX 2147483000
#endif

// Maxの迷路のサイズ
#define FLOORSIZE 5
// 設定する迷路のサイズ
#define SIZE 5

// 行動の数
#define NUM_ACTIONS 4

// 状態遷移確率
#define TRANS 0.8
// 観測確率
#define KANSOKU 0.7

typedef struct { int col; int row; } STATE;
typedef enum { NORTH = 0, EAST, SOUTH, WEST } ACTION;

typedef enum { STOP = 0, INITIALIZE, INITPOSITION, ACTIONDECISION, PREPAREWALK, WALK, CALCULATION, NEXTSTEP } RobotState;

/* 5*5 */
int WALL[2 * SIZE + 1][SIZE + 1] = {
	{ -1, 1, 1, 1, 1, 1 },			// -1はダミー
	{ 1, 1, 0, 0, 1, 1 },			//  1は壁
	{ -1, 0, 1, 1, 0, 0 },			//  0は通れる
	{ 1, 0, 0, 0, 1, 1 },
	{ -1, 0, 1, 1, 0, 0 },
	{ 1, 1, 0, 1, 0, 1 },
	{ -1, 0, 1, 0, 1, 0 },
	{ 1, 0, 0, 1, 0, 1 },
	{ -1, 1, 0, 0, 0, 1 },
	{ 1, 0, 1, 1, 0, 1 },
	{ -1, 1, 1, 1, 1, 1 } };

/* 3*3 */
//int WALL[2 * SIZE + 1][SIZE + 1] = {
//	{ -1, 1, 1, 1 },			// -1はダミー
//	{  1, 1, 0, 1 },			//  1は壁
//	{ -1, 0, 0, 1 },			//  0は通れる
//	{  1, 0, 0, 1 },
//	{ -1, 0, 1, 0 },
//	{  1, 0, 1, 1 },
//	{ -1, 1, 1, 1 } };

#endif