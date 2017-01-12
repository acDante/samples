#ifndef _q_learning_h_
#define _q_learning_h_

#ifndef RAND_MAX
#define RAND_MAX 2147483000
#endif

// Max�̖��H�̃T�C�Y
#define FLOORSIZE 5
// �ݒ肷����H�̃T�C�Y
#define SIZE 5

// �s���̐�
#define NUM_ACTIONS 4

// ��ԑJ�ڊm��
#define TRANS 0.8
// �ϑ��m��
#define KANSOKU 0.7

typedef struct { int col; int row; } STATE;
typedef enum { NORTH = 0, EAST, SOUTH, WEST } ACTION;

typedef enum { STOP = 0, INITIALIZE, INITPOSITION, ACTIONDECISION, PREPAREWALK, WALK, CALCULATION, NEXTSTEP } RobotState;

/* 5*5 */
int WALL[2 * SIZE + 1][SIZE + 1] = {
	{ -1, 1, 1, 1, 1, 1 },			// -1�̓_�~�[
	{ 1, 1, 0, 0, 1, 1 },			//  1�͕�
	{ -1, 0, 1, 1, 0, 0 },			//  0�͒ʂ��
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
//	{ -1, 1, 1, 1 },			// -1�̓_�~�[
//	{  1, 1, 0, 1 },			//  1�͕�
//	{ -1, 0, 0, 1 },			//  0�͒ʂ��
//	{  1, 0, 0, 1 },
//	{ -1, 0, 1, 0 },
//	{  1, 0, 1, 1 },
//	{ -1, 1, 1, 1 } };

#endif