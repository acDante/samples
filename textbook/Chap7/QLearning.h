#ifndef _q_learning_h_
#define _q_learning_h_

#ifndef RAND_MAX
#define RAND_MAX 2147483000
#endif

// Max�̖��H�̃T�C�Y
#define FLOORSIZE 5
// �ݒ肷����H�̃T�C�Y
#define SIZE 3

// �S�[���̈ʒu
#define GOAL_COL 2
#define GOAL_ROW 2

// �s���̐�
#define NUM_ACTIONS 4

// ��V
#define GOAL_REWARD 100.0		/* �S�[���ɂ��ǂ�����ꍇ */
#define HIT_WALL_PENALTY -10.0	/* �ǂɂԂ������ꍇ */
#define ONE_STEP_PENALTY -1.0	/* �ǂɂԂ��炸��1�}�X�i�񂾏ꍇ */

// e-greedy�@�̃p�����[�^
#define EPSILON 0.1
// ������
#define GAMMA 0.9
// �w�K��
#define BETA 0.1


typedef struct { int col; int row; } STATE;
typedef enum { NORTH = 0, EAST, SOUTH, WEST } ACTION;

double qvalues[SIZE][SIZE][NUM_ACTIONS];
int state_action_recency[SIZE][SIZE][NUM_ACTIONS];

typedef enum { STOP = 0, INITPOSITION, ACTIONDECISION, WALK, GETREWARD, NEXTSTEP } RobotState;

#endif