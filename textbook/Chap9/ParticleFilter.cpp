#include <sigverse/commonlib/ControllerEvent.h>
#include <sigverse/commonlib/Controller.h>
#include <sigverse/commonlib/Logger.h>
#include <windows.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

#include "ParticleFilter.h"

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class RobotController : public Controller 
{
public:
	void	 onInit(InitEvent &evt);
	double	 onAction(ActionEvent&);
	void	 onRecvMsg(RecvMsgEvent &evt);
	void	 onCollision(CollisionEvent &evt);

private:
	ViewService *m_view;
	RobotObj *robot;
	Vector3d roboPos;

	bool Collision;	// �Փ˔���
	bool Action;	// �s�����Ă������ǂ���

	RobotState ROBOT;
	ACTION act;

	void initPRTCL(double particle[SIZE][SIZE]);	// ���q�̏�����
	void normalize_weights();	// ���q�̏d�݂̐��K��
	std::vector<int> get_wall_status(STATE pos);
	void sampling(ACTION act);	// �T���v�����O
	void calcSENSOR(std::vector<int> wall);	// �Z���T���̊ϑ��m��
	void update_weights();	// ���q�̏d�ݕt��
	void resampling();
	void print(double particle[SIZE][SIZE]);	// �\��

	double PRTCL[SIZE][SIZE];	// ���q���i�[����Ă���z��
	double PRTCL_weight[SIZE][SIZE];	// �d�ݕt�����ꂽ���q���i�[����Ă���z��
	double SENSOR[SIZE][SIZE];	// �Z���T���̊ϑ��m�����i�[����Ă���z��

	int walkstep;
	int	step;

};

/*
 * onInit
 */
void RobotController::onInit(InitEvent &evt)
{
	m_view = (ViewService*)connectToService("SIGViewer");
	robot = getRobotObj(myname());

	Collision = false;
	Action = false;

}


/*
 * onAction
 */
double RobotController::onAction(ActionEvent &evt) 
{
	static STATE pos;
	static std::vector<int> walls;

	switch (ROBOT)
	{
	case INITIALIZE:
		walkstep = 0;
		step = 0;

		Collision = false;
		Action = false;

		initPRTCL(PRTCL);

		ROBOT = POSITION;
		break;
	case POSITION:
		// ���݈ʒu�̕ǂ̏����擾
		robot->getPosition(roboPos);
		pos.row = roboPos.z() / 100;
		pos.col = roboPos.x() / 100;
		walls = get_wall_status(pos);
		for (int i = 0; i < walls.size(); i++)
			std::cout << walls[i] << ", ";
		std::cout << std::endl;

		ROBOT = CALCULATION;
		break;
	case CALCULATION:
		/*
		 * step = 0 �F���q�̕��z��������(�ϓ��ɕ��z����)
		 *   -> ���{�b�g�͐i�ޕ��p�̎w����҂�
		 */
		if (step == 0) {
			for (int r = 0; r < SIZE; r++)
				for (int c = 0; c < SIZE; c++) 
					PRTCL[r][c] = PA / (SIZE * SIZE);
			step++;

			ROBOT = ACTIONDECISION;
			break;
		}
		std::cout << "�s���O�̗��q" << std::endl;
		print(PRTCL);

		// 1) ���q���ƂɎ���Ԃ���ԑJ�ڊm����p���ăT���v�����O
		sampling(act);
		std::cout << "���q�̃T���v�����O" << std::endl;
		print(PRTCL);

		// 2) �Z���T���̊ϑ��m�����v�Z
		calcSENSOR(walls);
		std::cout << "�Z���T���̊ϑ��m��" << std::endl;
		print(SENSOR);

		// 3) ���q�̏d�ݕt��
		update_weights();
		std::cout << "���q�̏d�ݕt��" << std::endl;
		print(PRTCL_weight);

		// 4) ���T���v�����O
		resampling();
		std::cout << "���T���v�����O" << std::endl;
		print(PRTCL);

		ROBOT = NEXTSTEP;
		break;
	case NEXTSTEP:
		step++;

		ROBOT = ACTIONDECISION;
		break;
	case ACTIONDECISION:
		walkstep = 0;
		if (Action == true) {
			if (act == 0)
				std::cout << "[��:0] ";
			else if (act == 1)
				std::cout << "[��:1] ";
			else if (act == 2)
				std::cout << "[��:2] ";
			else
				std::cout << "[��:3] ";
			ROBOT = PREPAREWALK;
			Action = false;
		}
		break;
	case PREPAREWALK:
		// �I�����������ɕǂ������WALK�̏������΂�
		if (walls[act] == 1) {
			std::cout << "�ǂ������ē����܂���" << std::endl;
			ROBOT = STOP;
			break;
		}

		ROBOT = WALK;
		break;
	case WALK:
	{
		Vector3d pos;
		walkstep++;
		if (walkstep < 11) {
			switch (act)
			{
			case WEST:
				pos.x(roboPos.x() - walkstep * 10);
				pos.y(roboPos.y());
				pos.z(roboPos.z());

				robot->setPosition(pos);

				break;
			case EAST:
				pos.x(roboPos.x() + walkstep * 10);
				pos.y(roboPos.y());
				pos.z(roboPos.z());

				robot->setPosition(pos);

				break;
			case NORTH:
				pos.x(roboPos.x());
				pos.y(roboPos.y());
				pos.z(roboPos.z() - walkstep * 10);

				robot->setPosition(pos);

				break;
			case SOUTH:
				pos.x(roboPos.x());
				pos.y(roboPos.y());
				pos.z(roboPos.z() + walkstep * 10);

				robot->setPosition(pos);

				break;
			}
			ROBOT = WALK;
		}
		else ROBOT = POSITION;
		break;
	}
	case STOP:
		Collision = false;
		// std::cout << "---------- STOP ----------" << std::endl;
		break;
	}
	return 0.1;
}

/*
 * onRecvMsg
 */
void RobotController::onRecvMsg(RecvMsgEvent &evt) 
{
	std::string sender = evt.getSender();
	std::string msg = evt.getMsg();

	std::stringstream ss;
	ss << msg;
	std::string header, body;
	ss >> header >> body;

	std::cout << "[ROBOT MSG]: " << msg << std::endl;

	if (msg == "ParticleFilter")
		ROBOT = INITIALIZE;
	if (msg == "initial")
		sendMsg("moderator_0", "initial");

	if (msg == "n") {
		act = static_cast<ACTION>(0);
		Action = true;
	}
	else if (msg == "e") {
		act = static_cast<ACTION>(1);
		Action = true;
	}
	else if (msg == "s") {
		act = static_cast<ACTION>(2);
		Action = true;
	}
	else if (msg == "w") {
		act = static_cast<ACTION>(3);
		Action = true;
	}

}

/*
 * onCollision
 */
void RobotController::onCollision(CollisionEvent &evt)
{

}

/*
 * ���q�̏�����
 */
void RobotController::initPRTCL(double particle[SIZE][SIZE])
{
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			particle[r][c] = 0.0;
}

/*
 * ���q�̏d�݂̐��K��
 */
void RobotController::normalize_weights()
{
	double sum = 0.0;
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			sum += PRTCL_weight[r][c];
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			PRTCL_weight[r][c] /= sum;
}

/*
 * pos�̍��W�l�ɂ����ē�����k�̊e�����ɕǂ�����̂��ǂ��������o
 * �ǂ����� -> 1
 * �ǂ��Ȃ� -> 0
 * �Ԃ�l : �k���쐼�̕ǂ̗L��
 */
std::vector<int> RobotController::get_wall_status(STATE pos)
{
	std::vector<int> tmpwall;
	int n = WALL[pos.row * 2][pos.col + 1];
	int w = WALL[pos.row * 2 + 1][pos.col];
	int e = WALL[pos.row * 2 + 1][pos.col + 1];
	int s = WALL[pos.row * 2 + 2][pos.col + 1];
	tmpwall.push_back(n);
	tmpwall.push_back(e);
	tmpwall.push_back(s);
	tmpwall.push_back(w);

	return tmpwall;
}

/*
 * 1) ���q���ƂɎ���Ԃ���ԑJ�ڊm����p���ăT���v�����O
 */
void RobotController::sampling(ACTION act)
{
	STATE tmp_pos;
	double newPRTCL[SIZE][SIZE];
	initPRTCL(newPRTCL);

	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++) {
			tmp_pos.row = r;
			tmp_pos.col = c;
			std::vector<int> tmpwall = get_wall_status(tmp_pos);
			int numPRTCL = (int)PRTCL[r][c];

			if (tmpwall[act] == 0) { // �ǂ��Ȃ��̂ŗ��q�̈ړ����\
				switch (act) 
				{
				case NORTH: {
					for (int i = 0; i < numPRTCL; i++){
						if ((double)rand() / RAND_MAX < TRANS) // TRNS�̊m���ŗ��q���ړ��\
							newPRTCL[r - 1][c] += 1;
						else newPRTCL[r][c] += 1; // (1-TRNS)�̊m���ŗ��q���ړ����s
					}
					break;
				}
				case EAST: {
					for (int i = 0; i < numPRTCL; i++){
						if ((double)rand() / RAND_MAX < TRANS) // TRNS�̊m���ŗ��q���ړ��\
							newPRTCL[r][c + 1] += 1;
						else newPRTCL[r][c] += 1; // (1-TRNS)�̊m���ŗ��q���ړ����s
					}
					break;
				}
				case SOUTH: {
					for (int i = 0; i < numPRTCL; i++){
						if ((double)rand() / RAND_MAX < TRANS) // TRNS�̊m���ŗ��q���ړ��\
							newPRTCL[r + 1][c] += 1;
						else newPRTCL[r][c] += 1; // (1-TRNS)�̊m���ŗ��q���ړ����s
					}
					break;
				}
				case WEST: {
					for (int i = 0; i < numPRTCL; i++){
						if ((double)rand() / RAND_MAX < TRANS) // TRNS�̊m���ŗ��q���ړ��\
							newPRTCL[r][c - 1] += 1;
						else newPRTCL[r][c] += 1; // (1-TRNS)�̊m���ŗ��q���ړ����s
					}
					break;
				}
				} // switch�� �����܂�
			}
			else newPRTCL[r][c] = newPRTCL[r][c] + PRTCL[r][c]; // �ǂ�����̂ŗ��q���ړ��ł��Ȃ�����
		}

	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			PRTCL[r][c] = newPRTCL[r][c];
}

/*
 * 2) �Z���T�����v�Z����(o_t)
 */
void RobotController::calcSENSOR(std::vector<int> wall)
{
	STATE pos;
	std::vector<int> tmpwall;
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++) {
			pos.row = r;
			pos.col = c;
			tmpwall = get_wall_status(pos);

			if (tmpwall == wall) // ���ꂼ��̒n�_�̃Z���T���Ɗϑ������Z���T��񂪈�v����ꍇ
				SENSOR[r][c] = KANSOKU;
			else // ���ꂼ��̒n�_�̃Z���T���Ɗϑ������Z���T��񂪈�v���Ȃ��ꍇ
				SENSOR[r][c] = (1 - KANSOKU) / 15;
		}
}

/*
 * 3) ���q�̏d�ݕt��
 */
void RobotController::update_weights()
{
	initPRTCL(PRTCL_weight);
	// �e�}�X�ɑ��݂��闱�q�̏d���̍��v
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			PRTCL_weight[r][c] = PRTCL[r][c] * SENSOR[r][c];

}

/*
 * 4) ���T���v�����O
 *    ���q�̏d�݂ɏ]���ė��q�����T���v�����O
 */
void RobotController::resampling()
{
	std::cout << "���q�̏d�݂̐��K��" << std::endl;
	normalize_weights();
	print(PRTCL_weight);

	// �ݐϕ��z
	double box[SIZE * SIZE] = { 0.0 };
	int n = 0;
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++) {
			box[n] = box[n - 1] + PRTCL_weight[r][c];
			n++;
		}
	
	// �����_���l�𐶐�
	int tmpNum[PA];
	int index[PA];
	double rnd[PA];
	for (int i = 0; i < PA; i++) {
		tmpNum[i] = rand();
		index[i] = 0;
		for (int j = 0; j < i; j++) {
			if (tmpNum[j] < tmpNum[i]) {
				if (index[i] <= index[j])
					index[i] = index[j] + 1;
			}
			else index[j]++;
		}
	}
	for (int i = 0; i < PA; i++)
		rnd[index[i]] = (double)tmpNum[i] / RAND_MAX;

	// �����_���l���J�E���g
	int i = 0, j = 0;
	int NumBox[SIZE * SIZE] = { 0 };
	while (i < PA) {
		if (rnd[i] < box[j]) {
			NumBox[j] += 1;
			i++;
		}
		else {
			j++;
		}
	}

	// �J�E���g��������������
	n = 0;
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++) {
			PRTCL[r][c] = NumBox[n];
			n++;
		}
}

/*
 * ���ׂĂ̍��W�̗^�����f�[�^��\��
 */
void RobotController::print(double particle[SIZE][SIZE])
{
	for (int r = 0; r < SIZE; r++) {
		for (int c = 0; c < SIZE; c++) {
			std::cout << "( " << r << ", " << c << " ):";
			std::cout.width(10);
			std::cout << particle[r][c] << " ";
			std::cout.width();
		}
		std::cout << std::endl;
	}

}

/*
 * Export this function
 */
extern "C" EXPORT_FUNC Controller * createController() 
{
	return new RobotController;
}

