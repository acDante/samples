#include <sigverse/commonlib/ControllerEvent.h>
#include <sigverse/commonlib/Controller.h>
#include <sigverse/commonlib/Logger.h>
#include <windows.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>

#include "QLearning.h"

/* 5*5 */
//int WALL[2 * SIZE + 1][SIZE + 1] = {
//	{ -1, 1, 1, 1, 1, 1 },			// -1�̓_�~�[
//	{  1, 0, 1, 1, 0, 1 },			//  1�͕�
//	{ -1, 1, 0, 0, 0, 1 },			//  0�͒ʂ��
//	{  1, 0, 0, 1, 0, 1 },
//	{ -1, 0, 1, 0, 1, 0 },
//	{  1, 1, 0, 1, 0, 1 },
//	{ -1, 0, 1, 1, 0, 0 },
//	{  1, 0, 0, 0, 1, 1 },
//	{ -1, 0, 1, 1, 0, 0 },
//	{  1, 1, 0, 0, 1, 1 },
//	{ -1, 1, 1, 1, 1, 1 } };

/* 3*3 */
int WALL[2 * SIZE + 1][SIZE + 1] = {
	{ -1, 1, 1, 1 },			// -1�̓_�~�[
	{  1, 1, 0, 1 },			//  1�͕�
	{ -1, 0, 0, 1 },			//  0�͒ʂ��
	{  1, 0, 0, 1 },
	{ -1, 0, 1, 0 },
	{  1, 0, 1, 1 },
	{ -1, 1, 1, 1 } };

class Moderator : public Controller 
{
public:
	void	 onInit(InitEvent &evt);
	double	 onAction(ActionEvent&);
	void	 onRecvMsg(RecvMsgEvent &evt);

private:
	// ��V��Ԃ��֐�
	double	 GetReward(std::string colli);

	// �����ݒ���s���֐�
	void	 InitRobot(void);	// ���{�b�g�̃X�^�[�g�ʒu��ݒ肷��
	void	 InitWorld(void);	// ���ݒ���s��
	void	 setWall(void);	// ���H�̕ǂ�ݒu����
	bool	 checkPosition(Vector3d pos);	// �S�[���ɂ��邩������s��

	ViewService *m_view;
	RobotObj *robot;
	Vector3d InitPos;
	Vector3d roboPos;
	Vector3d GoalPos;
	Rotation InitRot;
	
	std::string robotName;
};

/*
 * onInit
 */
void Moderator::onInit(InitEvent &evt)
{
	m_view = (ViewService*)connectToService("SIGViewer");
	robotName = "robot_000";
	robot = getRobotObj(robotName.c_str());

	// ���H�̃T�C�Y, �S�[���ʒu, �ǂ̔z�u��ǂݍ���
	InitWorld();

	// ���{�b�g�̏������(Position and Rotation)��ݒ肷��
	InitRobot();
	robot->getPosition(roboPos);
	robot->getRotation(InitRot);

}


/*
 * onAction
 */
double Moderator::onAction(ActionEvent &evt)
{

	return 1.0;
}

/*
 * onRecvMsg
 */
void Moderator::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	std::string msg = evt.getMsg();

	std::stringstream ss;
	ss << msg;
	std::string header, body;
	ss >> header >> body;

	if (header == "reward") {
		double value = GetReward(body);
		std::stringstream ssMsg;
		ssMsg << value;
		std::string msg = "reward " + ssMsg.str();
		std::cout << msg << std::endl;
		sendMsg(sender, msg);
	}
	if (msg == "initial") {
		// ���߂̏�Ԃɖ߂�
		std::cout << "[Moderator] : " << "New initial position" << std::endl;
		InitRobot();
		sendMsg(sender, "QLearning");
	}
}

/*
 * ��V��Ԃ�
 */
double Moderator::GetReward(std::string colli)
{
	if (colli == "1") {
		return HIT_WALL_PENALTY; // �ǂɏՓ�
	}
	else {
		Vector3d pos;
		robot->getPosition(pos);
		if (checkPosition(pos)) { // �S�[���ʒu�ɂ��邩�ǂ���
			return GOAL_REWARD;
		}
		else return ONE_STEP_PENALTY;	// 1�X�e�b�v�o��
	}
	
}


/*
 * ���{�b�g�̏����ݒ�
 */
void Moderator::InitRobot(void)
{

	int XPos = rand() % (SIZE - 1);
	int ZPos = rand() % (SIZE - 1);
	std::cout << "[Moderator] init Position : " << XPos << ", " << ZPos << std::endl;

	InitPos.x(XPos * 100);
	InitPos.y(54.0);
	InitPos.z(ZPos * 100);

	robot->setPosition(InitPos);
	sendMsg(robotName, "QLearning");
}

/*
 * ���{�b�g���S�[���ʒu�ɂ��邩�ǂ����𔻒�
 */
bool Moderator::checkPosition(Vector3d pos)
{
	bool xAxis = false;
	bool zAxis = false;

	if ((GoalPos.x() - 50) < pos.x() && pos.x() < (GoalPos.x() + 50)) xAxis = true;
	if ((GoalPos.z() - 50) < pos.z() && pos.z() < (GoalPos.z() + 50)) zAxis = true;

	if (xAxis && zAxis) return true;
	else return false;
}

/*
 * ���ݒ�
 */
void Moderator::InitWorld(void)
{
	SimObj *floor = getObj("floor");
	floor->setPosition((FLOORSIZE - 1) * 50, 0, (FLOORSIZE - 1) * 50);

	GoalPos.x(GOAL_COL * 100);
	GoalPos.y(54.0);
	GoalPos.z(GOAL_ROW * 100);
	setWall();
}

/*
 * �ǂ�ݒu����
 */
void Moderator::setWall(void)
{
	std::vector<std::string> moveWalls;

	for (int y = 0; y < 2 * SIZE + 1; y++) {
		if (y % 2 == 0) {
			for (int x = 1; x < SIZE + 1; x++) {
				if (WALL[y][x] == 1) {
					std::stringstream ssX, ssY;
					ssX << x - 1;
					ssY << y / 2;
					std::string wallName = "wallx_" + ssX.str() + ssY.str();
					moveWalls.push_back(wallName);
				}
			}
		}
		else {
			for (int x = 0; x < SIZE + 1; x++) {
				if (WALL[y][x] == 1) {
					std::stringstream ssX, ssY;
					ssX << x;
					ssY << y / 2;
					std::string wallName = "wallz_" + ssX.str() + ssY.str();
					moveWalls.push_back(wallName);
				}
			}
		}
	}

	for (int j = 0; j < (int)moveWalls.size(); j++) {
		SimObj *wall = getObj(moveWalls[j].c_str());
		//std::cout << removeWalls[j] << std::endl;
		Vector3d wallPos;
		wall->getPosition(wallPos);
		wall->setPosition(wallPos.x(), 50, wallPos.z());
	}
}


/*
 * Export this function
 */
extern "C" EXPORT_FUNC Controller * createController() 
{
	return new Moderator;
}

