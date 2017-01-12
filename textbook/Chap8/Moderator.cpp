#include <sigverse/commonlib/ControllerEvent.h>
#include <sigverse/commonlib/Controller.h>
#include <sigverse/commonlib/Logger.h>
#include <windows.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>

#include "BayesianFilter.h"

class Moderator : public Controller 
{
public:
	void	 onInit(InitEvent &evt);
	double	 onAction(ActionEvent&);
	void	 onRecvMsg(RecvMsgEvent &evt);

private:
	// �����ݒ���s���֐�
	void	 InitRobot(void);	// ���{�b�g�̃X�^�[�g�ʒu��ݒ肷��
	void	 InitWorld(void);	// ���ݒ���s��
	void	 setWall(void);	// ���H�̕ǂ�ݒu����



	ViewService *m_view;
	RobotObj *robot;
	Vector3d InitPos;
	Vector3d roboPos;
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

	if (msg == "initial") {
		// ���߂̏�Ԃɖ߂�
		std::cout << "[Moderator] : " << "New initial position" << std::endl;
		InitRobot();
		sendMsg(sender, "BayesianFilter");
		
	}
}


/*
 * ���{�b�g�̏����ݒ�
 */
void Moderator::InitRobot(void)
{

	int XPos = rand() % (SIZE - 1);
	int ZPos = rand() % (SIZE - 1);
	// XPos -> col, ZPos -> row
	std::cout << "[Moderator] init Position : " << ZPos << ", " << XPos << std::endl;

	InitPos.x(XPos * 100);
	InitPos.y(54.0);
	InitPos.z(ZPos * 100);

	robot->setPosition(InitPos);
	sendMsg(robotName, "BayesianFilter");
}

/*
 * ���ݒ�
 */
void Moderator::InitWorld(void)
{
	SimObj *floor = getObj("floor");
	floor->setPosition((FLOORSIZE - 1) * 50, 0, (FLOORSIZE - 1) * 50);

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

