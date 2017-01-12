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
	// 初期設定を行う関数
	void	 InitRobot(void);	// ロボットのスタート位置を設定する
	void	 InitWorld(void);	// 環境設定を行う
	void	 setWall(void);	// 迷路の壁を設置する



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
	
	// 迷路のサイズ, ゴール位置, 壁の配置を読み込む
	InitWorld();

	// ロボットの初期状態(Position and Rotation)を設定する
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
		// 初めの状態に戻る
		std::cout << "[Moderator] : " << "New initial position" << std::endl;
		InitRobot();
		sendMsg(sender, "BayesianFilter");
		
	}
}


/*
 * ロボットの初期設定
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
 * 環境設定
 */
void Moderator::InitWorld(void)
{
	SimObj *floor = getObj("floor");
	floor->setPosition((FLOORSIZE - 1) * 50, 0, (FLOORSIZE - 1) * 50);

	setWall();
}

/*
 * 壁を設置する
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

