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
//	{ -1, 1, 1, 1, 1, 1 },			// -1はダミー
//	{  1, 0, 1, 1, 0, 1 },			//  1は壁
//	{ -1, 1, 0, 0, 0, 1 },			//  0は通れる
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
	{ -1, 1, 1, 1 },			// -1はダミー
	{  1, 1, 0, 1 },			//  1は壁
	{ -1, 0, 0, 1 },			//  0は通れる
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
	// 報酬を返す関数
	double	 GetReward(std::string colli);

	// 初期設定を行う関数
	void	 InitRobot(void);	// ロボットのスタート位置を設定する
	void	 InitWorld(void);	// 環境設定を行う
	void	 setWall(void);	// 迷路の壁を設置する
	bool	 checkPosition(Vector3d pos);	// ゴールにいるか判定を行う

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

	if (header == "reward") {
		double value = GetReward(body);
		std::stringstream ssMsg;
		ssMsg << value;
		std::string msg = "reward " + ssMsg.str();
		std::cout << msg << std::endl;
		sendMsg(sender, msg);
	}
	if (msg == "initial") {
		// 初めの状態に戻る
		std::cout << "[Moderator] : " << "New initial position" << std::endl;
		InitRobot();
		sendMsg(sender, "QLearning");
	}
}

/*
 * 報酬を返す
 */
double Moderator::GetReward(std::string colli)
{
	if (colli == "1") {
		return HIT_WALL_PENALTY; // 壁に衝突
	}
	else {
		Vector3d pos;
		robot->getPosition(pos);
		if (checkPosition(pos)) { // ゴール位置にいるかどうか
			return GOAL_REWARD;
		}
		else return ONE_STEP_PENALTY;	// 1ステップ経過
	}
	
}


/*
 * ロボットの初期設定
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
 * ロボットがゴール位置にいるかどうかを判定
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
 * 環境設定
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

