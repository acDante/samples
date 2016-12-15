#include <sigverse/commonlib/ControllerEvent.h>
#include <sigverse/commonlib/Controller.h>
#include <sigverse/commonlib/Logger.h>
#include <windows.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>

#include "BayesianFilter.h"

/* 5*5 */
int WALL[2 * SIZE + 1][SIZE + 1] = {
	{ -1, 1, 1, 1, 1, 1 },			// -1はダミー
	{  0, 1, 0, 0, 1, 1 },			//  1は壁
	{ -1, 0, 1, 1, 0, 0 },			//  0は通れる
	{  1, 0, 0, 0, 1, 1 },
	{ -1, 0, 1, 1, 0, 0 },
	{  1, 1, 0, 1, 0, 1 },
	{ -1, 0, 1, 0, 1, 0 },
	{  1, 0, 0, 1, 0, 1 },
	{ -1, 1, 0, 0, 0, 1 },
	{  1, 0, 1, 1, 0, 0 },
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

class Moderator : public Controller 
{
public:
	void	 onInit(InitEvent &evt);
	double	 onAction(ActionEvent&);
	void	 onRecvMsg(RecvMsgEvent &evt);

private:
	void	 init_sonzai();
	double	 update_sonzai();
	int		 GetSensor(std::string act);
	void	 printPro();
	double SONZAI[SIZE][SIZE];

	// 初期設定を行う関数
	void	 InitRobot(void);	// ロボットのスタート位置を設定する
	void	 InitWorld(void);	// 環境設定を行う
	void	 setWall(void);	// 迷路の壁を設置する
	bool	 checkPosition(Vector3d pos);	// ゴールにいるか判定を行う

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

	init_sonzai();
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

	if (header == "sensor") {
		int wall = GetSensor(body);
		std::stringstream ssMsg;
		ssMsg << wall;
		std::string msg = "sensor " + ssMsg.str();
		//std::cout << msg << std::endl;
		sendMsg(sender, msg);
	}
	if (msg == "initial") {
		// 初めの状態に戻る
		std::cout << "[Moderator] : " << "New initial position" << std::endl;
		InitRobot();
		sendMsg(sender, "BayesianFilter");
		
	}
}

/*
 * 存在確率を初期化する
 */
void Moderator::init_sonzai()
{
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			SONZAI[r][c] = 0.0;

}

/*
 * ロボットが選択した行動に沿って存在確率を更新
 */
int Moderator::GetSensor(std::string act)
{
	if (act == "100") {	// 無情報
		for (int r = 0; r < SIZE; r++)
			for (int c = 0; c < SIZE; c++)
				SONZAI[r][c] = 1.0 / (SIZE * SIZE);
	}
	else {

	}
	printPro();
	
	int wall;
	//int n = WALL[row * 2][col + 1];
	//int w = WALL[row * 2 + 1][col];
	//int e = WALL[row * 2 + 1][col + 1];
	//int s = WALL[row * 2 + 2][col + 1];
	//std::cout << n << ", " << w << ", " << e << ", " << s << std::endl;
	
	return wall;
}

/*
 * 地図の存在確率を表示させる
 */
void Moderator::printPro()
{
	for (int r = 0; r < SIZE; r++) {
		for (int c = 0; c < SIZE; c++)
			std::cout << "( " << r << ", " << c << " ):" << SONZAI[r][c] << " ";
		std::cout << std::endl;
	}

}

/*
 * ロボットの初期設定
 */
void Moderator::InitRobot(void)
{

	//int XPos = rand() % (SIZE - 1);
	//int ZPos = rand() % (SIZE - 1);
	//std::cout << "[Moderator] init Position : " << XPos << ", " << ZPos << std::endl;

	//InitPos.x(XPos * 100);
	//InitPos.y(54.0);
	//InitPos.z(ZPos * 100);

	//robot->setPosition(InitPos);
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

