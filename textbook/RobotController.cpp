#include <sigverse/commonlib/ControllerEvent.h>
#include <sigverse/commonlib/Controller.h>
#include <sigverse/commonlib/Logger.h>
#include <windows.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class RobotController : public Controller 
{
public:
	void	 onInit(InitEvent &evt);
	double	 onAction(ActionEvent&);
	void	 onRecvMsg(RecvMsgEvent &evt);

private:
	// local variables
	void	 East(void);
	void	 West(void);
	void	 South(void);
	void	 North(void);

	void	 GoForward(void);
	void	 TurnRight(void);
	void	 TurnLeft(void);
	void	 GoBackward(void);

	int		 GetReward(void);

	void	 InitialPositionSetting(const std::string fileName);
	void	 parseFile(const std::string fileName);
	bool	 checkPosition(Vector3d pos);
	std::vector<std::string> getWallNum(std::string str);
	void	 setWall(std::vector<std::string> num, std::string axis);

	ViewService *m_view;
	RobotObj *robot;
	Vector3d InitPos;
	Vector3d roboPos;
	Vector3d GoalPos;
	Rotation InitRot;

	std::vector<std::string> m_entities;
	std::vector<std::string> m_walls;
	bool Collision;

};

/*
 * onInit
 */
void RobotController::onInit(InitEvent &evt)
{
	m_view = (ViewService*)connectToService("SIGViewer");
	robot = getRobotObj(myname());

	// 迷路のサイズ(5*5に対応), ゴール位置, 壁の配置を読み込む
	getAllEntities(m_entities);
	for (int i = 0; i < (int)m_entities.size(); i++) {
		if (m_entities[i] != myname() && m_entities[i] != "floor") m_walls.push_back(m_entities[i]);
	}
	parseFile("Configuration.txt");

	// ロボットの初期状態(Position and Rotation)を読み込む
	InitialPositionSetting("InitialPosition.txt");
	robot->getPosition(roboPos);
	robot->getRotation(InitRot);

	// 衝突判定
	Collision = false;
	
}


/*
 * onAction
 */
double RobotController::onAction(ActionEvent &evt) 
{
	// ゴール位置のマスにいれば左手を上げる
	Vector3d pos;
	robot->getPosition(pos);
	if (checkPosition(pos)) {
		robot->setJointAngle("LARM_JOINT2", DEG2RAD(180));
	}
	else {
		robot->setJointAngle("LARM_JOINT2", DEG2RAD(0));
	}
	
	return 1.0;
}

/*
 * onRecvMsg
 */
void RobotController::onRecvMsg(RecvMsgEvent &evt) 
{
	std::string sender = evt.getSender();
	std::string msg = evt.getMsg();
	
	// すべて1マスの移動
	if (msg == "e")	{
		East();	// X軸正の方向に進む
	}
	else if (msg == "w") {
		West(); // X軸負の方向に進む
	}
	else if (msg == "s") {
		South(); // Z軸正の方向に進む
	}
	else if (msg == "n") {
		North(); // Z軸負の方向に進む
	}
	if (msg == "f") {
		GoForward(); // 正面に進む
	}
	else if (msg == "b") {
		GoBackward(); // 後ろに戻る
	}
	else if (msg == "r") {
		TurnRight(); // 右に90度回転する
	}
	else if (msg == "l") {
		TurnLeft(); // 左に90度回転する
	}
	if (msg == "reward") {
		int value = GetReward(); // 報酬を答える
		std::cout << "Reward : " << value << std::endl;
	}
	if (msg == "initial") {
		// 初めの状態に戻る
		std::cout << "Return to the initial position" << std::endl;
		robot->setPosition(InitPos);
		robot->setRotation(InitRot);
		Collision = false;
	}
}


void RobotController::East(void)
{
	Vector3d pos;
	for (int i = 1; i < 11; i++) {
		pos.x(roboPos.x() + i * 10);
		pos.y(roboPos.y());
		pos.z(roboPos.z());

		CParts * parts = robot->getParts("RLEG_LINK6");
		if (parts->getCollisionState()) {
			robot->setPosition(roboPos);
			pos = roboPos;
			Collision = true;
			break;
		}
		else {
			robot->setPosition(pos);
		}
		Sleep(500);
	}
	roboPos = pos;
}

void RobotController::West(void)
{
	Vector3d pos;
	for (int i = 1; i < 11; i++) {
		pos.x(roboPos.x() - i * 10);
		pos.y(roboPos.y());
		pos.z(roboPos.z());

		CParts * parts = robot->getParts("RLEG_LINK6");
		if (parts->getCollisionState()) {
			robot->setPosition(roboPos);
			pos = roboPos;
			Collision = true;
			break;
		}
		else {
			robot->setPosition(pos);
		}
		Sleep(500);
	}
	roboPos = pos;
}

void RobotController::South(void)
{
	Vector3d pos;
	for (int i = 1; i < 11; i++) {
		pos.x(roboPos.x());
		pos.y(roboPos.y());
		pos.z(roboPos.z() + i * 10);

		CParts * parts = robot->getParts("RLEG_LINK6");
		if (parts->getCollisionState()) {
			robot->setPosition(roboPos);
			pos = roboPos;
			Collision = true;
			break;
		}
		else {
			robot->setPosition(pos);
		}
		Sleep(500);
	}
	roboPos = pos;
}

void RobotController::North(void)
{
	Vector3d pos;
	for (int i = 1; i < 11; i++) {
		pos.x(roboPos.x());
		pos.y(roboPos.y());
		pos.z(roboPos.z() - i * 10);

		CParts * parts = robot->getParts("LLEG_LINK6");
		if (parts->getCollisionState()) {
			robot->setPosition(roboPos);
			pos = roboPos;
			Collision = true;
			break;
		}
		else {
			robot->setPosition(pos);
		}
		Sleep(500);
	}
	roboPos = pos;
}

void RobotController::GoForward(void)
{
	Vector3d pos;
	Rotation rot;
	robot->getRotation(rot);
	//クォータニオンから回転角を導出します
	double theta = 2 * asin(rot.qy());
	for (int i = 1; i < 11; i++) {
		double dx = sin(theta) * 10 * i;
		double dz = cos(theta) * 10 * i;

		pos.x(roboPos.x() + dx);
		pos.y(roboPos.y());
		pos.z(roboPos.z() + dz);

		CParts * parts = robot->getParts("RLEG_LINK6");
		if (parts->getCollisionState()) {
			robot->setPosition(roboPos);
			pos = roboPos;
			Collision = true;
			break;
		}
		else {
			robot->setPosition(pos);
		}
		Sleep(500);
	}
	roboPos = pos;
}

void RobotController::GoBackward(void)
{
	Vector3d pos;
	Rotation rot;
	robot->getRotation(rot);
	//クォータニオンから回転角を導出します
	double theta = 2 * asin(rot.qy());
	for (int i = 1; i < 11; i++) {
		double dx = sin(theta) * 10 * i;
		double dz = cos(theta) * 10 * i;

		pos.x(roboPos.x() - dx);
		pos.y(roboPos.y());
		pos.z(roboPos.z() - dz);

		CParts * parts = robot->getParts("RLEG_LINK6");
		if (parts->getCollisionState()) {
			robot->setPosition(roboPos);
			pos = roboPos;
			Collision = true;
			break;
		}
		else {
			robot->setPosition(pos);
		}
		Sleep(500);
	}
	roboPos = pos;
}

void RobotController::TurnRight(void)
{
	Vector3d pos;
	Rotation rot;
	robot->getRotation(rot);
	double theta = 2 * asin(rot.qy());

	//体全体を回転させる
	for (int i = 1; i < 4; i++) {
		double y = theta + DEG2RAD(i * -30);
		if (y >= M_PI) y = y - 2 * M_PI;
		else if (y <= -M_PI) y = y + 2 * M_PI;
		robot->setAxisAndAngle(0, 1.0, 0, y);
		Sleep(500);
	}	
}

void RobotController::TurnLeft(void)
{
	Vector3d pos;
	Rotation rot;
	robot->getRotation(rot);
	double theta = 2 * asin(rot.qy());

	//体全体を回転させる
	for (int i = 1; i < 4; i++) {
		double y = theta + DEG2RAD(i * 30);
		if (y >= M_PI) y = y - 2 * M_PI;
		else if (y <= -M_PI) y = y + 2 * M_PI;
		robot->setAxisAndAngle(0, 1.0, 0, y);
		Sleep(500);
	}
}

int RobotController::GetReward(void)
{
	int value;
	
	if (Collision) {
		value = -1; //衝突:true -> -1
		Collision = false;
	}
	else value = 0;

	
	Vector3d pos;
	robot->getPosition(pos);
	if (checkPosition(pos)) { // ゴール位置:true -> 100
		value = 100;
	}

	return value;
}

// ロボットの初期設定のテキストを読み込む
void RobotController::InitialPositionSetting(const std::string fileName)
{
	std::ifstream ifs;
	ifs.open(fileName.c_str());
	std::string str;
		
	while (getline(ifs, str)) {
		std::stringstream ss;
		std::string header, body;
		ss << str;
		ss >> header >> body;
		if (header == "Position") {
			std::replace(body.begin(), body.end(), ',', ' ');
			std::replace(body.begin(), body.end(), '(', ' ');
			std::replace(body.begin(), body.end(), ')', ' ');
			std::stringstream ssPos;
			std::string PosX_str, PosZ_str;
			ssPos << body;
			ssPos >> PosX_str >> PosZ_str;

			InitPos.x(std::atof(PosX_str.c_str()) * 100);
			InitPos.y(54.0);
			InitPos.z(std::atof(PosZ_str.c_str()) * -100);
			robot->setPosition(InitPos);
		}
		else if (header == "Rotation") {
			std::replace(body.begin(), body.end(), ',', ' ');
			std::replace(body.begin(), body.end(), '(', ' ');
			std::replace(body.begin(), body.end(), ')', ' ');
			std::stringstream ssRot;
			std::string RotQw_str, RotQx_str, RotQy_str, RotQz_str;
			ssRot << body;
			ssRot >> RotQw_str >> RotQx_str >> RotQy_str >> RotQz_str;

			double qw = std::atof(RotQw_str.c_str());
			double qx = std::atof(RotQx_str.c_str());
			double qy = std::atof(RotQy_str.c_str());
			double qz = std::atof(RotQz_str.c_str());
			robot->setRotation(Rotation(qw, qz, qy, qz));
		}
	}
}

// 環境設定のテキストを読み込む
void RobotController::parseFile(const std::string fileName)
{
	std::ifstream fin;
	fin.open(fileName.c_str());
	std::string str, axis;
	std::vector<std::string> WallNum, WallName;

	while (getline(fin, str)) {
		std::stringstream ss;
		std::string header, body, x, z;
		ss << str;
		ss >> header >> body;
		if (header == "Size") {
			std::replace(body.begin(), body.end(), ',', ' ');
			std::replace(body.begin(), body.end(), '(', ' ');
			std::replace(body.begin(), body.end(), ')', ' ');
			std::stringstream ssSize;
			ssSize << body;
			ssSize >> x >> z;

			int xSize = std::atoi(x.c_str());
			int zSize = std::atoi(z.c_str());
			SimObj *floor = getObj("floor");
			floor->setPosition((xSize - 1) * 50, 0, (zSize - 1) * -50);
		}
		else if (header == "Goal") {
			std::replace(body.begin(), body.end(), ',', ' ');
			std::replace(body.begin(), body.end(), '(', ' ');
			std::replace(body.begin(), body.end(), ')', ' ');
			std::stringstream ssSize;
			ssSize << body;
			ssSize >> x >> z;

			int xPos = std::atoi(x.c_str());
			int zPos = std::atoi(z.c_str());
			GoalPos.x(xPos * 100);
			GoalPos.y(54.0);
			GoalPos.z(zPos * -100);
		}
		else if (header == "xWall" || header == "zWall") {
			WallNum = getWallNum(body);
			if (header == "xWall") axis = "x";
			else axis = "z";
			std::string wallNametmp = "wall" + axis + "_";
			for (int i = 0; i < (int)WallNum.size(); i++) {
				std::string wallName = wallNametmp + WallNum[i];
				WallName.push_back(wallName);
			}
		}
	}
	setWall(WallName, axis);
}

// ロボットがゴール位置にいるかどうかを判定
bool RobotController::checkPosition(Vector3d pos)
{
	bool xAxis = false;
	bool zAxis = false;
	
	if ((GoalPos.x() - 50) < pos.x() && pos.x() < (GoalPos.x() + 50)) xAxis = true;
	if ((GoalPos.z() - 50) < pos.z() && pos.z() < (GoalPos.z() + 50)) zAxis = true;

	if (xAxis && zAxis) return true;
	else return false;
}

// 設置する壁の位置を読み込む
std::vector<std::string> RobotController::getWallNum (const std::string str)
{
	std::string wallnum = str;
	std::vector<std::string> walltmp;
	std::vector<std::string> tmp;

	std::replace(wallnum.begin(), wallnum.end(), ')', ' ');
	std::replace(wallnum.begin(), wallnum.end(), '(', ' ');
	wallnum.erase(wallnum.size() - 1);

	std::stringstream ss;
	ss << wallnum;
	while (!ss.eof()) {
		std::string tmp;
		ss >> tmp;
		walltmp.push_back(tmp);
	}
	for (int i = 0; i < (int)walltmp.size(); i++) {
		std::replace(walltmp[i].begin(), walltmp[i].end(), ',', ' ');
		std::stringstream ssNum;
		std::string xNum, zNum;
		ssNum << walltmp[i];
		ssNum >> xNum >> zNum;
		std::string Num = xNum + zNum;
		tmp.push_back(Num);
	}

	return tmp;
}

// 壁を設置する
void RobotController::setWall(std::vector<std::string> num, std::string axis)
{
	std::vector<std::string> removeWalls = m_walls;

	for (int i = 0; i < (int)num.size(); i++) {
		removeWalls.erase(std::remove(removeWalls.begin(), removeWalls.end(), num[i]), removeWalls.end());
	}
	for (int j = 0; j < (int)removeWalls.size(); j++) {
		SimObj *wall = getObj(removeWalls[j].c_str());
		Vector3d wallPos;
		wall->getPosition(wallPos);
		wall->setPosition(wallPos.x(), -100, wallPos.z());
	}
}


/*
 * Export this function
 */
extern "C" EXPORT_FUNC Controller * createController() 
{
	return new RobotController;
}

