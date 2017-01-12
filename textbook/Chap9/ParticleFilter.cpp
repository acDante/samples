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

	bool Collision;	// 衝突判定
	bool Action;	// 行動していいかどうか

	RobotState ROBOT;
	ACTION act;

	int walkstep;
	int	step;

	void calcSENSOR(std::vector<int> wall);
	void information_integration();
	void normalization();
	void maximum(double probability[SIZE][SIZE]);

	double preSONZAI[SIZE][SIZE];
	double SONZAI[SIZE][SIZE];
	double SENSOR[SIZE][SIZE];
	double G[SIZE][SIZE]; // SONZAIに比例する値

	void init_PARTICLE();
	std::vector<int> get_wall_status(STATE pos);
	void transition(ACTION act);
	void print(double particle[SIZE][SIZE]);

	double PARTICLE[SIZE][SIZE];
	double prePARTICLE[SIZE][SIZE];


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
	static STATE olds, news;
	static std::vector<int> walls;

	switch (ROBOT)
	{
	case INITIALIZE:
		walkstep = 0;
		step = 0;

		Collision = false;
		Action = false;

		init_PARTICLE();

		ROBOT = INITPOSITION;
		break;
	case INITPOSITION:
		robot->getPosition(roboPos);
		news.row = roboPos.z() / 100;
		news.col = roboPos.x() / 100;
		std::cout << "[〇] " /*<< olds.row << ", " << olds.col << std::endl*/;

		ROBOT = CALCULATION;
		break;
	case CALCULATION:
	{
		// 現在位置の壁の情報を取得
		walls = get_wall_status(news);
		for (int i = 0; i < walls.size(); i++)
			std::cout << walls[i] << ", ";
		std::cout << std::endl;

		/* 
		 * step = 0 ：粒子の分布を初期化(均等に分布する)
		 *   - ロボットは進む方角の指示を待つ
		 */ 
		if (step == 0) {
			for (int r = 0; r < SIZE; r++)
				for (int c = 0; c < SIZE; c++)
					prePARTICLE[r][c] = PA / (SIZE * SIZE);
			ROBOT = ACTIONDECISION;
			std::cout << "粒子の分布を初期化" << std::endl;
			print(prePARTICLE);
			step++;
			break;
		}

		std::cout << "1ステップ前の粒子の分布" << std::endl;
		print(prePARTICLE);

		// 1) 粒子ごとに次状態を状態遷移確率を用いてサンプリング
		transition(act);
		std::cout << "移動, 粒子" << std::endl;
		print(PARTICLE); // 粒子の表示

		// 2) センサ情報の観測確率を計算
		calcSENSOR(walls);
		std::cout << "センサ情報の観測確率観測" << std::endl;
		print(SENSOR);

		//// 3) 粒子の重み付け
		//information_integration();
		//std::cout << "情報統合" << std::endl;
		//print(G);

		//// 4) 粒子をリサンプリング
		//normalization();
		//std::cout << "正規化" << std::endl;
		//print(SONZAI); // 存在確率の表示

		//// 自己位置の表示
		//maximum(SONZAI);
		
		ROBOT = NEXTSTEP;
		break;
	}
	case NEXTSTEP:
		for (int r = 0; r < SIZE; r++)
			for (int c = 0; c < SIZE; c++)
				prePARTICLE[r][c] = PARTICLE[r][c];

		step++;
		olds = news;
		ROBOT = ACTIONDECISION;
		break;
	case ACTIONDECISION:
		walkstep = 0;

		if (Action == true) {
			if (act == 0)
				std::cout << "[↑:0] ";
			else if (act == 1)
				std::cout << "[→:1] ";
			else if (act == 2)
				std::cout << "[↓:2] ";
			else
				std::cout << "[←:3] ";
			ROBOT = PREPAREWALK;
			Action = false;
		}
		break;
	case PREPAREWALK:
		// 選択した方向に壁があるとWALKの処理を飛ばす
		if (walls[act] == 1) {
			ROBOT = CALCULATION;
			break;
		}
		//std::cout << "[olds pos] " << olds.row << ", " << olds.col << std::endl;
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
		else {
			robot->getPosition(roboPos);
			news.row = roboPos.z() / 100;
			news.col = roboPos.x() / 100;
			//std::cout << "[news pos] " << news.row << ", " << news.col << std::endl;
			ROBOT = CALCULATION;
		}
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
		//ROBOT = INITPOSITION;
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
 * 粒子フィルタの初期化
 */
void RobotController::init_PARTICLE()
{
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			PARTICLE[r][c] = 0.0;
}

/*
 * posの座標値において東西南北の各方向に壁があるのかどうかを検出
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
 * 1) 粒子ごとに次状態を状態遷移確率を用いてサンプリング
 */
void RobotController::transition(ACTION act)
{
	STATE pos;
	std::vector<int> tmpwall;
	int opposite;

	if (act == 0) opposite = 2;
	else if (act == 1) opposite = 3;
	else if (act == 2) opposite = 0;
	else if (act == 3) opposite = 1;

	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++) {
			pos.row = r;
			pos.col = c;
			tmpwall = get_wall_status(pos);

			double prePosPARTICLE;
			if (act == 0) prePosPARTICLE = prePARTICLE[r + 1][c];
			else if (act == 1) prePosPARTICLE = prePARTICLE[r][c - 1];
			else if (act == 2) prePosPARTICLE = prePARTICLE[r - 1][c];
			else if (act == 3) prePosPARTICLE = prePARTICLE[r][c + 1];

			if (tmpwall[opposite] == 1) // actへの移動を失敗したということ
				PARTICLE[r][c] = prePARTICLE[r][c] * (1 - TRANS);
			else if (tmpwall[opposite] == 0) { // actへの移動を成功したということ
				if (tmpwall[act] == 1) // そもそもactへの移動ができない
					PARTICLE[r][c] = prePosPARTICLE * TRANS + prePARTICLE[r][c];
				else if (tmpwall[act] == 0) // 動いたつもりが動けてない
					PARTICLE[r][c] = prePosPARTICLE * TRANS + prePARTICLE[r][c] * (1 - TRANS);
			}

		}
}

/*
 * 2) センサ情報を計算する
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

			if (tmpwall == wall) // それぞれの地点のセンサ情報と観測したセンサ情報が一致する場合
				SENSOR[r][c] = KANSOKU;
			else // それぞれの地点のセンサ情報と観測したセンサ情報が一致しない場合
				SENSOR[r][c] = (1 - KANSOKU) / 15;
		}
}

/*
 * 3) 移動と観測の情報統合
 */
void RobotController::information_integration()
{
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			G[r][c] = SONZAI[r][c] * SENSOR[r][c];
}

/*
 * 4) 正規化
 */
void RobotController::normalization()
{
	double total;
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			total += G[r][c];

	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			SONZAI[r][c] = G[r][c] / total;
}

/*
 * すべての座標値における粒子を表示
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
 * 存在確率の最大値である座標を表示させる
 */
void RobotController::maximum(double probability[SIZE][SIZE])
{
	double max_value = probability[0][0];
	std::vector<std::pair<int, int> > pair_pos;
		
	for (int r = 0; r < SIZE; r++) {
		for (int c = 0; c < SIZE; c++) {
			if (max_value < probability[r][c]) {
				pair_pos.clear();
				max_value = probability[r][c];
				pair_pos.push_back(std::make_pair(r, c));
			}
			else if (max_value == probability[r][c]) 
				pair_pos.push_back(std::make_pair(r, c));
		}
	}
	if (max_value == probability[0][0]) 
		pair_pos.push_back(std::make_pair(0, 0));
	std::cout << "max value = " << max_value << std::endl;
	std::cout << "max value pos" << std::endl;
	for (int i = 0; i < pair_pos.size(); i++)
		std::cout << "  ( " << pair_pos[i].first << ", " << pair_pos[i].second << " )" << std::endl;
	
}

/*
 * Export this function
 */
extern "C" EXPORT_FUNC Controller * createController() 
{
	return new RobotController;
}

