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

	void initPRTCL(double particle[SIZE][SIZE]);	// 粒子の初期化
	void normalize_weights();	// 粒子の重みの正規化
	std::vector<int> get_wall_status(STATE pos);
	void sampling(ACTION act);	// サンプリング
	void calcSENSOR(std::vector<int> wall);	// センサ情報の観測確率
	void update_weights();	// 粒子の重み付け
	void resampling();
	void print(double particle[SIZE][SIZE]);	// 表示

	double PRTCL[SIZE][SIZE];	// 粒子が格納されている配列
	double PRTCL_weight[SIZE][SIZE];	// 重み付けされた粒子が格納されている配列
	double SENSOR[SIZE][SIZE];	// センサ情報の観測確率が格納されている配列

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
		// 現在位置の壁の情報を取得
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
		 * step = 0 ：粒子の分布を初期化(均等に分布する)
		 *   -> ロボットは進む方角の指示を待つ
		 */
		if (step == 0) {
			for (int r = 0; r < SIZE; r++)
				for (int c = 0; c < SIZE; c++) 
					PRTCL[r][c] = PA / (SIZE * SIZE);
			step++;

			ROBOT = ACTIONDECISION;
			break;
		}
		std::cout << "行動前の粒子" << std::endl;
		print(PRTCL);

		// 1) 粒子ごとに次状態を状態遷移確率を用いてサンプリング
		sampling(act);
		std::cout << "粒子のサンプリング" << std::endl;
		print(PRTCL);

		// 2) センサ情報の観測確率を計算
		calcSENSOR(walls);
		std::cout << "センサ情報の観測確率" << std::endl;
		print(SENSOR);

		// 3) 粒子の重み付け
		update_weights();
		std::cout << "粒子の重み付け" << std::endl;
		print(PRTCL_weight);

		// 4) リサンプリング
		resampling();
		std::cout << "リサンプリング" << std::endl;
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
			std::cout << "壁があって動けません" << std::endl;
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
 * 粒子の初期化
 */
void RobotController::initPRTCL(double particle[SIZE][SIZE])
{
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			particle[r][c] = 0.0;
}

/*
 * 粒子の重みの正規化
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
 * posの座標値において東西南北の各方向に壁があるのかどうかを検出
 * 壁がある -> 1
 * 壁がない -> 0
 * 返り値 : 北東南西の壁の有無
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

			if (tmpwall[act] == 0) { // 壁がないので粒子の移動が可能
				switch (act) 
				{
				case NORTH: {
					for (int i = 0; i < numPRTCL; i++){
						if ((double)rand() / RAND_MAX < TRANS) // TRNSの確率で粒子が移動可能
							newPRTCL[r - 1][c] += 1;
						else newPRTCL[r][c] += 1; // (1-TRNS)の確率で粒子が移動失敗
					}
					break;
				}
				case EAST: {
					for (int i = 0; i < numPRTCL; i++){
						if ((double)rand() / RAND_MAX < TRANS) // TRNSの確率で粒子が移動可能
							newPRTCL[r][c + 1] += 1;
						else newPRTCL[r][c] += 1; // (1-TRNS)の確率で粒子が移動失敗
					}
					break;
				}
				case SOUTH: {
					for (int i = 0; i < numPRTCL; i++){
						if ((double)rand() / RAND_MAX < TRANS) // TRNSの確率で粒子が移動可能
							newPRTCL[r + 1][c] += 1;
						else newPRTCL[r][c] += 1; // (1-TRNS)の確率で粒子が移動失敗
					}
					break;
				}
				case WEST: {
					for (int i = 0; i < numPRTCL; i++){
						if ((double)rand() / RAND_MAX < TRANS) // TRNSの確率で粒子が移動可能
							newPRTCL[r][c - 1] += 1;
						else newPRTCL[r][c] += 1; // (1-TRNS)の確率で粒子が移動失敗
					}
					break;
				}
				} // switch文 ここまで
			}
			else newPRTCL[r][c] = newPRTCL[r][c] + PRTCL[r][c]; // 壁があるので粒子が移動できなかった
		}

	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			PRTCL[r][c] = newPRTCL[r][c];
}

/*
 * 2) センサ情報を計算する(o_t)
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
 * 3) 粒子の重み付け
 */
void RobotController::update_weights()
{
	initPRTCL(PRTCL_weight);
	// 各マスに存在する粒子の重さの合計
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			PRTCL_weight[r][c] = PRTCL[r][c] * SENSOR[r][c];

}

/*
 * 4) リサンプリング
 *    粒子の重みに従って粒子をリサンプリング
 */
void RobotController::resampling()
{
	std::cout << "粒子の重みの正規化" << std::endl;
	normalize_weights();
	print(PRTCL_weight);

	// 累積分布
	double box[SIZE * SIZE] = { 0.0 };
	int n = 0;
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++) {
			box[n] = box[n - 1] + PRTCL_weight[r][c];
			n++;
		}
	
	// ランダム値を生成
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

	// ランダム値をカウント
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

	// カウントした数を代入する
	n = 0;
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++) {
			PRTCL[r][c] = NumBox[n];
			n++;
		}
}

/*
 * すべての座標の与えたデータを表示
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

