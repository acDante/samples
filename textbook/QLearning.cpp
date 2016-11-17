#include <sigverse/commonlib/ControllerEvent.h>
#include <sigverse/commonlib/Controller.h>
#include <sigverse/commonlib/Logger.h>
#include <windows.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>

#include "QLearning.h"

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )


class RobotController : public Controller 
{
public:
	void	 onInit(InitEvent &evt);
	double	 onAction(ActionEvent&);
	void	 onRecvMsg(RecvMsgEvent &evt);
	void	 onCollision(CollisionEvent &evt);

private:
	// local variables
	// QLearn
	void	 init_qvalues();
	void	 update_qvalues(STATE olds, STATE news, ACTION action, double r);
	double	 best_qvalue(STATE state);
	ACTION	 eGreedy(STATE state, int step);
	double	 reward_value;
	int		 episode;
	int		 step;

	ViewService *m_view;
	RobotObj *robot;
	Vector3d roboPos;
	//衝突判定
	bool Collision;
	bool initPosition;
	bool startStep;

	RobotState ROBOT;
	int walkstep;
};

/*
 * onInit
 */
void RobotController::onInit(InitEvent &evt)
{
	m_view = (ViewService*)connectToService("SIGViewer");
	robot = getRobotObj(myname());
	Collision = false;
	initPosition = false;
	startStep = false;
	episode = 0;
	step = 0;

	walkstep = 0;
}


/*
 * onAction
 */
double RobotController::onAction(ActionEvent &evt) 
{
	static STATE olds, news;
	static ACTION act;

	switch (ROBOT)
	{
	case INITPOSITION:
		robot->getPosition(roboPos);
		std::cout << "INI : " << roboPos.x() << " " << roboPos.y() << " " << roboPos.z() << std::endl;
		olds.row = roboPos.z() / 100; 
		olds.col = roboPos.x() / 100;
		std::cout << "[〇] " << olds.row << ", " << olds.col << std::endl;
		ROBOT = ACTIONDECISION;
		break;
	case ACTIONDECISION:
		std::cout << std::endl;
		std::cout << "*************************" << std::endl;
		std::cout << "EPISODE : " << episode << std::endl;
		std::cout << "STEP    : " << step << std::endl;
		std::cout << "GOAL    : " << GOAL_ROW << ", " << GOAL_COL << std::endl;
		std::cout << "Old     : " << olds.row << ", " << olds.col << std::endl;

		walkstep = 0;

		act = eGreedy(olds, step);
		state_action_recency[olds.row][olds.col][act] = step;

		ROBOT = WALK;
		break;
	case WALK:
	{
		Vector3d pos;
		walkstep++;
		if (walkstep < 11) {
			//std::cout << "PRE : " << roboPos.x() << " " << roboPos.y() << " "<< roboPos.z() << std::endl;
			switch (act)
			{
			case WEST:
				pos.x(roboPos.x() - walkstep * 10);
				pos.y(roboPos.y());
				pos.z(roboPos.z());
				//std::cout << "GO  : " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;

				robot->setPosition(pos);

				break;
			case EAST:
				pos.x(roboPos.x() + walkstep * 10);
				pos.y(roboPos.y());
				pos.z(roboPos.z());
				//std::cout << "GO  : " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;

				robot->setPosition(pos);

				break;
			case NORTH:
				pos.x(roboPos.x());
				pos.y(roboPos.y());
				pos.z(roboPos.z() - walkstep * 10);
				//std::cout << "GO  : " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;

				robot->setPosition(pos);

				break;
			case SOUTH:
				pos.x(roboPos.x());
				pos.y(roboPos.y());
				pos.z(roboPos.z() + walkstep * 10);
				//std::cout << "GO  : " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;

				robot->setPosition(pos);

				break;
			}
			ROBOT = WALK;
		}
		else {
			ROBOT = GETREWARD;
		}
		
		break;

	}
	case GETREWARD:
	{
		std::stringstream ss;
		ss << Collision;
		std::string msg = "reward " + ss.str();
		sendMsg("moderator_0", msg);

		ROBOT = STOP;
		break;
	}
	case NEXTSTEP:
		robot->getPosition(roboPos);
		//std::cout << "New : " << roboPos.x() << " " << roboPos.y() << " " << roboPos.z() << std::endl;
		news.row = roboPos.z() / 100;
		news.col = roboPos.x() / 100;
		std::cout << "New     : " << news.row << ", " << news.col << std::endl;

		update_qvalues(olds, news, act, reward_value);

		olds.row = news.row;
		olds.col = news.col;

		Collision = false;
		if (reward_value == GOAL_REWARD) {
			step = 0;
			episode++;
			sendMsg("moderator_0", "initial");
			ROBOT = STOP;
		}
		else if (reward_value == HIT_WALL_PENALTY || reward_value == ONE_STEP_PENALTY){
			step++;
			ROBOT = ACTIONDECISION;
		}
		break;
	case STOP:
		Collision = false;
		std::cout << "STOP STOP STOP STOP" << std::endl;
		break;

	}


	return 0.05;
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

	if (header == "reward") {
		robot->getPosition(roboPos);

		std::stringstream ssbody;
		ssbody << body;
		ssbody >> reward_value;
		
		ROBOT = NEXTSTEP;
	}

	if (msg == "QLearning")
		ROBOT = INITPOSITION;
	

}

/*
 * onCollision
 */
void RobotController::onCollision(CollisionEvent &evt)
{
	robot->setPosition(roboPos);
	Collision = true;

	ROBOT = GETREWARD;
}


/*
 * Q値を初期化する
 */
void RobotController::init_qvalues()
{
	for (int r = 0; r < (SIZE - 1); r++)
		for (int c = 0; c < (SIZE - 1); c++)
			for (int a = NORTH; a <= WEST; a++){
				qvalues[r][c][a] = 0.0;
				state_action_recency[r][c][a] = 0;
			}
				
}

/*
 * Q値の更新
 */
void RobotController::update_qvalues(STATE olds, STATE news, ACTION action, double r)
{
	double best_new_qval, qval;

	best_new_qval = best_qvalue(news);
	qval = qvalues[olds.row][olds.col][action];
	qvalues[olds.row][olds.col][action] = (1 - BETA) * qval + BETA * (r + GAMMA * best_new_qval);

	std::cout << "Q-value[" << olds.row << "][" << olds.col << "][" << action << "] = " << qvalues[olds.row][olds.col][action] << std::endl;
}

/*
 * 状態stateで行ったときにQ値が最大となるような行動
 */
double RobotController::best_qvalue(STATE state)
{
	double best_val = -1000000.0;
	int a;

	for (a = NORTH; a <= WEST; a++) {
		if (qvalues[state.row][state.col][a] > best_val) {
			best_val = qvalues[state.row][state.col][a];
		}
	}
	return (best_val);
}


/*
 * epsilon-greedy method
 */
ACTION RobotController::eGreedy(STATE state, int step)
{
	ACTION act = NORTH;
	double rvalue = rand();
	double best_val = 0.0;
	double curr_val = -1000000.0;
	std::vector<int> tmpAct;

	if (rvalue > EPSILON) {
		// (1 - EPSILON)の確率 ランダムにactを選択
		act = static_cast<ACTION>(rand() % (NUM_ACTIONS));
		if (act < 0 || act > 3)
			fprintf(stderr, "Fatal error in choose_best_action rand-(%d)\n", (int)act);
	}
	else {
		// EPSILONの確率 Q値が最大となるようなactionを選択
		for (int index = 0; index < NUM_ACTIONS; index++) {
			curr_val = qvalues[state.row][state.col][static_cast<ACTION>(index)];
			if (curr_val > best_val) {
				best_val = curr_val;
				act = static_cast<ACTION>(index);
				tmpAct.push_back(act);
			}
			else if (curr_val == best_val) {
				// Q値が等しいものがあった場合 (未確認 2016/11/17) 
				act = static_cast<ACTION>(index);
				tmpAct.push_back(act);
				int tmpNum = rand() % tmpAct.size();
				act = static_cast<ACTION>(tmpAct[tmpNum]);
			}
		}

	}
	if (act < 0 || act > 3)
		fprintf(stderr, "Fatal error in eGreedy (%d)\n", (int)act);
	
	if (act == 0) 
		std::cout << "[↑:0] ";
	else if (act == 1) 
		std::cout << "[→:1] ";
	else if (act == 2) 
		std::cout << "[↓:2] ";
	else 
		std::cout << "[←:3] ";

	return (act);
}

/*
 * Export this function
 */
extern "C" EXPORT_FUNC Controller * createController() 
{
	return new RobotController;
}

