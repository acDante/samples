#include <sigverse/commonlib/ControllerEvent.h>
#include <sigverse/commonlib/Controller.h>
#include <sigverse/commonlib/Logger.h>
#include <windows.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>

#include "QLearning.h"

class RobotController : public Controller
{
public:
	void	 onInit(InitEvent &evt);
	double	 onAction(ActionEvent&);
	void	 onRecvMsg(RecvMsgEvent &evt);
	void	 onCollision(CollisionEvent &evt);

private:
	ACTION	 best_value_action(STATE state);
	void	 read_qvalues(void);

	ViewService *m_view;
	RobotObj *robot;
	Vector3d roboPos;

	RobotState ROBOT;


	double reward_value;
	int step;
	int walkstep;
};

/*
 * onInit
 */
void RobotController::onInit(InitEvent &evt)
{
	m_view = (ViewService*)connectToService("SIGViewer");
	robot = getRobotObj(myname());

	walkstep = 0;
	step = 0;
	ROBOT = STOP;
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
		std::cout << "STEP    : " << step << std::endl;
		std::cout << "GOAL    : " << GOAL_ROW << ", " << GOAL_COL << std::endl;
		std::cout << "Old     : " << olds.row << ", " << olds.col << std::endl;

		read_qvalues();

		act = best_value_action(olds);

		walkstep = 0;
		
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
		else ROBOT = GETREWARD;
		break;
	}
	case GETREWARD:
	{
		std::string msg = "reward 0";
		sendMsg("moderator_0", msg);

		ROBOT = STOP;
		break;
	}
	case NEXTSTEP:
		robot->getPosition(roboPos);
		news.row = roboPos.z() / 100;
		news.col = roboPos.x() / 100;
		std::cout << "New     : " << news.row << ", " << news.col << std::endl;

		olds.row = news.row;
		olds.col = news.col;

		if (reward_value == GOAL_REWARD) {
			step = 0;
			sendMsg("moderator_0", "initial");
			ROBOT = STOP;
		}
		else {
			step++;
			ROBOT = ACTIONDECISION;		
		}
		break;
	case STOP:
		std::cout << "---------- STOP ----------" << std::endl;
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
	ROBOT = STOP;
}

/*
 * 学習したQ値を読み込む
 */
void RobotController::read_qvalues(void)
{
	std::ifstream inputfile("Q-values.txt");
	std::string str;
	std::vector<double> values;

	if (inputfile.fail())
		std::cerr << "失敗しました" << std::endl;
	
	while (getline(inputfile, str)) {
		std::stringstream ss;
		ss << str;
		std::string tmpmaze, tmpqval;
		ss >> tmpmaze >> tmpqval;
		
		// qvaluesに代入
		std::replace(tmpmaze.begin(), tmpmaze.end(), '[', ' ');
		std::replace(tmpmaze.begin(), tmpmaze.end(), ']', ' ');
		std::stringstream tmpss;
		tmpss << tmpmaze;
		std::string namestr, rstr, cstr, astr;
		tmpss >> namestr >> rstr >> cstr >> astr;

		int r = std::stoi(rstr);
		int c = std::stoi(cstr);
		int a = std::stoi(astr);
		double qval = std::stod(tmpqval);
		
		qvalues[r][c][a] = qval;
	}
}

/*
 * 状態stateの時のQ値が最大になる行動を選択する
 */
ACTION RobotController::best_value_action(STATE state)
{
	double best_value = -100000.0;
	ACTION act = static_cast<ACTION>(-1);
	std::vector<int> tmpAct;

	for (int a = NORTH; a <= WEST; a++) {
		//std::cout << "best_val : " << best_value << std::endl;
		//std::cout << "curr_val : " << qvalues[state.row][state.col][a] << std::endl;
		if (qvalues[state.row][state.col][a] > best_value) {
			best_value = qvalues[state.row][state.col][a];
			act = static_cast<ACTION>(a);
			tmpAct.push_back(act);
		}
		else if (qvalues[state.row][state.col][a] == best_value){
			// Q値が等しいものがあった場合
			act = static_cast<ACTION>(a);
			tmpAct.push_back(act);
			if (!tmpAct.empty()) {
				// Q値が等しいものがあった場合，ランダムに行動を選択する
				int tmpNum = rand() % tmpAct.size();
				act = static_cast<ACTION>(tmpAct[tmpNum]);
			}
		}
	}

	if (act == -1)
		fprintf(stderr, "Fatal error in best_qvalue_action (%d)\n", (int)act);

	if (act == 0)
		std::cout << "[↑:0]" << std::endl;
	else if (act == 1)
		std::cout << "[→:1]" << std::endl;
	else if (act == 2)
		std::cout << "[↓:2]" << std::endl;
	else
		std::cout << "[←:3]" << std::endl;

	return act;
}

/*
 * Export this function
 */
extern "C" EXPORT_FUNC Controller * createController()
{
	return new RobotController;
}
