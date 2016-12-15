#include <sigverse/commonlib/ControllerEvent.h>
#include <sigverse/commonlib/Controller.h>
#include <sigverse/commonlib/Logger.h>
#include <windows.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>

#include "BayesianFilter.h"

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )


class RobotController : public Controller 
{
public:
	void	 onInit(InitEvent &evt);
	double	 onAction(ActionEvent&);
	void	 onRecvMsg(RecvMsgEvent &evt);
	void	 onCollision(CollisionEvent &evt);

private:
	void	 init_sonzai();
	void	 printPro();

	ViewService *m_view;
	RobotObj *robot;
	Vector3d roboPos;

	double SONZAI[SIZE][SIZE];
	
	bool Collision;	// 衝突判定
	bool Action;	// 行動していいかどうか

	RobotState ROBOT;
	ACTION act;

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

	init_sonzai();
	walkstep = 0;
	step = 0;

	Collision = false;
	Action = false;
}


/*
 * onAction
 */
double RobotController::onAction(ActionEvent &evt) 
{
	static STATE olds, news;

	switch (ROBOT)
	{
	case INITPOSITION:
		robot->getPosition(roboPos);
		// std::cout << "INI : " << roboPos.x() << " " << roboPos.y() << " " << roboPos.z() << std::endl;
		olds.row = roboPos.z() / 100;
		olds.col = roboPos.x() / 100;
		std::cout << "[〇] " /*<< olds.row << ", " << olds.col << std::endl*/;

		ROBOT = GETSENSOR;
		break;
	case GETSENSOR:
	{
		Action = false;
		robot->getPosition(roboPos);
		news.row = roboPos.z() / 100;
		news.col = roboPos.x() / 100;
		std::stringstream ss;
		ss << news.row << ":" << news.col;
		std::cout << news.row << ", " << news.col << std::endl;

		std::string msg = "sensor " + ss.str();
		sendMsg("moderator_0", msg);

		ROBOT = STOP;
		break;
	}
	case CALCILATION:
		if (step == 0) {	// 無情報
			for (int r = 0; r < SIZE; r++)
				for (int c = 0; c < SIZE; c++)
					SONZAI[r][c] = 1.0 / (SIZE * SIZE);
		}
		else {

		}
		printPro();
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
			ROBOT = WALK;
		}
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
		else ROBOT = GETSENSOR;
		break;
	}
	case STOP:
		Collision = false;
		std::cout << "---------- STOP ----------" << std::endl;
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

	if (msg == "BayesianFilter")
		ROBOT = INITPOSITION;

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

	if (header == "sensor") {
		ROBOT = CALCILATION;
	}
}

/*
 * onCollision
 */
void RobotController::onCollision(CollisionEvent &evt)
{

}

/*
* 存在確率を初期化する
*/
void RobotController::init_sonzai()
{
	for (int r = 0; r < SIZE; r++)
		for (int c = 0; c < SIZE; c++)
			SONZAI[r][c] = 0.0;

}

/*
 * 地図の存在確率を表示させる
 */
void RobotController::printPro()
{
	for (int r = 0; r < SIZE; r++) {
		for (int c = 0; c < SIZE; c++)
			std::cout << "( " << r << ", " << c << " ):" << SONZAI[r][c] << " ";
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

