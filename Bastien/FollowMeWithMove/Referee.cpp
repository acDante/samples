#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>
#include <string> 
#include <iostream>
#include <math.h>
#include <unistd.h>

#define WAIT_DISTANCE 200

using namespace std;

class MyController : public Controller
{
public:
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	void onCollision(CollisionEvent &evt); 

private:
	SimObj *robotObj;
	SimObj *operatorObj;

	std::string robotName;
	std::string operatorName;

	//bool task;
	bool follow;
};

void MyController::onInit(InitEvent &evt) 
{
	robotName    = "robot_004";
	operatorName = "operator";

	//task = false;
	follow = false;
}

double MyController::onAction(ActionEvent &evt)
{
	robotObj = getObj(robotName.c_str());
	operatorObj = getObj(operatorName.c_str());

	Vector3d robotPos;
	robotObj->getPosition(robotPos);

	// 位置取得
	Vector3d operatorPos;
	operatorObj->getPosition(operatorPos);

	Vector3d vec(operatorPos.x()-robotPos.x(), operatorPos.y()-robotPos.y(), operatorPos.z()-robotPos.z());

	double distance = sqrt((vec.x()*vec.x())+(vec.z()*vec.z()));

	if(distance>=WAIT_DISTANCE){
		if(follow){
			std::string msg = "NotFollowing";
			sendMsg(operatorName.c_str(),msg);
			follow = false;
			//LOG_MSG((msg.c_str()));
		}
	}
	else{
		if(!follow){
			std::string msg = "Following";
			sendMsg(operatorName.c_str(),msg);
			follow = true;
			//LOG_MSG((msg.c_str()));
		}
	}

	return 0.1;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	/*string msg = evt.getMsg();
	if(msg == "start"){
		task = true;
	}*/
}

void MyController::onCollision(CollisionEvent &evt)
{
}

extern "C" Controller * createController() {
	return new MyController;  
}


