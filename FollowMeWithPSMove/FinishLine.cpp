#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  

char robotName[]    = "robot_004";
char operatorName[] = "operator";

class MyController : public Controller {
public:
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	void onCollision(CollisionEvent &evt); 

private:
	SimObj *m_my;

	// 判定のサイズ
	double checkSize_x, checkSize_z;

	bool sentMsg;
	Vector3d myPos;

	bool flag1; // 1.オペレータが接触
	bool flag2; // 2.ロボットが接触
};

void MyController::onInit(InitEvent &evt) {  
	m_my = getObj(myname());

	// この範囲で判定
	checkSize_x  = 100.0;
	checkSize_z  = 100.0; 

	// 自分の位置取得
	m_my->getPosition(myPos);

	sentMsg = false;
	flag1 = false;
	flag2 = false;
}

double MyController::onAction(ActionEvent &evt) 
{
	Vector3d robotPos;
	Vector3d operatorPos;

	// エンティティ取得
	SimObj *robot_obj    = getObj(robotName);
	SimObj *operator_obj = getObj(operatorName);

	robot_obj->getPosition(robotPos);
	operator_obj->getPosition(operatorPos);

	// operator
	if(fabs(operatorPos.x()-myPos.x()) < checkSize_x/2.0 &&
	   fabs(operatorPos.z()-myPos.z()) < checkSize_z/2.0 &&
	   flag2 == false){
		flag1 = true;
	}

	// robot
	if(fabs(robotPos.x()-myPos.x()) < checkSize_x/2.0 &&
	   fabs(robotPos.z()-myPos.z()) < checkSize_z/2.0 &&
	   flag1 == true){
		flag2 = true;
	}

	if(flag1 && flag2 && !sentMsg){
		LOG_MSG(("crowd_clear"));
		sendMsg("score", "crowd_clear");
		sentMsg = true;
	}

	return 0.1;
}

void MyController::onRecvMsg(RecvMsgEvent &evt) {
	std::string msg = evt.getMsg();
	if (msg == "Task_start"){
		sentMsg = false;
		flag1 = false;
		flag2 = false;
	}
}

void MyController::onCollision(CollisionEvent &evt) { 
}

extern "C" Controller * createController() {  
	return new MyController;  
}

