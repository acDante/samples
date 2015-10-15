#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"
#include <unistd.h>
#include <algorithm>

#define PI 3.1415926535

//角度からラジアンに変換します
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )   

bool start;
bool sw;
using namespace std;

class MyController : public Controller {
public:
	void onInit(InitEvent &evt);
	double onAction(ActionEvent&);
	void onRecvMsg(RecvMsgEvent &evt);
	void onCollision(CollisionEvent &evt);

private:
	SimObj *my;

	FILE* fp;
	float stepWidth;
	int sleeptime;
	const static int SIZE = 30;
	int motionNum;
	float HEIGHT[SIZE];
	float LARM_JOINT1[SIZE]; // left shoulder
	float LARM_JOINT3[SIZE]; // left elbow
	float RARM_JOINT1[SIZE]; // right shoulder
	float RARM_JOINT3[SIZE]; // right elbow
	float LLEG_JOINT2[SIZE]; // left groin(leg)
	float LLEG_JOINT4[SIZE]; // left knee
	float LLEG_JOINT6[SIZE]; // left ankle
	float RLEG_JOINT2[SIZE]; // right groin
	float RLEG_JOINT4[SIZE]; // right knee
	float RLEG_JOINT6[SIZE]; // right ankle

	int count;
	int step;

	Vector3d initPos;

};

void MyController::onInit(InitEvent &evt)
{
	my = getObj(myname());
	my->getPosition(initPos);

	start = false;
	sw = false;

	// 手を下げる
	my->setJointAngle("LARM_JOINT2", DEG2RAD(-90));
	my->setJointAngle("RARM_JOINT2", DEG2RAD(90));

	stepWidth = 70;
	sleeptime = 300000;

	if((fp = fopen("motion.txt", "r")) == NULL) {
		LOG_MSG(("File do not exist."));
	}
	else{
		fscanf(fp, "%d", &motionNum);
		fscanf(fp, "%d", &sleeptime);
		for(int i=0; i<motionNum; i++){
			fscanf(fp, "%f %f %f %f %f %f %f %f %f %f %f",
					   &HEIGHT[i],
					   &LARM_JOINT1[i],
					   &LARM_JOINT3[i],
					   &RARM_JOINT1[i],
					   &RARM_JOINT3[i],
					   &LLEG_JOINT2[i],
					   &LLEG_JOINT4[i],
					   &LLEG_JOINT6[i],
					   &RLEG_JOINT2[i],
					   &RLEG_JOINT4[i],
					   &RLEG_JOINT6[i]);
		}
	}

	count = 0;
	step = 6;
}

double MyController::onAction(ActionEvent &evt)
{
	Vector3d pos;

	if (start == true){

		my->getPosition(pos);

		double dx = 0;
		double dz = -2.5;
		double addx = 0.0;
		double addz = 0.0;

		if(count%2){
			for(int i=0; i<motionNum; i++){
				addx += dx;
				addz += dz;
				if(motionNum)
					usleep(sleeptime);
				my->setPosition(pos.x()+addx, HEIGHT[i], pos.z()+addz);
				my->setJointAngle("LARM_JOINT1", DEG2RAD(LARM_JOINT1[i]));
				my->setJointAngle("LARM_JOINT3", DEG2RAD(LARM_JOINT3[i]));
				my->setJointAngle("RARM_JOINT1", DEG2RAD(RARM_JOINT1[i]));
				my->setJointAngle("RARM_JOINT3", DEG2RAD(RARM_JOINT3[i]));
				my->setJointAngle("LLEG_JOINT2", DEG2RAD(LLEG_JOINT2[i]));
				my->setJointAngle("LLEG_JOINT4", DEG2RAD(LLEG_JOINT4[i]));
				my->setJointAngle("LLEG_JOINT6", DEG2RAD(LLEG_JOINT6[i]));
				my->setJointAngle("RLEG_JOINT2", DEG2RAD(RLEG_JOINT2[i]));
				my->setJointAngle("RLEG_JOINT4", DEG2RAD(RLEG_JOINT4[i]));
				my->setJointAngle("RLEG_JOINT6", DEG2RAD(RLEG_JOINT6[i]));
			}
		}
		else{
			for(int i=0; i<motionNum; i++){
				addx += dx;
				addz += dz;
				usleep(sleeptime);
				my->setPosition(pos.x()+addx, HEIGHT[i], pos.z()+addz);
				my->setJointAngle("RARM_JOINT1", DEG2RAD(LARM_JOINT1[i]));
				my->setJointAngle("RARM_JOINT3", DEG2RAD(-LARM_JOINT3[i]));
				my->setJointAngle("LARM_JOINT1", DEG2RAD(RARM_JOINT1[i]));
				my->setJointAngle("LARM_JOINT3", DEG2RAD(-RARM_JOINT3[i]));
				my->setJointAngle("RLEG_JOINT2", DEG2RAD(LLEG_JOINT2[i]));
				my->setJointAngle("RLEG_JOINT4", DEG2RAD(LLEG_JOINT4[i]));
				my->setJointAngle("RLEG_JOINT6", DEG2RAD(LLEG_JOINT6[i]));
				my->setJointAngle("LLEG_JOINT2", DEG2RAD(RLEG_JOINT2[i]));
				my->setJointAngle("LLEG_JOINT4", DEG2RAD(RLEG_JOINT4[i]));
				my->setJointAngle("LLEG_JOINT6", DEG2RAD(RLEG_JOINT6[i]));
			}
		}
		count++;

		if(count==2){
			sleep(3);
		}
		if(count==4){
			sendMsg("operator","Passed_through");
		}
		if(count>step){
			start = false;
		}
	}

	return 0.1;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	string msg = evt.getMsg();
	if(msg == "walk"){
		start = true;
	}
	else if(msg == "Task_start"){
		start = false;
		my->setPosition(initPos);
		count = 0;
	}
	else if(msg == "Give_up"){
		start = false;
	}

}

void MyController::onCollision(CollisionEvent &evt)
{
}

extern "C" Controller * createController() {
	return new MyController;
}

