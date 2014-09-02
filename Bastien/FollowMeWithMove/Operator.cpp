#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>
//#include <string> 
#include <sstream>
#include <iostream>
#include <math.h>
#include <unistd.h>

// Convert degree to radian
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

#define WAIT_PERSON       0.5 //人が歩き始める場所
#define WALKING_PERSON    1.0 //人が歩き始める場所
#define CHECK_POINT1      2.0 //1st Sectionの点数が入る場所
#define FRONT_OF_ELEVATOR 2.5 //エレベータ内の待機場所
#define ELEVATOR          3.0 //エレベータ内の待機場所
#define ELEVATOR_CLEAR    4.0 //2nd Sectionの点数が入る場所
#define CROWD             5.0 //人ごみを抜けた後の待機場所
#define CROWD_END         6.0 //3rd Sectionの点数が入る場所
#define END              99.0 //終了地点

struct coordinate{
	double x[255];
	double y[255];
	double z[255];
	double flag[255];
};

char robotName[] = "robot_004";

using namespace std;

class MyController : public Controller {  
public:
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	//void onCollision(CollisionEvent &evt); 

	double getDistanceToRobot();

	void resetPosition();

private:
	SimObj *my;
	SimObj *robot;

	std::vector<std::string> m_entities;

	coordinate node; 

	//bool first;
	bool start;
	bool passed;
	bool elevator;
	bool crowd;
	bool end;
	bool stop;
	bool doorClose;
	bool getoff;
	bool elevator_end;
	bool waitForHuman;
	bool waitForCrowd;
	bool waitForElevator;

	bool sentMsg_Man;
	bool sendMsg_CheckPoint1;
	bool sentMsg_Elevator;
	bool sentMsg_CheckPoint2;
	bool sentMsg_Crowd;
	bool sentMsg_Finishline;

	double dx,dy,dz;
	int i; 

	//bool follow;
	bool walking;

	//int trialCount;

	bool resetPos;

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
	float RLEG_JOINT2[SIZE]; // right groin(leg)
	float RLEG_JOINT4[SIZE]; // right knee
	float RLEG_JOINT6[SIZE]; // right ankle

	int count;
	int step;

	int taskNum;

	double interval;

	void initCondition();
};

void MyController::onInit(InitEvent &evt) 
{
	//first = true;
	taskNum = 0;

	if((fp = fopen("tasknum.txt", "r")) == NULL) {
		printf("File do not exist:trial.txt\n");
		exit(0);
	}
	else{
		fscanf(fp, "%d", &taskNum);
		LOG_MSG(("Set taskNum: %d",taskNum));
		fclose(fp);
	}

	my = getObj(myname());
	robot = getObj(robotName);

	// Put arms down
	my->setJointAngle("LARM_JOINT2", DEG2RAD(-90));
	my->setJointAngle("RARM_JOINT2", DEG2RAD(90));

	stepWidth = 45;

	if((fp = fopen("motion.txt", "r")) == NULL) {
		printf("File do not exist.\n");
		exit(0);
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
		fclose(fp);
	}

	interval = sleeptime / 1000000;

}

void MyController::initCondition()
{
	start = false;
	passed = false;
	elevator = false;
	crowd = false;
	end = false;
	stop = true;
	doorClose = false;
	getoff=false;
	elevator_end = false;
	waitForHuman = false;
	waitForCrowd = false;
	waitForElevator = false;
	
	resetPos = false;

	//follow = true;
	walking = false;

	sentMsg_Man = false;
	sendMsg_CheckPoint1 = false;
	sentMsg_Elevator = false;
	sentMsg_CheckPoint2 = false;
	sentMsg_Crowd = false;
	sentMsg_Finishline = false;

	count = 0;
	step = 0;

	double x=0;
	double y=0;
	double z=0;
	double flag=0; //Check point

	dx=0;
	dy=0;
	dz=0;

	for(int k=0; k<256; k++){
		node.x[k] = 0.0;
		node.y[k] = 0.0;
		node.z[k] = 0.0;
		node.flag[k] = 0.0;
	}

	std::stringstream nodePath;
	nodePath << "nodes/node_" << (int)(taskNum++/2) << ".txt";

	//std::cout << "[HUMAN]" << " load file : " << nodePath.str() << std::endl;
	LOG_MSG(("[HUMAN] load file: %s", nodePath.str().c_str()));

	if((fp = fopen(nodePath.str().c_str(), "r")) == NULL) {
		LOG_MSG(("File do not exist."));
		exit(0);
	}

	i=0;

	while(fscanf(fp, "%lf,%lf,%lf,%lf", &x,&y,&z,&flag) != EOF) {
		node.x[i]=x;
		node.y[i]=y;
		node.z[i]=z;
		node.flag[i]=flag;
		i++;
	}

	fclose(fp);
	i=0;

	if(taskNum%2){
		stepWidth = 45;
	}
	else{
		stepWidth = 60;
	}

}

void MyController::resetPosition()
{
	//std::cout << "x:" << node.x[0] << " z:" << node.z[0] << std::endl;
	my->setPosition(node.x[0], 60.0, node.z[0]);
	
	if(node.flag[0] == -1){ // left
		double rad = 0;
		Rotation rot(cos(0), 0.0, -sin(0), 0.0);
		my->setRotation(rot);
		robot->setPosition(node.x[0], 30.0, node.z[0]-100);
		robot->setRotation(rot);
	}
	else if(node.flag[0] == -2){ // forward
		double rad = M_PI/4;
		Rotation rot(cos(rad), 0.0, -sin(rad), 0.0);
		my->setRotation(rot);
		robot->setPosition(node.x[0]+100, 30.0, node.z[0]);
		robot->setRotation(rot);
	}
	else if(node.flag[0] == -3){ // right
		double rad = M_PI/2;
		Rotation rot(cos(rad), 0.0, -sin(rad), 0.0);
		my->setRotation(rot);
		robot->setPosition(node.x[0], 30.0, node.z[0]+100);
		robot->setRotation(rot);
	}
	else if(node.flag[0] == -4){ // backward
		double rad = M_PI;
		Rotation rot(cos(rad), 0.0, -sin(rad), 0.0);
		my->setRotation(rot);
		robot->setPosition(node.x[0]-100, 30.0, node.z[0]);
		robot->setRotation(rot);
	}
	
	i=1;
}

double MyController::onAction(ActionEvent &evt)
{
	//LOG_MSG(("on action"));
	if(!end && start){
		
		Vector3d pos;
		double angle;

		my->getPosition(pos);

		// 歩いていないとき(node上にいるとき)
		if(!walking){
			dx=(node.x[i]-pos.x());
			dz=(node.z[i]-pos.z());
			angle = atan2(dx,dz);
			
			my->setAxisAndAngle(0,1.0, 0, angle);
			
			double r = sqrt(pow(dx,2)+pow(dz,2));
			
			step = 2 * r / stepWidth;
			
			dx /= step*motionNum;
			dz /= step*motionNum;

			walking = true;
		}

		// 歩く
		if(!stop){
			double addx = 0.0;
			double addz = 0.0;
			
			if(count%2){
				for(int j=0; j<motionNum; j++){
					addx += dx;
					addz += dz;
					usleep(sleeptime);
					my->setPosition(pos.x()+addx, HEIGHT[i], pos.z()+addz);
					my->setJointAngle("LARM_JOINT1", DEG2RAD(LARM_JOINT1[j]));
					my->setJointAngle("LARM_JOINT3", DEG2RAD(LARM_JOINT3[j]));
					my->setJointAngle("RARM_JOINT1", DEG2RAD(RARM_JOINT1[j]));
					my->setJointAngle("RARM_JOINT3", DEG2RAD(RARM_JOINT3[j]));
					my->setJointAngle("LLEG_JOINT2", DEG2RAD(LLEG_JOINT2[j]));
					my->setJointAngle("LLEG_JOINT4", DEG2RAD(LLEG_JOINT4[j]));
					my->setJointAngle("LLEG_JOINT6", DEG2RAD(LLEG_JOINT6[j]));
					my->setJointAngle("RLEG_JOINT2", DEG2RAD(RLEG_JOINT2[j]));
					my->setJointAngle("RLEG_JOINT4", DEG2RAD(RLEG_JOINT4[j]));
					my->setJointAngle("RLEG_JOINT6", DEG2RAD(RLEG_JOINT6[j]));
				}
			}
			else{
				for(int j=0; j<motionNum; j++){
					addx += dx;
					addz += dz;
					usleep(sleeptime);
					my->setPosition(pos.x()+addx, HEIGHT[i], pos.z()+addz);
					my->setJointAngle("RARM_JOINT1", DEG2RAD(LARM_JOINT1[j]));
					my->setJointAngle("RARM_JOINT3", DEG2RAD(-LARM_JOINT3[j]));
					my->setJointAngle("LARM_JOINT1", DEG2RAD(RARM_JOINT1[j]));
					my->setJointAngle("LARM_JOINT3", DEG2RAD(-RARM_JOINT3[j]));
					my->setJointAngle("RLEG_JOINT2", DEG2RAD(LLEG_JOINT2[j]));
					my->setJointAngle("RLEG_JOINT4", DEG2RAD(LLEG_JOINT4[j]));
					my->setJointAngle("RLEG_JOINT6", DEG2RAD(LLEG_JOINT6[j]));
					my->setJointAngle("LLEG_JOINT2", DEG2RAD(RLEG_JOINT2[j]));
					my->setJointAngle("LLEG_JOINT4", DEG2RAD(RLEG_JOINT4[j]));
					my->setJointAngle("LLEG_JOINT6", DEG2RAD(RLEG_JOINT6[j]));
				}
			}
			count++;

			// 目標点に到着
			if(step==count){
				count = 0;
				step = 0;
				walking = false;
				i++;
			}
		}

		double checkPoint = node.flag[i-1];

		/*// 人が歩き始める前に待機
		if(checkPoint == WAIT_PERSON && !waitForHuman){
			stop = true;
			if(getDistanceToRobot() < 150){
				stop = false;
				waitForHuman = true;
			}
		}*/
		// チェックポイント1
		/*else if(checkPoint == CHECK_POINT1 && !sendMsg_CheckPoint1){
			//sendMsg("score","checkpoint1_clear");
			//LOG_MSG(("check point1 clear"));
			sendMsg_CheckPoint1 = true;
		}*/

		// 人が歩き始める
		if(checkPoint == WALKING_PERSON && !sentMsg_Man){
			//sendMsg("man_001", "point1");
			
			//LOG_MSG(("walk"));
			stop = true;
			if(getDistanceToRobot() < 200){
				sendMsg("man_001", "walk");
				sentMsg_Man = true;
			}
		}

		// エレベータ前待機
		else if(checkPoint == FRONT_OF_ELEVATOR && !waitForElevator){
			stop = true;
			if(getDistanceToRobot() < 200){
				stop = false;
				waitForElevator = true;
			}
		}
		// エレベータ奥
		if(checkPoint == ELEVATOR && !elevator){
			if(!getoff){
				stop = true;
				elevator = true;
			}
		}
		// ロボットが退出したらエレベータから出る
		if(getoff && !elevator_end){
			elevator = false;
			stop = false;
			elevator_end = true;
		}
		// エレベータクリア
		if(checkPoint==ELEVATOR_CLEAR && !sentMsg_CheckPoint2){
			getoff = false;
			//sendMsg("score","elevator_clear");
			//LOG_MSG(("elevator_clear"));
			sentMsg_CheckPoint2 = true;
		}
		// 人ごみを通過後待機
		else if(checkPoint == CROWD && !waitForCrowd){
			stop = true;
			if(getDistanceToRobot() < 200){
				stop = false;
				waitForCrowd = true;
			}
		}
		// 人ごみを通過
		else if(checkPoint == CROWD_END && !sentMsg_Crowd){
			//std::string msg5 = "crowd";
			//sendMsg("score", msg5);
			//LOG_MSG(("crowd"));
			sentMsg_Crowd = true;
		}

		// 終了点に着いた
		//if(checkPoint==END && follow){
		if(checkPoint==END){
			end = true;
		}
	}

	return interval;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	string msg = evt.getMsg();
	//LOG_MSG(("get: %s", msg.c_str()));

	// タスク開始
	if(msg == "Task_start"){
		count = 0;
		//initCondition();
		start = true;
		stop = false;
		//trialCount++;
	}
	else if(msg == "Reset_position"){
		LOG_MSG(("Reset"));
		initCondition();
		//if(!resetPos){
			//initCondition();
			resetPosition();
			//resetPos = true;
		//}
	}
	else if(msg == "Passed_through" && sentMsg_Man){
		passed = true;
		stop = false;
	}
	else if(msg == "Door_close" && elevator && doorClose && !sentMsg_Elevator){
		//sendMsg("wall_008", "elevator_close");
		//LOG_MSG(("elevator close"));
		sentMsg_Elevator = true;
	}
	// ロボットがエレベータに入り終わった
	else if(msg == "entered"){
		LOG_MSG(("entered"));
		doorClose = true;
	}
	// ロボットがエレベータから退出した
	else if(msg == "Get_off"){
		getoff = true;
		//sendMsg("score","elevator_clear");
	}
	if(msg == "Give_up"){
		//start = false;
		end = true;
		//LOG_MSG(("get: give up"));
	}
	if(msg == "Task_end"){
		//LOG_MSG(("get: task end"));
		//sendMsg("Moderator_0","Get_end_msg");
		stop = true;
		end = true;
	}
}

/*void MyController::onCollision(CollisionEvent &evt)
{
}*/

double MyController::getDistanceToRobot()
{
	SimObj *robotObj = getObj(robotName);

	Vector3d robotPos;
	robotObj->getPosition(robotPos);

	// 位置取得
	Vector3d operatorPos;
	my->getPosition(operatorPos);

	Vector3d vec(operatorPos.x()-robotPos.x(), operatorPos.y()-robotPos.y(), operatorPos.z()-robotPos.z());

	return sqrt((vec.x()*vec.x())+(vec.z()*vec.z()));
}

extern "C" Controller * createController() {
	return new MyController;  
}


