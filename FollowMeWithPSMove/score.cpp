#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <sstream>
#include <sys/time.h>

using namespace std;

class MyController : public Controller {
public:
	void onInit(InitEvent &evt);
	double onAction(ActionEvent&);
	void onRecvMsg(RecvMsgEvent &evt);
	//void onCollision(CollisionEvent &evt);

private:
	BaseService *m_ref;   // Referee service
	double retValue;      // Refresh rate of the modification

	SimObj *m_my;
	bool check1;
	//bool elevator;
	bool entered;
	bool crowd_clear;
	bool collision;
	bool check1_clear;
	bool elevator_clear;
	bool all_clear;
	//bool not_elevator;
	bool a;
	bool b;
	bool c;
	bool d;
	bool e;
	int total;

	//std::vector<std::string> m_entities;

	struct timeval t0, t1;
};

void MyController::onInit(InitEvent &evt)
{
	m_ref = NULL;
	retValue  = 0.05;

	check1 = false;
	check1_clear = false;
	//elevator = false;
	entered = false;
	elevator_clear = false;
	crowd_clear = false;
	all_clear = false;
	a = false;
	b = false;
	c = false;
	d = false;
	e = false;
	total = 0;

	// ** Need to correctly connect to referee service
	m_my = getObj(myname());

}

double MyController::onAction(ActionEvent &evt)
{
	/*gettimeofday(&t0, NULL);
	int sec, msec;
	if (t0.tv_usec < t1.tv_usec) {
		sec = t0.tv_sec - t1.tv_sec - 1;
		msec= 1000000 + t0.tv_usec - t1.tv_usec;
	}
	else {
		sec = t0.tv_sec - t1.tv_sec;
		msec = t0.tv_usec - t1.tv_usec;
	}
	LOG_MSG(("%d.%d",sec,msec));
	t1 = t0;*/

	// check whether Referee service is available or not
	bool available = checkService("RoboCupReferee");
	if(!available && m_ref != NULL) m_ref = NULL;
	else if(available && m_ref == NULL){
		m_ref = connectToService("RoboCupReferee");
	}

	if (check1_clear == true && a == false && b == false && d == false){
		total = total + 300;
		stringstream ss;
		ss << total;
		string result = ss.str();
		sendMsg("SIGViewer", result);
		std::string msg = "RoboCupReferee/Check point1 clear/300";
		if(m_ref != NULL){
			m_ref->sendMsgToSrv(msg.c_str());
		}
		LOG_MSG((msg.c_str()));
		a = true;
	}

	if (elevator_clear == true && a == true && b == false && d == false){
		total = total + 300;
		stringstream ss2;
		ss2 << total;
		string result2 = ss2.str();
		sendMsg("SIGViewer", result2);
		std::string msg = "RoboCupReferee/Elevator clear" "/300";
		if(m_ref != NULL){
			m_ref->sendMsgToSrv(msg.c_str());
		}
		LOG_MSG((msg.c_str()));
		b = true;
	}
	if (crowd_clear == true && a == true  && b == true && d == false){
		total = total + 300;
		stringstream ss5;
		ss5 << total;
		string result5 = ss5.str();
		sendMsg("SIGViewer", result5);
		std::string msg = "RoboCupReferee/Crowded loacation clear" "/300";
		if(m_ref != NULL){
			m_ref->sendMsgToSrv(msg.c_str());
		}
		LOG_MSG((msg.c_str()));
		d = true;
	}
	if (all_clear == true && a == true  && b == true && d == true && e == false){
		total = total + 100;
		stringstream ss;
		ss << total;
		string result = ss.str();
		sendMsg("SIGViewer", result);
		std::string msg = "RoboCupReferee/All check points clear" "/100";
		if(m_ref != NULL){
			m_ref->sendMsgToSrv(msg.c_str());
		}
		LOG_MSG((msg.c_str()));
		e = true;
	}
	return retValue;
}

void MyController::onRecvMsg(RecvMsgEvent &evt) {
	string msg = evt.getMsg();
	if (msg == "checkpoint1"){
		if(!check1_clear){
			check1 = true;
			LOG_MSG(("checkpoint1"));
		}
	}
	else if (msg == "checkpoint1_clear"){
		if(check1){
			check1_clear = true;
			LOG_MSG(("checkpoint1_clear"));
		}
	}
	else if (msg == "elevator_clear"){
		if(entered){
			elevator_clear = true;
		}
	}
	else if (msg == "Entered"){
		if(!elevator_clear){
			//elevator = true;
			entered = true;
		}
	}
	else if (msg == "crowd_clear"){
		crowd_clear = true;
		if(check1_clear && elevator_clear && crowd_clear){
			all_clear = true;
		}
	}
	/*else if (msg == "finish_line"){
		if(check1_clear && elevator_clear && crowd_clear){
			finish_line = true;
		}
	}*/
	else if(msg == "Task_start"){
		check1 = false;
		check1_clear = false;
		//elevator = false;
		elevator_clear = false;
		crowd_clear = false;
		collision = false;
		all_clear = false;
		a = false;
		b = false;
		c = false;
		d = false;
		e = false;
	}
}

/*void MyController::onCollision(CollisionEvent &evt) {
}*/

extern "C" Controller * createController() {
	return new MyController;
}


