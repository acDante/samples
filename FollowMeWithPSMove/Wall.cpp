#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <unistd.h>
#include <algorithm>

class MyController : public Controller
{
public:
	void onRecvMsg(RecvMsgEvent &evt);  
};

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	SimObj *obj = getObj(myname());
	Vector3d pos;
	std::string msg = evt.getMsg();
	if(msg == "Door_close"){
		obj->getPosition(pos);
		obj->setPosition( pos.x(), pos.y() , pos.z()+150);
		sleep(5);
		obj->setPosition( pos.x(), pos.y() , pos.z());
		//sendMsg("operator","elevator_open");
	}
}

extern "C" Controller * createController() {  
	return new MyController;  
}
