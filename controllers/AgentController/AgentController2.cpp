#include "sigverse/commonlib/Controller.h"
#include "sigverse/commonlib/Logger.h"
#include "sigverse/commonlib/ControllerEvent.h"

#include <string>
#include <math.h>

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

using namespace std;  //Using namespace definitions

class AgentController : public Controller
{
public:
	double onAction(ActionEvent &evt);

	// Declare the use of onRecvMessage function when it receives a message
	void onRecvMessage(RecvMessageEvent &evt);
};

double AgentController::onAction(ActionEvent &evt)
{
	try {
		SimObj *my = getObj(myname());
		if (!my->dynamics()) { 

			// Lower down the left.
			my->setJointAngle("LARM_JOINT2", DEG2RAD(-90));

			// Lower down the right
			my->setJointAngle("RARM_JOINT2", DEG2RAD(90));

		}
	}
	catch(SimObj::Exception &) {
	}
	return 5.0;
}


void AgentController::onRecvMessage(RecvMessageEvent &evt)
{
	//get an handle to agent
	SimObj *my = getObj(myname());

	//string message is received 
	string value(evt.getString(0)); 
	if (value =="Hello") {

		//bend the hip joint to 45[deg]
		my->setJointAngle("WAIST_JOINT1", DEG2RAD(45));
	}
}

extern "C"  Controller * createController ()
{
	return new AgentController;
}
