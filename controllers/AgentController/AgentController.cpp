#include "sigverse/commonlib/Controller.h"
#include "sigverse/commonlib/Logger.h"
#include <math.h>

//Coverts angles to radians
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class AgentController : public Controller
{
public:
	double onAction(ActionEvent &evt);
}; 


//This function is called on regular basis
double AgentController::onAction(ActionEvent &evt)
{
	try {
		// get handle for the agent
		SimObj *my = getObj(myname());
		if (!my->dynamics()) {
			
			// "LARM_JOINT2" is set to 45 degrees
			my->setJointAngle("LARM_JOINT2", DEG2RAD(45));
		}
	}
	catch(SimObj::Exception &) {
	}
	return 5.0;
}


extern "C"  Controller * createController ()
{
	return new AgentController;
}
