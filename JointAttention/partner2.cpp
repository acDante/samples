#include <math.h>

#include "sigverse/commonlib/ControllerEvent.h"
#include "sigverse/commonlib/Controller.h"
#include "sigverse/commonlib/Logger.h"

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

using namespace std;

class MyController : public Controller {
public:
  void onInit(InitEvent &evt);
  double onAction(ActionEvent&);
  void onRecvMsg(RecvMsgEvent &evt);
  void onCollision(CollisionEvent &evt);
};

void MyController::onInit(InitEvent &evt) {
}

double MyController::onAction(ActionEvent &evt)
{
	Vector3d pos;
	SimObj *user    = getObj("man_000");
	SimObj *partner = getObj(myname());
	if(!user->getJointPosition(pos, "RARM_JOINT4"))
		LOG_ERR(("failed to get joint position"));

	double theta = (93-pos.y()) / 6.0 * 20 + 15;
	double phi   = (pos.x()-17) / 7.5 * 20;

	// 目玉関節を回転させる
	partner->setJointAngle("LEYE_JOINT1",DEG2RAD(phi));
	partner->setJointAngle("REYE_JOINT1",DEG2RAD(phi));
	partner->setJointAngle("LEYE_JOINT0",DEG2RAD(theta));
	partner->setJointAngle("REYE_JOINT0",DEG2RAD(theta));

	return 0.15;
}


void MyController::onRecvMsg(RecvMsgEvent &evt)
{
}


void MyController::onCollision(CollisionEvent &evt) {
}

extern "C" Controller * createController() {
  return new MyController;
}
