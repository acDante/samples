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

double MyController::onAction(ActionEvent &evt) {
  return 1.0;
}

void MyController::onRecvMsg(RecvMsgEvent &evt) {

  //取得したメッセージを表示します
  string msg = evt.getMsg();
  //LOG_MSG(("msg : %s", msg.c_str()));
  SimObj *my = getObj(myname());

  if(strstr(msg.c_str()," "))
    {
      // phi theta に分ける 
      int n = 0;
      n = msg.find(" ");
      string phi_s = msg.substr(0,n);
      string theta_s = msg.substr(n+1);

      double phi = atof(theta_s.c_str());
      double theta = atof(phi_s.c_str());

      // 目玉関節を回転させる
      my->setJointAngle("LEYE_JOINT1",DEG2RAD(phi));
      my->setJointAngle("REYE_JOINT1",DEG2RAD(phi));
      my->setJointAngle("LEYE_JOINT0",DEG2RAD(theta));
      my->setJointAngle("REYE_JOINT0",DEG2RAD(theta));
    }
}
void MyController::onCollision(CollisionEvent &evt) {
}

extern "C" Controller * createController() {
  return new MyController;
}
