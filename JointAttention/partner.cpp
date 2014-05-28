#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <math.h>

//#define M_PI 3.141592
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

  //����������å�������ɽ�����ޤ�
  string msg = evt.getMsg();
  //LOG_MSG(("msg : %s", msg.c_str()));
  SimObj *my = getObj(myname());

  if(strstr(msg.c_str()," "))
    {
      // phi theta ��ʬ���� 
      int n = 0;
      n = msg.find(" ");
      string phi_s = msg.substr(0,n);
      string theta_s = msg.substr(n+1);

      double phi = atof(theta_s.c_str());
      double theta = atof(phi_s.c_str());

      // �̴ܶ�����ž������
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
