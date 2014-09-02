#include <Controller.h>
#include <ControllerEvent.h>
#include <Logger.h>
#include <ViewImage.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// PSMove API
#include "PSMoveData.h"
#include <psmoveapi/psmove.h>

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )

class UserController : public Controller
{
public:

  void moveBodyByKINECT(char* msg);
  void moveHeadByHMD(const std::string ss);
  void onRecvMsg(RecvMsgEvent &evt);
  void onInit(InitEvent &evt);
  double onAction(ActionEvent &evt);

private:

  //移動速度
  double vel;
  ViewService *m_view;
  BaseService *m_kinect;
  BaseService *m_hmd;
  BaseService *m_wii;

  //初期位置
  double m_posx, m_posy, m_posz;
  double m_yrot;
  double m_range;

  //データ数（関節数）最大値
  int m_maxsize;

  // 体全体の角度
  double m_qw, m_qy, m_qx, m_qz;

  // 前回送信したyaw, pitch roll
  double pyaw, ppitch, proll;

  // ロボットの名前
  std::string robotName;


  // move calibration
  PSMoveArmCalibration arm_calibration[2];
};

void UserController::onInit(InitEvent &evt)
{
  robotName = "robot_000";

  //m_kinect = connectToService("SIGKINECT");
  //m_hmd = connectToService("SIGHMD");
  m_kinect = NULL;
  m_hmd = NULL;
  m_wii = NULL;

  vel      = 10.0;
  srand(time(NULL));

  //初期位置の設定
  SimObj *my = this->getObj(this->myname());
  m_posx = my->x();
  m_posy = my->y();
  m_posz = my->z();
  m_range = 0.1;
  m_maxsize = 15;
  double qw = my->qw();
  double qy = my->qy();
  m_yrot = acos(fabs(qw))*2;
  if(qw*qy > 0)
    m_yrot = -1*m_yrot;

  // 体全体の向き
  m_qw = 1.0;
  m_qx = 0.0;
  m_qy = 0.0;
  m_qz = 0.0;

	// Add by inamura on 28th June
	my->setJointAngle ("RLEG_JOINT2", DEG2RAD(0));
	my->setJointAngle ("LLEG_JOINT2", DEG2RAD(0));
	my->setJointAngle ("RLEG_JOINT4", DEG2RAD(0));
	my->setJointAngle ("LLEG_JOINT4", DEG2RAD(0));

  pyaw = ppitch = proll = 0.0;
}

//定期的に呼び出される関数
double UserController::onAction(ActionEvent &evt)
{

  // サービスが使用可能か定期的にチェックする
  bool av_kinect = checkService("SIGKINECT");
  bool av_hmd = checkService("SIGORS");
  bool av_wii = checkService("Wii_Service");
  sendMsg("PSMoveAPIService", "Bonjour Message");
  std::cout << "message sent to PSMoveAPIService" << std::endl;

  // 使用可能
  if(av_kinect && m_kinect == NULL){
    // サービスに接続
    m_kinect = connectToService("SIGKINECT");

  }
  else if (!av_kinect && m_kinect != NULL){
    m_kinect = NULL;
  }

  // 使用可能
  if(av_hmd && m_hmd == NULL){
    // サービスに接続
    m_hmd = connectToService("SIGORS");

  }
  else if (!av_hmd && m_hmd != NULL){
    m_hmd = NULL;
  }

  // 使用可能
  if(av_wii && m_wii == NULL){
    // サービスに接続
    m_wii = connectToService("Wii_Service");
  }
  else if (!av_wii && m_wii != NULL){
    m_wii = NULL;
  }

  return 1.5;
}

void UserController::onRecvMsg(RecvMsgEvent &evt)
{

  std::string sender = evt.getSender();

  //自分自身の取得
  SimObj *my = getObj(myname());

  //メッセージ取得
  char *all_msg = (char*)evt.getMsg();

  std::string ss = all_msg;
  //ヘッダーの取り出し
  int strPos1 = 0;
  int strPos2;
  std::string headss;
  std::string tmpss;
  strPos2 = ss.find(" ", strPos1);
  headss.assign(ss, strPos1, strPos2-strPos1);


  //std::cout<<ss<<std::endl;

  if(headss == "ORS_DATA"){
    //HMDデータによる頭部の動き反映
    moveHeadByHMD(ss);
  }
  else if(headss == "KINECT_DATA") {
	  //KINECTデータによる頭部以外の体の動き反映
	  moveBodyByKINECT(all_msg);
	  // Add by inamura on 2014-03-02
	  my->setJointAngle ("RLEG_JOINT2", DEG2RAD(0));
	  my->setJointAngle ("LLEG_JOINT2", DEG2RAD(0));
	  my->setJointAngle ("RLEG_JOINT4", DEG2RAD(0));
	  my->setJointAngle ("LLEG_JOINT4", DEG2RAD(0));
	  // Do not collide with a desk
	  if (my->y() < 60)  my->y(60);
  }
  else if(ss == "go") {
    sendMsg("robot_000","go");
    LOG_MSG(("Starting the clean up task"));
    std::cout<<"go"<<std::endl;
  }

  else if(ss == "take" ) {
    sendMsg("robot_000","take");
    LOG_MSG(("Taking the trash"));
    std::cout<<"take"<<std::endl;
  }

  else if(ss == "put" ) {
    sendMsg("robot_000","put");
    LOG_MSG(("Putting the trash in the trash box"));
    std::cout<<"put"<<std::endl;
  }





  else if(ss == "init") {
    sendMsg("robot_000","init");
  }

 else {

    std::vector<MoveData> moves = buildMoveData(all_msg);
    // std::cout << "moves.size() : " << moves.size() << std::endl;
    std::vector<MoveData>::iterator move;
    int i = 0;
    for (move = moves.begin(); move != moves.end(); ++move) {

      if (move->releasedButtons & Btn_MOVE) {
	if (!arm_calibration[move->id].shoulderCalibrated) {
	  arm_calibration[move->id].shoulder = move->tracker;
	  arm_calibration[move->id].shoulderCalibrated = true;
	  std::cout << "Shoulder calibrated for move " << move->id << std::endl;
	}
	else if (!arm_calibration[move->id].handExtensionCalibrated) {
	  arm_calibration[move->id].handExtension = move->tracker;
	  arm_calibration[move->id].handExtensionCalibrated = true;
	  arm_calibration[move->id].armLength = fabs(arm_calibration[move->id].shoulder.z - arm_calibration[move->id].handExtension.z);
	  std::cout << "Arm fully calibrated for move " << move->id << std::endl;
	}
	else {
	  // std::cout << "DEBUG-1-cleanUpUser.cpp" << std::endl;
	  std::stringstream ss;
	  ss << "PSMOVE:psmove_reset_orientation:" << move->id;
	  sendMsg("PSMoveAPIService", ss.str());
	  std::cout << ss.str() << std::endl;
	}
      }
      if (arm_calibration[move->id].shoulderCalibrated and arm_calibration[move->id].handExtensionCalibrated){

	// set position of the object
	double xPosition = 10*move->tracker.x;
	double yPosition = 10*move->tracker.y+90;
	double zPosition = 10*(move->tracker.z - arm_calibration[move->id].shoulder.z);

	Vector3d pos;
	std::stringstream ss;
	ss << "toy_" << move->id;
	SimObj* hand = this->getObj(ss.str().c_str());
	hand->setPosition(xPosition, yPosition, -zPosition);

	// set orientation of the object using quaternions
	hand->setRotation(Rotation(-move->q.z, move->q.y, -move->q.x, move->q.w));

	// std::cout << "DEBUG-2-cleanUpUser.cpp" << "  move->q.x:" << move->q.x << std::endl;
	// std::cout << "DEBUG-3-cleanUpUser.cpp" << "  move->q.y:" << move->q.y << std::endl;
	// std::cout << "DEBUG-4-cleanUpUser.cpp" << "  move->q.z:" << move->q.z << std::endl;
	// std::cout << "DEBUG-5-cleanUpUser.cpp" << "  move->q.w:" << move->q.w << std::endl;

      }
    }

 }
}

void UserController::moveHeadByHMD(const std::string ss){

  //自分自身の取得
  SimObj *my = this->getObj(this->myname());

  //ヘッダーの取り出し
  int strPos1 = 0;
  int strPos2;
  std::string headss;
  std::string tmpss;
  strPos2 = ss.find(" ", strPos1);
  headss.assign(ss, strPos1, strPos2-strPos1);

  if(headss == "ORS_DATA"){
//    LOG_MSG((all_msg));
//  }
//  if(headss == "HMD_DATA"){

    double yaw, pitch, roll;
    strPos1 = strPos2+1;
    tmpss = "";

    strPos2 = ss.find(",", strPos1);
    tmpss.assign(ss, strPos1, strPos2-strPos1);
    yaw = -atof(tmpss.c_str());

    strPos1 = strPos2+1;
    strPos2 = ss.find(",", strPos1);
    tmpss.assign(ss, strPos1, strPos2-strPos1);
    pitch = atof(tmpss.c_str());

    strPos1 = strPos2+1;
    strPos2 = ss.find(",", strPos1);
    tmpss.assign(ss, strPos1, strPos2-strPos1);
    roll = atof(tmpss.c_str());

    if(yaw == pyaw && pitch == ppitch && roll == proll)  return;
    else {
      pyaw = yaw;
      ppitch = pitch;
      proll = roll;
    }

    dQuaternion qyaw;
    dQuaternion qpitch;
    dQuaternion qroll;

    qyaw[0] = cos(-yaw/2.0);
    qyaw[1] = 0.0;
    qyaw[2] = sin(-yaw/2.0);
    qyaw[3] = 0.0;

    qpitch[0] = cos(-pitch/2.0);
    qpitch[1] = sin(-pitch/2.0);
    qpitch[2] = 0.0;
    qpitch[3] = 0.0;

    qroll[0] = cos(-roll/2.0);
    qroll[1] = 0.0;
    qroll[2] = 0.0;
    qroll[3] = sin(-roll/2.0);
    dQuaternion tmpQ1;
    dQuaternion tmpQ2;

    dQMultiply0(tmpQ1, qyaw, qpitch);
    dQMultiply0(tmpQ2, tmpQ1, qroll);

    dQuaternion bodyQ;
    bodyQ[0] = m_qw;
    bodyQ[1] = m_qx;
    bodyQ[2] = m_qy;
    bodyQ[3] = m_qz;

    dQuaternion tmpQ3;
    dQMultiply1(tmpQ3, bodyQ, tmpQ2);

    my->setJointQuaternion("HEAD_JOINT0", tmpQ3[0], tmpQ3[1], tmpQ3[2], tmpQ3[3]);
  }
}


void UserController::moveBodyByKINECT(char* all_msg){

  //自分自身の取得
  SimObj *my = this->getObj(this->myname());
  char* msg = strtok(all_msg," ");
  if(strcmp(msg,"KINECT_DATA") == 0){
    int i = 0;
    while(true){
      i++;
      if(i == m_maxsize+1) break;
      char *type = strtok(NULL,":");
      if(strcmp(type,"POSITION") == 0){
	//体の位置
	double x = atof(strtok(NULL,","));
	double y = atof(strtok(NULL,","));
	double z = atof(strtok(NULL," "));
	//エージェント座標からグローバル座標への変換
	double gx = cos(m_yrot)*x - sin(m_yrot)*z;
	double gz = sin(m_yrot)*x + cos(m_yrot)*z;
	my->setPosition(m_posx+gx,m_posy+y,m_posz+gz);
	//limitedSetPosition(my, m_posx+gx,m_posy+y,m_posz+gz);
	continue;
      }else if(strcmp(type,"WAIST") == 0){
	//体全体の回転

	double w = atof(strtok(NULL,","));
	double x = atof(strtok(NULL,","));
	double y = atof(strtok(NULL,","));
	double z = atof(strtok(NULL," "));
	my->setJointQuaternion("ROOT_JOINT0",w, x, y, z);
	m_qw = w;
	m_qx = x;
	m_qy = y;
	m_qz = z;
	continue;
      }else if(strcmp(type,"END") == 0){
	break;
      }else{
	//関節の回転
	double w = atof(strtok(NULL,","));
	double x = atof(strtok(NULL,","));
	double y = atof(strtok(NULL,","));
	double z = atof(strtok(NULL," "));
	double angle = acos(w)*2;
	double tmp = sin(angle/2);
	double vx = x/tmp;
	double vy = y/tmp;
	double vz = z/tmp;
	double len = sqrt(vx*vx+vy*vy+vz*vz);
	if(len < (1 - m_range) || (1 + m_range) < len) continue;
	// HEAD_JOINT1はHMDにより回転
	if(strcmp(type,"HEAD_JOINT1") != 0 ){
	  my->setJointQuaternion(type,w,x,y,z);
	}
	continue;
      }
    }
  }

}
extern "C" Controller * createController ()
{
  return new UserController;
}
