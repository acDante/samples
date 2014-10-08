#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <algorithm>
#include <unistd.h>
#include <math.h>

//convert angle unit from degree to radian
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )
#define error_angle_arm 0.2
std::string joints_Names[] = {"LARM_JOINT0","LARM_JOINT1","LARM_JOINT2","LARM_JOINT3","LARM_JOINT4","LARM_JOINT5","LARM_JOINT6","RARM_JOINT0",
  "RARM_JOINT1","RARM_JOINT2","RARM_JOINT3","RARM_JOINT4","RARM_JOINT5","RARM_JOINT6","LLEG_JOINT0","LLEG_JOINT1","LLEG_JOINT2","LLEG_JOINT3","LLEG_JOINT4",
  "LLEG_JOINT5","RLEG_JOINT0","RLEG_JOINT1","RLEG_JOINT2","RLEG_JOINT3","RLEG_JOINT4","RLEG_JOINT5","ROOT_JOINT0","ROOT_JOINT1","ROOT_JOINT2","NECK_JOINT0",
  "NECK_JOINT1","NECK_JOINT2","LEYE_JOINT0","LEYE_JOINT1","REYE_JOINT0","REYE_JOINT1","LHAND_THUMB_BASE","LHAND_THUMB_0","LHAND_THUMB_1","LHAND_THUMB_2",
  "LHAND_FINGER4_BASE","LHAND_FINGER4_0","LHAND_FINGER4_1","LHAND_FINGER4_2","LHAND_FINGER3_BASE","LHAND_FINGER3_0","LHAND_FINGER3_1","LHAND_FINGER3_2",
  "LHAND_FINGER2_BASE","LHAND_FINGER2_0","LHAND_FINGER2_1","LHAND_FINGER2_2","LHAND_FINGER1_BASE","LHAND_FINGER1_0","LHAND_FINGER1_1","LHAND_FINGER1_2",
  "RHAND_THUMB_BASE","RHAND_THUMB_0","RHAND_THUMB_1","RHAND_THUMB_2","RHAND_FINGER4_BASE","RHAND_FINGER4_0","RHAND_FINGER4_1","RHAND_FINGER4_2",
  "RHAND_FINGER3_BASE","RHAND_FINGER3_0","RHAND_FINGER3_1","RHAND_FINGER3_2","RHAND_FINGER2_BASE","RHAND_FINGER2_0","RHAND_FINGER2_1","RHAND_FINGER2_2",
  "RHAND_FINGER1_BASE","RHAND_FINGER1_0","RHAND_FINGER1_1","RHAND_FINGER1_2"
};
//ControllerのサブクラスMoveControllerの宣言します
class RobotController : public Controller {
public:

  void onInit(InitEvent &evt);
  double onAction(ActionEvent&);
  void onRecvMsg(RecvMsgEvent &evt);
  bool moveBody(std::vector <double  > joints_Val);
  void Capture(std::string number);
private:
  SimObj *m_my;
 std::vector< std::vector <double> >  joints_Values;
int m_state;
int i;
};

void RobotController::onInit(InitEvent &evt)
{
  m_state = 20;
  i = 0;
    m_my = getObj(myname());
 // m_my->setJointVelocity("RLEG_JOINT3", -2, 0.0);
  m_my->setJointAngle("LARM_JOINT2",0.57);
  m_my->setJointAngle("RARM_JOINT2",-0.57);
  //m_my->setJointAngle("RARM_JOINT2",-1.57);
  //m_my->setJointAngle("RARM_JOINT3",-1.57);
 // m_my->setJointVelocity("LARM_JOINT1", -2, 0.0);
 // m_my->setJointVelocity("RARM_JOINT1", -2, 0.0);
 // m_my->setJointVelocity("ROOT_JOINT2", -4, 0.0);
}

double RobotController::onAction(ActionEvent &evt)
{
switch(m_state)
    {
    case 0:   { 
          i = 0;
          if(joints_Values.size() != 0) 
        {
          std::cout << "the Sequence Size is  "<< joints_Values.size() << std::endl ;
          m_state = 1;
         } 
        else 
        {
        m_state = 20;
        }
          break;
        }
  case 1:   { //preparation of the arm for grasp
          if (moveBody(joints_Values[i]) == true) m_state = 2;

          break;
        }
   case 2:   { 
          i= i+1;
        std::cout << "the Sequence Number is  "<< i << std::endl ;
       if(i < joints_Values.size()) 
        {
        m_state = 1;
        }
        else 
        {
        m_state = 20;
        }
          break;
        }       
}
return 0.1;
}

void RobotController::onRecvMsg(RecvMsgEvent &evt)
{

std::string sender = evt.getSender();
 std::string msg    = evt.getMsg();
  // 送信者がゴミ認識サービスの場合
//  char *all_msg = (char*)evt.getMsg()
 std::cout << "the Message is "+ msg << std::endl ;
 std::string task;
  std::string joint;
 std::string value;
  LOG_MSG(("%s: %s",sender.c_str(), msg.c_str()));
  std::size_t found=0;
  std::size_t found2=0;
  std::size_t found3= msg.size();  
found = msg.find(":",found2);
    if (found != std::string::npos){
      joint = msg.substr(found2,found-found2);
   
value = msg.substr(found+1,found3);
 std::cout << "the the value  is "+ value << std::endl ;
int val = atoi(value.c_str());
double angle = (val*3.14) / 180;
m_my->setJointAngle(joint.c_str(),angle);
 }

found = msg.find("!",found2);
    if (found != std::string::npos){
      joint = msg.substr(found2,found-found2);
   
value = msg.substr(found+1,found3);
 std::cout << "the the value  is "+ value << std::endl ;
  Capture(value);
 }

found = msg.find("#",found2);
    if (found != std::string::npos){
      joint = msg.substr(found2,found-found2);
   
value = msg.substr(found+1,found3);
 std::cout << "the the value  is "+ value << std::endl ;
  m_state = 0;
 }



 found = msg.find("X=",found2);
    if (found != std::string::npos){
      joint = msg.substr(found2,found-found2);
   
value = msg.substr(found+2,found3);
 std::cout << "the the value  is "+ value << std::endl ;
 int val2 = atoi(value.c_str());
 Vector3d point;
  m_my->getPosition(point);
  m_my->setPosition(val2,point.y(),point.z());
 }

found = msg.find("Y=",found2);
    if (found != std::string::npos){
      joint = msg.substr(found2,found-found2);
   
value = msg.substr(found+2,found3);
 std::cout << "the the value  is "+ value << std::endl ;
 int val2 = atoi(value.c_str());
 Vector3d point;
  m_my->getPosition(point);
  m_my->setPosition(point.x(),point.y(),val2);
 }
found = msg.find("??",found2);
    if (found != std::string::npos){
      joint = msg.substr(found2,found-found2);
   
value = msg.substr(found+2,found3);
 //std::cout << "the the value  is "+ value << std::endl ;

for (int j=0; j<joints_Values.size(); j++)
{
for (int i=0; i<76; i++)
 {

 std::cout << " I  ..."<< j << "   J ..." <<  i << "  the the value  is   ... " << joints_Values[j][i] << std::endl ;

 }

  }
}

}


void RobotController::Capture( std::string number)
{

double  joints_current[76]; 
std::vector <double> Vec_value;
for (int j=0; j<76; j++)
 {
joints_current[j] = m_my->getJointAngle(joints_Names[j].c_str());
Vec_value.push_back(joints_current[j]);
std::cout << joints_Names[j] << " value is  " << joints_current[j] << "  ... "  << std::endl ;
 }
std::cout << "Capture  .....  " << number << "    done  "  << std::endl ;
joints_Values.push_back(Vec_value);
}


bool RobotController::moveBody(std::vector <double > joints_Val)
{
  //bool j0 = false, j1 = false , j3 = false, j4 = false, j5 = false, j6 = false, j7 = false;
  bool Joint_State[76];
  double joints_current[76];
  bool global_status = false; 
for (int j=0; j<76; j++)
 {
Joint_State[j]= false;
joints_current[j] = m_my->getJointAngle(joints_Names[j].c_str());
 }


 for (int i=0; i<76; i++)

 {
  if(joints_current[i] != joints_Val[i] )
  {
    if(joints_current[i] < joints_Val[i] && joints_Val[i]-joints_current[i] > error_angle_arm)
    {
      m_my->setJointVelocity(joints_Names[i].c_str(), 0.2, 0.0);
      joints_current[i] = m_my->getJointAngle(joints_Names[i].c_str());
    }
    else if(joints_current[i] > joints_Val[i] && joints_current[i]-joints_Val[i] > error_angle_arm)
     {
      m_my->setJointVelocity(joints_Names[i].c_str(), -0.2, 0.0);
      joints_current[i] = m_my->getJointAngle(joints_Names[i].c_str());
     }
    else
    {
      m_my->setJointVelocity(joints_Names[i].c_str(), 0.0, 0.0);
      Joint_State[i] = true;
    }

  }
  else Joint_State[i] = true;

}

global_status = true;

for (int k=0; k<76; k++)
 {
if (Joint_State[k] == false)
global_status = false; 
 }
  
  return global_status;
}

//自身のインスタンスをSIGVerseに返します
extern "C" Controller * createController() {
  return new RobotController;
}