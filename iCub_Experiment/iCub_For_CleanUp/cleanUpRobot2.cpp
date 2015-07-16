#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <algorithm>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <time.h> // for clock

#define PI 3.1415926535

//角度からラジアンに変換します
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )


// The robot must release the Object before to start a new task

//double tab_joint_left[6][7] = {{0,0,0,0,0,0,0},{0.5,-0.8,-0.2,-1.5,-0.2,0,0},{-0.25,-0.6,-0.9,-0.65,-0.3,0,0},{0,-1,-1.3,-1.3,0,0,0},{0,-1,-1,-1,0,0,0},{0,0,0,-1.5,0,0,0}};
double tab_joint_left[6][7] = {{0,0,0,0,0,0,0},{-0.4,-0.1,0.45,-1.2,-0.1,0.37,0},{-0.47,-0.76,0.16,-0.85,0.0,0.6,0},{0,-1,-1.3,-1.3,0,0,0},{0,-1,-1,-1,0,0,0},{0,0,0,-1.5,0,0,0}};

double tab_joint_right[6][7] = {{0,0,0,0,0,0,0},{-0.5,-0.5,0.2,-1.5,0,0,0},{0,-1.1,1.27,-0.7,0.1,0,0},{0,-1.1,1.57,-1.57,0.3,0,0},{0,-1,1,-1,0,0,0},{0,0,0,-1.5,0,0,0}};
double tab_Hand_left[2][20] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0.453,-0.122,0.122,-0.453,-0.21,0,-0.366,-0.697,-0.122,0,-0.366,-0.697,0.035,0,-0.366,-0.697,0.122,0,-0.366,-0.697}};


double tab_joint_walk[14][8] = {{0,0,0,0,0,0,0,0},{-0.174,0.26,0,0,0,0,0,0},{-0.348,0.697,0,0.348,0.087,0,0,0},{-0.488,0.872,0,0.244,0,0,0,0},{-0.697,0.610,0,0.348,0,0,0,0},{-0.523,0.523,0,0.436,0,0,0,0},{-0.174,0.174,0,0.174,0,0,0,0},{0,0,0,-0.174,0.26,0,0,0},{0.348,0.087,0,-0.348,0.697,0,0,0},{0.244,0,0,-0.488,0.872,0,0,0},{0.348,0,0,-0.697,0.610,0,0,0},{0.436,0,0,-0.523,0.523,0,0,0},{0.174,0,0,-0.174,0.174,0,0,0},{0,0,0,0,0,0,0,0}};



std::string joints_Names_LHand[] = {"LHAND_THUMB_BASE","LHAND_THUMB_0","LHAND_THUMB_1","LHAND_THUMB_2","LHAND_FINGER4_BASE","LHAND_FINGER4_0","LHAND_FINGER4_1",
  "LHAND_FINGER4_2","LHAND_FINGER3_BASE","LHAND_FINGER3_0","LHAND_FINGER3_1","LHAND_FINGER3_2","LHAND_FINGER2_BASE","LHAND_FINGER2_0","LHAND_FINGER2_1",
  "LHAND_FINGER2_2","LHAND_FINGER1_BASE","LHAND_FINGER1_0","LHAND_FINGER1_1","LHAND_FINGER1_2"};





std::string joints_Names_RHand[] = { "RHAND_THUMB_BASE","RHAND_THUMB_0","RHAND_THUMB_1","RHAND_THUMB_2","RHAND_FINGER4_BASE","RHAND_FINGER4_0","RHAND_FINGER4_1",
"RHAND_FINGER4_2","RHAND_FINGER3_BASE","RHAND_FINGER3_0","RHAND_FINGER3_1","RHAND_FINGER3_2","RHAND_FINGER2_BASE","RHAND_FINGER2_0","RHAND_FINGER2_1",
"RHAND_FINGER2_2","RHAND_FINGER1_BASE","RHAND_FINGER1_0","RHAND_FINGER1_1","RHAND_FINGER1_2"
};



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

std::string joints_Legs[] = {"LLEG_JOINT0","LLEG_JOINT1","LLEG_JOINT2","LLEG_JOINT3","LLEG_JOINT4",
  "LLEG_JOINT5","RLEG_JOINT0","RLEG_JOINT1","RLEG_JOINT2","RLEG_JOINT3","RLEG_JOINT4","RLEG_JOINT5"
};


#define error_angle 0.09
#define error_distance 1.3
#define error_angle_arm 0.09
#define eps 0

#define error_angle_walk 0.2

int sgn(double d){ 
    return d<-eps?-1:d>eps;
}


double MAG(Vector3d kvec)
{
double sum = sqrt(kvec.x()*kvec.x()+kvec.y()*kvec.y()+kvec.z()*kvec.z());
 return  sum; 
}

Vector3d MINV(Vector3d kvec, Vector3d svec)          /* CROSS Vector Product */              
{  

Vector3d vec_res ;
   vec_res.x(kvec.x()-svec.x()) ;
   vec_res.y(kvec.y()-svec.y()) ;
   vec_res.z(kvec.z()-svec.z()) ;
  return  vec_res;
}


Vector3d PLSV(Vector3d kvec, Vector3d svec)          /* CROSS Vector Product */              
{  
Vector3d vec_res ;
   vec_res.x(kvec.x()+svec.x()) ;
   vec_res.y(kvec.y()+svec.y()) ;
   vec_res.z(kvec.z()+svec.z()) ;
  return  vec_res;
}

Vector3d MULT(Vector3d kvec, double v)          /* CROSS Vector Product */              
{  
Vector3d vec_res ;
   vec_res.x(kvec.x()*v) ;
   vec_res.y(kvec.y()*v) ;
   vec_res.z(kvec.z()*v) ;
  return  vec_res;
}


double DOT(Vector3d kvec, Vector3d svec)            /* DOT Vector Product */                
{   
double s;                                                                    
    s = kvec.x()*svec.x() + kvec.y()*svec.y() + kvec.z()*svec.z();  

    return s;           
}

double NORM(Vector3d kvec)          /* Normalize*/              
{
  double s;                                                                    
    s = sqrt(kvec.x()*kvec.x()+kvec.y()*kvec.y()+kvec.z()*kvec.z()); 

    return s;           
}


Vector3d CROSS(Vector3d kvec, Vector3d svec)          /* CROSS Vector Product */              
{            
    Vector3d vec_res ;                                                       
    vec_res.x(kvec.y()*svec.z() - kvec.z()*svec.y()) ;                             
    vec_res.y(kvec.z()*svec.x() - kvec.x()*svec.z()) ;                             
    vec_res.z(kvec.x()*svec.y() - kvec.y()*svec.x()) ; 

    return  vec_res;                           
}

Vector3d NORMAL(Vector3d kvec)          /* Normalize*/              
{
     Vector3d vec_res ;
   double sum = sqrt(kvec.x()*kvec.x()+kvec.y()*kvec.y()+kvec.z()*kvec.z());
   vec_res.x(kvec.x()/sum) ;
   vec_res.y(kvec.y()/sum) ;
   vec_res.z(kvec.z()/sum) ;
  return  vec_res; 
}




double PhiVal_2D(Vector3d A,Vector3d B)
{
double  Phi;
double nA;
double nB;
Vector3d NormPlan(0, 0, 1);
Vector3d nn = MINV(A , A);
Vector3d aA = NORMAL(nn);
//Vector_Tree nnA = MIN(aA,NORMAL(CROSS(CROSS(aA,NormPlan),NormPlan)));
Vector3d nnA = NORMAL(CROSS(CROSS(aA,NormPlan),NormPlan));
nA = NORM(nnA);

nB = NORM(B);
//Phi = acos(DOT(aA,B) /(nA*nB));

/*
printf("A X :  %f\n",nnA.X );
printf("A Y :  %f\n",nnA.Y );
printf("A Z :  %f\n",nnA.Z );

printf("B X :  %f\n",B.X );
printf("B Y :  %f\n",B.Y );
printf("B Z :  %f\n",B.Z );

*/
Phi = atan2(aA.x()*B.y()-aA.y()*B.x(),aA.x()*B.x()-aA.y()*B.y());
return Phi;
}





//ControllerのサブクラスMoveControllerの宣言します
class RobotController : public Controller {
public:

  void onInit(InitEvent &evt);
  double onAction(ActionEvent&);
  void onRecvMsg(RecvMsgEvent &evt);
  //void onCollision(CollisionEvent &evt);
  bool recognizeTrash(Vector3d &pos, std::string &name);
  std::string getPointedTrashName(std::string entName);
  void stopRobotMove(void);

    //function use by other function
    void recognizeObjectPosition(Vector3d &pos, std::string &name);
    double getAngularXonVect(Vector3d pos, Vector3d mypos);
    double getDistoObj(Vector3d pos, Vector3d pos2);
    double getDist2D(Vector3d pos, Vector3d pos2);
    double getDist3D(Vector3d pos, Vector3d pos2);
    double getRoll(Rotation rot);
    double getPitch(Rotation rot);
    double getYaw(Rotation rot);
    bool GraspLeft();

    //function for the left arm
    bool moveLeftArm();
    void grasp_left_hand();
    void release_left_hand();
    void grasp_right_hand();
    void release_right_hand();
    bool goTo(Vector3d pos, double rangeToPoint);
    void chooze_task_arm_left(int task);
    bool moveRightArm();
    void chooze_task_arm_right(int task);
    void chooze_task_Hand_Left(int task);
   // void grasp_left_hand();
   bool moveBody(std::vector <double  > joints_Val);
   void Capture(std::string number);
   void moveHeadByHMD(const std::string ss);
   void Record_Position();
   bool Image_Range();
   void Walk_robot();
    
  // エージェントが指差している方向にあるオブジェクトの名前を取得します
  std::string getPointedObjectName(std::string entName);

  /* @brief  位置を指定しその方向に回転を開始し、回転終了時間を返します
   * @param  pos 回転したい方向の位置
   * @param  vel 回転速度
   * @param  now 現在時間
   * @return 回転終了時間
   */
  double rotateTowardObj(Vector3d pos, double vel, double now);

  /* @brief  位置を指定しその方向に進みます
   * @param  pos   行きたい場所
   * @param  vel   移動速度
   * @param  range 半径range以内まで移動
   * @param  now   現在時間
   * @return 到着時間
   */
  double goToObj(Vector3d pos, double vel, double range, double now);
  void initWalk(void);
private:
  // New defifinitions
    RobotObj *my;
    //define different joint of right and left arm
    double joint_left[7];
    double joint_right[7];
    double m_joint_left[7];
    double m_joint_right[7];
    double joints_Vals[20];
    //grasping
    bool m_grasp_left;
    double joint_veloc;
    double joint_veloc_walk;
    //to define the object and the table
    std::string m_object, m_table;

    int m_robotState;
    int m_state;
    int m_frontState;
    int stopfront;


   int cycle;
 //   Vector3d go_to;
    double Robot_speed ;
    double Change_Robot_speed;

Vector3d m_BottleFront ;
Vector3d m_MuccupFront;
Vector3d m_CanFront;

Vector3d m_WagonFront;
Vector3d m_BurnableFront;
Vector3d m_UnburnableFront;
Vector3d m_RecycleFront;
Vector3d m_relayPoint1;
Vector3d m_relayPoint2;
Vector3d m_relayFrontTable;
Vector3d m_relayFrontTrash;
Vector3d m_Object_togo;
Vector3d m_Trash_togo;
Vector3d m_relayFrontTable_reset;


  // Takeitの対称から除外するオブジェクトの名前
  std::vector<std::string> exobj;
  std::vector<std::string> extrash;
  //RobotObj *m_my;
  Vector3d m_tpos;

  //状態
  /***
   * @brief ロボットの状態
   * 0 "TAKEIT"の指示を待っている状態
   * 1 取りに行くオブジェクトを特定中
   * 2 オブジェクトに向けて移動中
   * 3 オブジェクトを取得し戻る途中
   */
 // int m_robotState;
 // int m_state;

  // エージェントの正面の方向ベクトル通常はz軸の正の方向と考える
  Vector3d m_defaultNormalVector;

  // 取りにいくオブジェクト名
  std::string m_pointedObject;


// pointed trash
  std::string m_pointedtrash;
  // 指示を受けたアバター名
  std::string m_avatar;

  // onActionの戻り値
  double m_onActionReturn;

  // 1stepあたりの移動距離
  double m_speedDelta;

  // ゴミの名前
  std::string m_tname;
  // ゴミ候補オブジェクト
  std::vector<std::string> m_trashes;
  // ゴミ箱オブジェクト
  std::vector<std::string> m_trashboxs;

  // 車輪の角速度
  double m_vel;

  // 関節の回転速度
  double m_jvel;

  // 車輪半径
  double m_radius;

  // 車輪間距離
  double m_distance;

  // 移動終了時間
  double m_time;
  double m_time_LA1;
  double m_time_LA4;
  double m_time_RA1;
  double m_time_RA4;

  // 初期位置
  Vector3d m_inipos;

  Vector3d pos_a;
  Vector3d pos_b;

  // grasp中かどうか
  bool m_grasp;

  // ゴミ認識サービス
  BaseService *m_recogSrv;

  // サービスへのリクエスト複数回送信を防ぐ
  bool m_sended;

  std::string msg_ob;
  std::string msg_trash;
  Vector3d  m_BottleReset;
  Vector3d  m_MuccupReset;
  Vector3d  m_CanReset;
  Vector3d  m_RobotPos;
  Vector3d  m_Table ;
  Vector3d  Object_reset;
  Rotation rot;
  bool reset_op;
  bool reset_out;
  bool take_action;
  bool put_action;

  FILE* fp;
  std::string file_name;

  FILE* Rfp;
  std::string Rfile_name;

  float stepWidth;
  int sleeptime;

  const static int SIZE = 31;
  const static int RSIZE = 14;
  int motionNum;
    int RmotionNum;
  float HEIGHT[SIZE];
  float LARM_JOINT0[SIZE]; // left shoulder
  float LARM_JOINT3[SIZE]; // left elbow
  float RARM_JOINT0[SIZE]; // right shoulder
  float RARM_JOINT3[SIZE]; // right elbow
  float LLEG_JOINT2[SIZE]; // left groin(leg)
  float LLEG_JOINT4[SIZE]; // left knee
  float LLEG_JOINT6[SIZE]; // left ankle
  float RLEG_JOINT2[SIZE]; // right groin(leg)
  float RLEG_JOINT4[SIZE]; // right knee
  float RLEG_JOINT6[SIZE]; // right ankle

float RHEIGHT[SIZE];
  float RLARM_JOINT0[RSIZE]; // left shoulder
  float RLARM_JOINT3[RSIZE]; // left elbow
  float RRARM_JOINT0[RSIZE]; // right shoulder
  float RRARM_JOINT3[RSIZE]; // right elbow
  float RLLEG_JOINT2[RSIZE]; // left groin(leg)
  float RLLEG_JOINT4[RSIZE]; // left knee
  float RLLEG_JOINT6[RSIZE]; // left ankle
  float RRLEG_JOINT2[RSIZE]; // right groin(leg)
  float RRLEG_JOINT4[RSIZE]; // right knee
  float RRLEG_JOINT6[RSIZE]; // right ankle




  int Rcount;
  int Rjinc;

  int count;
  int jinc;

 std::vector< std::vector <double> >  joints_Values;
 double pyaw, ppitch, proll;

  double m_qw, m_qy, m_qx, m_qz;
  double o_qw, o_qx, o_qy, o_qz;
  double po_qw, po_qx, po_qy, po_qz;

  BaseService *m_hmd;

  std::ofstream Position_Entities;
  SimObj *Entitie_Pos;
  std::vector<std::string> m_entities;
 double take_time;
 bool init_time ;
double elapsedTime;
Vector3d Coord_Pos[7];
bool States_image[5];
bool Get_range;
CParts * parts2;
CParts * parts1;
CParts * parts3;



bool walk_State;
int walk_Step;
double joint_walk[7];

std::vector < Vector3d > Data_vec;
std::vector< std::vector < Vector3d > >  Data_record;
};
void RobotController::stopRobotMove(void)
{
  my->setWheelVelocity(0.0, 0.0);
       my->setJointAngle("LLEG_JOINT0", 0);
    //   my->setJointAngle("LARM_JOINT3",0);
       my->setJointAngle("LLEG_JOINT3", 0);

       my->setJointAngle("LLEG_JOINT4", 0);
       my->setJointAngle("RLEG_JOINT0", 0);

    //   my->setJointAngle("RARM_JOINT3", 0);
       my->setJointAngle("RLEG_JOINT3", 0);
       my->setJointAngle("RLEG_JOINT4", 0);





         // my->setJointAngle("LARM_JOINT3", DEG2RAD(LARM_JOINT3[jinc]));
         // my->setJointAngle("RARM_JOINT1", DEG2RAD(RARM_JOINT1[jinc]));
         // my->setJointAngle("RARM_JOINT3", DEG2RAD(RARM_JOINT3[jinc]));
}
void RobotController::initWalk(void)
{
  count = 0;
  jinc = 0;
  Rcount = 0;
  Rjinc = 0;

}






/************************************************************************************/
/*******Find roll,pitch,yaw, lenght between to point, angle between to point*********/
/************************************************************************************/

double RobotController::getRoll(Rotation rot)
{
  // get angles arround y-axis
  double qw = rot.qw();
  double qx = rot.qx();
  double qy = rot.qy();
  double qz = rot.qz();

  double roll  = atan2(2*qy*qw - 2*qx*qz, 1 - 2*qy*qy - 2*qz*qz);

  return roll;
}

double RobotController::getPitch(Rotation rot)
{
  // get angles arround y-axis
  double qw = rot.qw();
  double qx = rot.qx();
  double qy = rot.qy();
  double qz = rot.qz();

  double pitch = atan2(2*qx*qw - 2*qy*qz, 1 - 2*qx*qx - 2*qz*qz);

  return pitch;
}

double RobotController::getYaw(Rotation rot)
{
    // get angles arround y-axis
  double qw = rot.qw();
  double qx = rot.qx();
  double qy = rot.qy();
  double qz = rot.qz();

  double yaw   = asin(2*qx*qy + 2*qz*qw);

  return yaw;
}

double RobotController::getDistoObj(Vector3d pos, Vector3d pos2)
{
  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= pos2;

  // measure actual distance
  double distance = sqrt(l_pos.x()*l_pos.x()+l_pos.z()*l_pos.z());

  return distance;
}

double RobotController::getAngularXonVect(Vector3d pos, Vector3d mypos)
{
  double targetAngle;

  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= mypos;

  // rotation angle from z-axis to vector
  targetAngle = atan(l_pos.x()/l_pos.z());

  // direction
  if(l_pos.z() < 0 && l_pos.x()>=0)
    targetAngle = M_PI+targetAngle;
  else if (l_pos.z()<0 && l_pos.x()<0)
    targetAngle = targetAngle-M_PI;

  return targetAngle;
}

double RobotController::getDist2D(Vector3d pos, Vector3d pos2)
{
  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= pos2;

  return sqrt(l_pos.x()*l_pos.x()+l_pos.z()*l_pos.z());
}

double RobotController::getDist3D(Vector3d pos, Vector3d pos2)
{
  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= pos2;

  return sqrt(l_pos.x()*l_pos.x()+l_pos.z()*l_pos.z()+l_pos.y()*l_pos.y());
}



/************************************************************************************/

/************************************************************************************/
/****************************function for the left arm*******************************/
/************************************************************************************/

bool RobotController::moveLeftArm()
{
  bool j0 = false, j1 = false , j3 = false, j4 = false, j5 = false, j6 = false, j2 = false;

  if(joint_left[0] != m_joint_left[0] )
  {
    if(joint_left[0] < m_joint_left[0] && m_joint_left[0]-joint_left[0] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT0", joint_veloc, 0.0);
      joint_left[0] = my->getJointAngle("LARM_JOINT0");
    }
    else if(joint_left[0] > m_joint_left[0] && joint_left[0]-m_joint_left[0] > error_angle_arm)
     {
      my->setJointVelocity("LARM_JOINT0", -joint_veloc, 0.0);
      joint_left[0] = my->getJointAngle("LARM_JOINT0");
     }
    else
    {
      my->setJointVelocity("LARM_JOINT0", 0.0, 0.0);
      j0 = true;
    }

  }
  else j0 = true;

  if(joint_left[1] != m_joint_left[1] )
  {
    if(joint_left[1] < m_joint_left[1] && m_joint_left[1]-joint_left[1] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT1", joint_veloc, 0.0);
      joint_left[1] = my->getJointAngle("LARM_JOINT1");
    }
    else if(joint_left[1] > m_joint_left[1] && joint_left[1]-m_joint_left[1] > error_angle_arm)
       {
        my->setJointVelocity("LARM_JOINT1", -joint_veloc, 0.0);
        joint_left[1] = my->getJointAngle("LARM_JOINT1");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT1", 0.0, 0.0);
         j1 = true;
       }
  }
  else j1 = true;


  if(joint_left[2] != m_joint_left[2] )
  {
    if(joint_left[2] < m_joint_left[2] && m_joint_left[2]-joint_left[2] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT2", joint_veloc, 0.0);
      joint_left[2] = my->getJointAngle("LARM_JOINT2");
    }
    else if(joint_left[2] > m_joint_left[2] && joint_left[2]-m_joint_left[2] > error_angle_arm)
       {
        my->setJointVelocity("LARM_JOINT2", -joint_veloc, 0.0);
        joint_left[2] = my->getJointAngle("LARM_JOINT2");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT2", 0.0, 0.0);
         j2 = true;
       }
  }
  else j2 = true;




  if(joint_left[3] != m_joint_left[3] )
  {
    if(joint_left[3] < m_joint_left[3] && m_joint_left[3]-joint_left[3] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT3", joint_veloc, 0.0);
      joint_left[3] = my->getJointAngle("LARM_JOINT3");
    }
    else if(joint_left[3] > m_joint_left[3] && joint_left[3]-m_joint_left[3] > error_angle_arm)
       {
        my->setJointVelocity("LARM_JOINT3", -joint_veloc, 0.0);
        joint_left[3] = my->getJointAngle("LARM_JOINT3");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT3", 0.0, 0.0);
         j3 = true;
       }
  }
  else j3 = true;

  if(joint_left[4] != m_joint_left[4] )
  {
    if(joint_left[4] < m_joint_left[4] && m_joint_left[4]-joint_left[4] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT4", joint_veloc, 0.0);
      joint_left[4] = my->getJointAngle("LARM_JOINT4");
    }
    else if(joint_left[4] > m_joint_left[4] && joint_left[4]-m_joint_left[4] > error_angle_arm)
       {
        my->setJointVelocity("LARM_JOINT4", -joint_veloc, 0.0);
        joint_left[4] = my->getJointAngle("LARM_JOINT4");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
         j4 = true;
       }
  }
  else j4 = true;

  if(joint_left[5] != m_joint_left[5] )
  {
    if(joint_left[5] < m_joint_left[5] && m_joint_left[5]-joint_left[5] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT5", joint_veloc, 0.0);
      joint_left[5] = my->getJointAngle("LARM_JOINT5");
    }
    else if(joint_left[5] > m_joint_left[5] && joint_left[5]-m_joint_left[5] > error_angle_arm)
       {
        my->setJointVelocity("LARM_JOINT5", -joint_veloc, 0.0);
        joint_left[5] = my->getJointAngle("LARM_JOINT5");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT5", 0.0, 0.0);
         j5 = true;
       }
  }
  else j5 = true;

  if(joint_left[6] != m_joint_left[6] )
  {
    if(joint_left[6] < m_joint_left[6] && m_joint_left[6]-joint_left[6] > error_angle_arm)
    {
      my->setJointVelocity("LARM_JOINT6", joint_veloc, 0.0);
      joint_left[6] = my->getJointAngle("LARM_JOINT6");
    }
    else if(joint_left[6] > m_joint_left[6] && joint_left[6]-m_joint_left[6] > error_angle_arm)
       {
        my->setJointVelocity("LARM_JOINT6", -joint_veloc, 0.0);
        joint_left[6] = my->getJointAngle("LARM_JOINT6");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT6", 0.0, 0.0);
         j6 = true;
       }
  }
  else j6 = true;



  if (j0 == true && j1 == true && j3 == true && j4 == true && j5 == true && j6 == true && j2 == true)
    return true;
  else
    return false;

  return false;
}

void RobotController::chooze_task_arm_left(int task)
{
  for (int i=0; i<7; i++)
    m_joint_left[i] = tab_joint_left[task][i];
}





/************************************************************************************/

/************************************************************************************/
/****************************function for the right arm*******************************/
/************************************************************************************/

bool RobotController::moveRightArm()
{
  bool j0 = false, j1 = false , j3 = false, j4 = false, j5 = false, j6 = false, j2 = false;

  if(joint_right[0] != m_joint_right[0] )
  {
    if(joint_right[0] < m_joint_right[0] && m_joint_right[0]-joint_right[0] > error_angle_arm)
    {
      my->setJointVelocity("RARM_JOINT0",joint_veloc, 0.0);
      joint_right[0] = my->getJointAngle("RARM_JOINT0");
    }
    else if(joint_right[0] > m_joint_right[0] && joint_right[0]-m_joint_right[0] > error_angle_arm)
     {
      my->setJointVelocity("RARM_JOINT0",-joint_veloc, 0.0);
      joint_right[0] = my->getJointAngle("RARM_JOINT0");
     }
    else
    {
      my->setJointVelocity("RARM_JOINT0", 0.0, 0.0);
      j0 = true;
    }

  }
  else j0 = true;

  if(joint_right[1] != m_joint_right[1] )
  {
    if(joint_right[1] < m_joint_right[1] && m_joint_right[1]-joint_right[1] > error_angle_arm)
    {
      my->setJointVelocity("RARM_JOINT1",joint_veloc, 0.0);
      joint_right[1] = my->getJointAngle("RARM_JOINT1");
    }
    else if(joint_right[1] > m_joint_right[1] && joint_right[1]-m_joint_right[1] > error_angle_arm)
       {
        my->setJointVelocity("RARM_JOINT1",-joint_veloc, 0.0);
        joint_right[1] = my->getJointAngle("RARM_JOINT1");
       }
       else
       {
         my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
         j1 = true;
       }
  }
  else j1 = true;

  if(joint_right[2] != m_joint_right[2] )
  {
    if(joint_right[2] < m_joint_right[2] && m_joint_right[2]-joint_right[2] > error_angle_arm)
    {
      my->setJointVelocity("RARM_JOINT3",joint_veloc, 0.0);
      joint_right[2] = my->getJointAngle("RARM_JOINT3");
    }
    else if(joint_right[2] > m_joint_right[2] && joint_right[2]-m_joint_right[2] > error_angle_arm)
       {
        my->setJointVelocity("RARM_JOINT3",-joint_veloc, 0.0);
        joint_right[2] = my->getJointAngle("RARM_JOINT3");
       }
       else
       {
         my->setJointVelocity("RARM_JOINT3", 0.0, 0.0);
         j3 = true;
       }
  }
  else j3 = true;

  if(joint_right[3] != m_joint_right[3] )
  {
    if(joint_right[3] < m_joint_right[3] && m_joint_right[3]-joint_right[3] > error_angle_arm)
    {
      my->setJointVelocity("RARM_JOINT4", joint_veloc, 0.0);
      joint_right[3] = my->getJointAngle("RARM_JOINT4");
    }
    else if(joint_right[3] > m_joint_right[3] && joint_right[3]-m_joint_right[3] > error_angle_arm)
       {
        my->setJointVelocity("RARM_JOINT4",-joint_veloc, 0.0);
        joint_right[3] = my->getJointAngle("RARM_JOINT4");
       }
       else
       {
         my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
         j4 = true;
       }
  }
  else j4 = true;

  if(joint_right[4] != m_joint_right[4] )
  {
    if(joint_right[4] < m_joint_right[4] && m_joint_right[4]-joint_right[4] > error_angle_arm)
    {
      my->setJointVelocity("RARM_JOINT5", joint_veloc, 0.0);
      joint_right[4] = my->getJointAngle("RARM_JOINT5");
    }
    else if(joint_right[4] > m_joint_right[4] && joint_right[4]-m_joint_right[4] > error_angle_arm)
       {
        my->setJointVelocity("RARM_JOINT5", -joint_veloc, 0.0);
        joint_right[4] = my->getJointAngle("RARM_JOINT5");
       }
       else
       {
         my->setJointVelocity("RARM_JOINT5", 0.0, 0.0);
         j5 = true;
       }
  }
  else j5 = true;

  if(joint_right[5] != m_joint_right[5] )
  {
    if(joint_right[5] < m_joint_right[5] && m_joint_right[5]-joint_right[5] > error_angle_arm)
    {
      my->setJointVelocity("RARM_JOINT6", joint_veloc, 0.0);
      joint_right[5] = my->getJointAngle("RARM_JOINT6");
    }
    else if(joint_right[5] > m_joint_right[5] && joint_right[5]-m_joint_right[5] > error_angle_arm)
       {
        my->setJointVelocity("RARM_JOINT6", -joint_veloc, 0.0);
        joint_right[5] = my->getJointAngle("RARM_JOINT6");
       }
       else
       {
         my->setJointVelocity("RARM_JOINT6", 0.0, 0.0);
         j6 = true;
       }
  }
  else j6 = true;

  if(joint_right[6] != m_joint_right[6] )
  {
    if(joint_right[5] < m_joint_right[6] && m_joint_right[6]-joint_right[5] > error_angle_arm)
    {
      my->setJointVelocity("RARM_JOINT2", joint_veloc, 0.0);
      joint_right[5] = my->getJointAngle("RARM_JOINT2");
    }
    else if(joint_right[5] > m_joint_right[6] && joint_right[5]-m_joint_right[6] > error_angle_arm)
       {
        my->setJointVelocity("RARM_JOINT7", -joint_veloc, 0.0);
        joint_right[5] = my->getJointAngle("RARM_JOINT2");
       }
       else
       {
         my->setJointVelocity("RARM_JOINT2", 0.0, 0.0);
         j2 = true;
       }
  }
  else j2 = true;

  if (j0 == true && j1 == true && j3 == true && j4 == true && j5 == true && j6 == true && j2 == true)
    return true;
  else
    return false;

  return false;
}


void RobotController::chooze_task_arm_right(int task)
{
  for (int i=0; i<7; i++)
    m_joint_right[i] = tab_joint_right[task][i];
}



void RobotController::chooze_task_Hand_Left(int task)
{
  for (int i=0; i<20; i++)
    joints_Vals[i] = tab_Hand_left[task][i];
}






void RobotController::grasp_left_hand()
{
  Vector3d hand, object;
  SimObj *obj = getObj(m_pointedObject.c_str());

  obj->getPosition(object);
  my->getJointPosition(hand, "LARM_JOINT7");

  double distance = getDist3D(hand,object);

  if(distance < 13 &&  m_grasp_left == false)
  {
    CParts * parts = my->getParts("GRASP_LINK");
    if (parts->graspObj(m_pointedObject))
    {
      m_grasp_left = true;
    //  broadcastMsg("Object_grasped");
    }
  }
}

void RobotController::grasp_right_hand()
{
  Vector3d hand, object;
  SimObj *obj = getObj(m_pointedObject.c_str());

  obj->getPosition(object);
  my->getJointPosition(hand, "RARM_JOINT6");

  double distance = getDist3D(hand,object);

  if(distance < 13 &&  m_grasp_left == false)
  {
    CParts * parts = my->getParts("RARM_LINK6");
    if (parts->graspObj(m_pointedObject))
    {
      m_grasp_left = true;
    //  broadcastMsg("Object_grasped");
    }
  }
}


void RobotController::release_left_hand()
{
  CParts * parts = my->getParts("GRASP_LINK");
  if( m_grasp_left == true)
    printf("I'm releasing \n");
    parts->releaseObj();
  m_grasp_left = false;
}


void RobotController::release_right_hand()
{
  CParts * parts = my->getParts("RARM_LINK6");
  if( m_grasp_left == true)
    printf("I'm releasing \n");
    parts->releaseObj();
  m_grasp_left = false;
}


bool RobotController::GraspLeft()
{
  //bool j0 = false, j1 = false , j3 = false, j4 = false, j5 = false, j6 = false, j7 = false;
  bool Joint_State[20];
  double joints_current[20];
  bool global_status = false; 
for (int j=0; j<20; j++)
 {
Joint_State[j]= false;
joints_current[j] = my->getJointAngle(joints_Names_LHand[j].c_str());
 }


 for (int i=0; i<20; i++)

 {
  if(joints_current[i] != joints_Vals[i] )
  {
    if(joints_current[i] < joints_Vals[i] && joints_Vals[i]-joints_current[i] > error_angle_arm)
    {
      my->setJointVelocity(joints_Names_LHand[i].c_str(), 0.2, 0.0);
      joints_current[i] = my->getJointAngle(joints_Names_LHand[i].c_str());
    }
    else if(joints_current[i] > joints_Vals[i] && joints_current[i]-joints_Vals[i] > error_angle_arm)
     {
      my->setJointVelocity(joints_Names_LHand[i].c_str(), -0.2, 0.0);
      joints_current[i] = my->getJointAngle(joints_Names_LHand[i].c_str());
     }
    else
    {
      my->setJointVelocity(joints_Names_LHand[i].c_str(), 0.0, 0.0);
      Joint_State[i] = true;
    }

  }
  else Joint_State[i] = true;

}

global_status = true;

for (int k=0; k<20; k++)
 {
if (Joint_State[k] == false)
global_status = false; 
 }
  
  return global_status;
}



/*************************************************************************************/

/*************************************************************************************/

void RobotController::recognizeObjectPosition(Vector3d &pos, std::string &name)
{
  // get object of trash selected
  SimObj *trash = getObj(name.c_str());

  // get trash's position
  trash->getPosition(pos);
}

/*************************************************************************************/


/************************************************************************************/
/***************************Move the robot in the world******************************/
/************************************************************************************/

bool RobotController::goTo(Vector3d pos, double rangeToPoint)
{
  double speed;

  Vector3d ownPosition;
  my->getPosition(ownPosition);

  Rotation ownRotation;
  my->getRotation(ownRotation);

  double angle = getAngularXonVect(pos, ownPosition);
  double dist = getDistoObj(pos,ownPosition);
  double roll = getRoll(ownRotation);
/*if(dist-rangeToPoint > 0.5)
{
  
}*/

  if (angle > 3 || angle < -3) angle = M_PI;


  // error on angle
  if ((angle-roll)>-error_angle && (angle-roll)<error_angle)
  // error on distance
  {
      

    if (dist-rangeToPoint < error_distance && dist-rangeToPoint > -error_distance)
    {
      stopRobotMove();
      initWalk();

      return true;
    }
    else
    {

  
if(count%2){
         // printf("Motion ...  %f  the counts %d \n ",LLEG_JOINT4[jinc],count );
        //  my->setJointAngle("LARM_JOINT0", DEG2RAD(LARM_JOINT0[jinc]));
        //  my->setJointAngle("LARM_JOINT3", DEG2RAD(LARM_JOINT3[jinc]));
        //  my->setJointAngle("RARM_JOINT1", DEG2RAD(RARM_JOINT0[jinc]));
       //   my->setJointAngle("RARM_JOINT3", DEG2RAD(RARM_JOINT3[jinc]));

          my->setJointAngle("LLEG_JOINT0", DEG2RAD(LLEG_JOINT2[jinc]));
          my->setJointAngle("LLEG_JOINT3", DEG2RAD(LLEG_JOINT4[jinc]));
          my->setJointAngle("LLEG_JOINT4", DEG2RAD(LLEG_JOINT6[jinc]));
          my->setJointAngle("RLEG_JOINT0", DEG2RAD(RLEG_JOINT2[jinc]));
          my->setJointAngle("RLEG_JOINT3", DEG2RAD(RLEG_JOINT4[jinc]));
          my->setJointAngle("RLEG_JOINT4", DEG2RAD(RLEG_JOINT6[jinc]));
       if (jinc == motionNum-1)
       {
        jinc=0;
        count++;
       }
      }

      else {
        
        //  my->setJointAngle("RARM_JOINT0", DEG2RAD(LARM_JOINT0[jinc]));
        //  my->setJointAngle("RARM_JOINT3", DEG2RAD(-LARM_JOINT3[jinc]));
        //  my->setJointAngle("LARM_JOINT0", DEG2RAD(RARM_JOINT0[jinc]));
        //  my->setJointAngle("LARM_JOINT3", DEG2RAD(-RARM_JOINT3[jinc]));

          my->setJointAngle("RLEG_JOINT0", DEG2RAD(LLEG_JOINT2[jinc]));
          my->setJointAngle("RLEG_JOINT3", DEG2RAD(LLEG_JOINT4[jinc]));
          my->setJointAngle("RLEG_JOINT4", DEG2RAD(LLEG_JOINT6[jinc]));
          my->setJointAngle("LLEG_JOINT0", DEG2RAD(RLEG_JOINT2[jinc]));
          my->setJointAngle("LLEG_JOINT3", DEG2RAD(RLEG_JOINT4[jinc]));
          my->setJointAngle("LLEG_JOINT4", DEG2RAD(RLEG_JOINT6[jinc]));
          if (jinc == motionNum-1)
       {
        jinc=0;
        count++;
       } 
      }
    
    jinc++;


//Walk_robot();
      speed = dist-rangeToPoint;
      if (dist-rangeToPoint < 5)
        if( dist-rangeToPoint > 0 )
          my->setWheelVelocity(1, 1);
        else
          my->setWheelVelocity(-1, -1);
      else if( dist-rangeToPoint > 0 )
        my->setWheelVelocity(Robot_speed , Robot_speed );
      else
        my->setWheelVelocity(-Robot_speed , -Robot_speed );
      return false;
    }
  }

  else
  {
    




/*

LARM_JOINT0    X  shoulder
LARM_JOINT1    Y  shoulder
LARM_JOINT2    Z  shoulder

LARM_JOINT3    X  Elbow

LARM_JOINT4    X  Hand  
LARM_JOINT5    Y  Hand
LARM_JOINT6    Z  Hand

RARM_JOINT0    X Shoulder
RARM_JOINT1    Y Shoulder
RARM_JOINT2    Z Shoulder

RARM_JOINT3    X Elbow

RARM_JOINT4    X Hand
RARM_JOINT5    Y Hand
RARM_JOINT6    Z Hand

LLEG_JOINT0    X Hip 
LLEG_JOINT1    Y Hip
LLEG_JOINT2    Z Hip

LLEG_JOINT3    X Knee

LLEG_JOINT4    X Feet
LLEG_JOINT5    Y Feet

RLEG_JOINT0    X Hip
RLEG_JOINT1    Y Hip
RLEG_JOINT2    Z Hip

RLEG_JOINT3    X Knee

RLEG_JOINT4    X Feet    
RLEG_JOINT5    Y Feet

*/


if(Rcount%2){
         // printf("Rotation .....  %f  the counts %d \n ",RRLEG_JOINT6[Rjinc],Rcount );
         // my->setJointAngle("LARM_JOINT0", DEG2RAD(RLARM_JOINT0[jinc]));
         // my->setJointAngle("LARM_JOINT3", DEG2RAD(RLARM_JOINT3[jinc]));
         // my->setJointAngle("RARM_JOINT0", DEG2RAD(RRARM_JOINT0[jinc]));
         //z my->setJointAngle("RARM_JOINT3", DEG2RAD(RRARM_JOINT3[jinc]));

          my->setJointAngle("LLEG_JOINT0", DEG2RAD(RLLEG_JOINT2[Rjinc]));
          my->setJointAngle("LLEG_JOINT3", DEG2RAD(RLLEG_JOINT4[Rjinc]));
          my->setJointAngle("LLEG_JOINT1", DEG2RAD(RLLEG_JOINT6[Rjinc]));
          my->setJointAngle("RLEG_JOINT0", DEG2RAD(RRLEG_JOINT2[Rjinc]));
          my->setJointAngle("RLEG_JOINT3", DEG2RAD(RRLEG_JOINT4[Rjinc]));
          my->setJointAngle("RLEG_JOINT1", DEG2RAD(RRLEG_JOINT6[Rjinc]));
       if (Rjinc == RmotionNum-1)
       {
        Rjinc=0;
        Rcount++;
       }
      }

      else {
        
        //  my->setJointAngle("RARM_JOINT0", DEG2RAD(RLARM_JOINT0[jinc]));
        //  my->setJointAngle("RARM_JOINT3", DEG2RAD(-RLARM_JOINT3[jinc]));
        //  my->setJointAngle("LARM_JOINT0", DEG2RAD(RRARM_JOINT0[jinc]));
        //  my->setJointAngle("LARM_JOINT3", DEG2RAD(-RRARM_JOINT3[jinc]));

          my->setJointAngle("RLEG_JOINT0", DEG2RAD(RLLEG_JOINT2[Rjinc]));
          my->setJointAngle("RLEG_JOINT3", DEG2RAD(RLLEG_JOINT4[Rjinc]));
          my->setJointAngle("RLEG_JOINT1", DEG2RAD(RLLEG_JOINT6[Rjinc]));
          my->setJointAngle("LLEG_JOINT0", DEG2RAD(RRLEG_JOINT2[Rjinc]));
          my->setJointAngle("LLEG_JOINT3", DEG2RAD(RRLEG_JOINT4[Rjinc]));
          my->setJointAngle("LLEG_JOINT1", DEG2RAD(RRLEG_JOINT6[Rjinc]));
          if (Rjinc == RmotionNum-1)
       {
        Rjinc=0;
        Rcount++;
       } 
      }
    Rjinc++;



    speed = fabs(angle-roll)*4;
    if (speed/4 > 0.3)
      if (angle < -M_PI_2 && roll > M_PI_2)
        my->setWheelVelocity(-0.5, 0.5);
      else if (angle > M_PI_2 && roll < -M_PI_2)
          my->setWheelVelocity(0.5, -0.5);
      else if (angle < roll)
          my->setWheelVelocity(0.5, -0.5);
      else
        my->setWheelVelocity(-0.5, 0.5);
    else if (angle < -M_PI_2 && roll > M_PI_2)
        my->setWheelVelocity(-speed, speed);
       else if (angle > M_PI_2 && roll < -M_PI_2)
          my->setWheelVelocity(speed, -speed);
       else if (angle < roll)
          my->setWheelVelocity(speed, -speed);
       else
        my->setWheelVelocity(-speed, speed);

    return false;
  }
        


  return false;
}

/************************************************************************************/

void RobotController::Capture( std::string number)
{

double  joints_current[76]; 
std::vector <double> Vec_value;
for (int j=0; j<76; j++)
 {
joints_current[j] = my->getJointAngle(joints_Names[j].c_str());
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
joints_current[j] = my->getJointAngle(joints_Names[j].c_str());
 }


 for (int i=0; i<76; i++)

 {
  if(joints_current[i] != joints_Val[i] )
  {
    if(joints_current[i] < joints_Val[i] && joints_Val[i]-joints_current[i] > error_angle_arm)
    {
      my->setJointVelocity(joints_Names[i].c_str(), 0.2, 0.0);
      joints_current[i] = my->getJointAngle(joints_Names[i].c_str());
    }
    else if(joints_current[i] > joints_Val[i] && joints_current[i]-joints_Val[i] > error_angle_arm)
     {
      my->setJointVelocity(joints_Names[i].c_str(), -0.2, 0.0);
      joints_current[i] = my->getJointAngle(joints_Names[i].c_str());
     }
    else
    {
      my->setJointVelocity(joints_Names[i].c_str(), 0.0, 0.0);
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










void RobotController::onInit(InitEvent &evt)
{



  m_robotState = 0;
  m_frontState = 0;
  joint_veloc = 0.6;
  joint_veloc_walk = 2;


  //my = getRobotObj(myname());
  //my->setWheel(10.0, 10.0);
  my = getRobotObj(myname());
  my->setWheel(10.0, 10.0);
  stopfront = 0;

  m_grasp_left = false;
  Change_Robot_speed = 3; // to change the robot's velocity
  Robot_speed  = Change_Robot_speed;
  m_state = 0;
  // bjects
  m_BottleFront = Vector3d(40.0, 30, -10.0);
 // m_MuccupFront = Vector3d(0.0, 30, -10.0);
  m_CanFront = Vector3d(-40.0, 30, -10.0);


// trash
 // m_WagonFront = Vector3d(-120.0, 30, -120);
  m_BurnableFront = Vector3d(140.0, 30, -120);
  m_UnburnableFront = Vector3d(20.0, 30, -120);
  m_RecycleFront = Vector3d(260.0, 30, -120);


  m_relayPoint1 = Vector3d(100, 30, -70);
  m_relayPoint2 = Vector3d(0, 30, -70);
  m_relayFrontTable = Vector3d(0, 30,-20);
  m_relayFrontTable_reset = Vector3d(0, 30,-50);
  m_relayFrontTrash = Vector3d(0, 30, -100);


  cycle = 3;
  m_robotState = 0;
  //エージェントの正面の定義はz軸の正の向きを向いていると仮定する
  m_defaultNormalVector = Vector3d(0,0,1);
  m_onActionReturn = 1.0;
  m_speedDelta = 2.0;

  m_avatar = "man_000";

  //m_my = getRobotObj(myname());

  // 初期位置取得
  //m_my->getPosition(m_inipos);
  my->getPartsPosition(m_inipos,"RARM_LINK2");

  pos_a = Vector3d(0, 30, -80);//
  pos_b = Vector3d(0, 30, -10);  ///

  // 車輪間距離
  m_distance = 10.0;

  // 車輪半径
  m_radius  = 10.0;

  // 移動終了時間初期化
  m_time = 0.0;
  m_time_LA1 = 0.0;
  m_time_LA4 = 0.0;
  m_time_RA1 = 0.0;
  m_time_RA4 = 0.0;

  // 車輪の半径と車輪間距離設定
  my->setWheel(m_radius, m_distance);
 // m_state = 20;

  srand((unsigned)time( NULL ));

  // 車輪の回転速度
  m_vel = 0.3;

  // 関節の回転速度
  m_jvel = 0.6;

  // grasp初期化
  m_grasp = false;
  m_recogSrv = NULL;
  m_sended = false;

  // ここではゴミの名前が分かっているとします
  //m_trashes.push_back("petbottle_0");
  //m_trashes.push_back("petbottle_1");
  //m_trashes.push_back("petbottle_2");
  //m_trashes.push_back("petbottle_3");
  m_trashes.push_back("petbottle");
  //m_trashes.push_back("banana");
  //m_trashes.push_back("chigarette");
  //m_trashes.push_back("chocolate");
  //m_trashes.push_back("mayonaise_0");
  //m_trashes.push_back("mayonaise_1");
  //m_trashes.push_back("mugcup");
  //m_trashes.push_back("can_0");
  //m_trashes.push_back("can_1");
  m_trashes.push_back("can");
  //m_trashes.push_back("can_3");

  // ゴミ箱登録
  m_trashboxs.push_back("recycle");
  m_trashboxs.push_back("burnable");
  m_trashboxs.push_back("unburnable");
 // m_trashboxs.push_back("wagon");

  m_BottleReset = Vector3d(40.0, 59.15, 50.0);
 // m_MuccupReset = Vector3d(0.0, 52.15, 50.0);
  m_CanReset = Vector3d(-40.0, 54.25, 50.0);
  m_RobotPos = Vector3d(100.0, 30.0,-100.0);
  m_Table = Vector3d(0.0, 24.0,70.0);
  rot.setQuaternion(1, 0, 0, 0);
reset_op = false;
reset_out = false;

take_action = true;
put_action = true;




  if((fp = fopen("Walk.txt", "r")) == NULL) {
    printf("File do not exist.\n");
    exit(0);
  }
  else{
    fscanf(fp, "%d", &motionNum);
    fscanf(fp, "%d", &sleeptime);
    for(int i=0; i<motionNum; i++){
      fscanf(fp, "%f %f %f %f %f %f %f %f %f %f %f",
             &HEIGHT[i],
             &LARM_JOINT0[i],
             &LARM_JOINT3[i],
             &RARM_JOINT0[i],
             &RARM_JOINT3[i],
             &LLEG_JOINT2[i],
             &LLEG_JOINT4[i],
             &LLEG_JOINT6[i],
             &RLEG_JOINT2[i],
             &RLEG_JOINT4[i],
             &RLEG_JOINT6[i]);
    }
  }

 fclose (fp);





    if((Rfp = fopen("motion_rotation.txt", "r")) == NULL) {
    printf("File do not exist.\n");
    exit(0);
  }
  else{
    fscanf(Rfp, "%d", &RmotionNum);
    fscanf(Rfp, "%d", &sleeptime);
    for(int i=0; i<RmotionNum; i++){
      fscanf(fp, "%f %f %f %f %f %f %f %f %f %f %f",
             &RHEIGHT[i],
             &RLARM_JOINT0[i],
             &RLARM_JOINT3[i],
             &RRARM_JOINT0[i],
             &RRARM_JOINT3[i],
             &RLLEG_JOINT2[i],
             &RLLEG_JOINT4[i],
             &RLLEG_JOINT6[i],
             &RRLEG_JOINT2[i],
             &RRLEG_JOINT4[i],
             &RRLEG_JOINT6[i]);
    }
  }
for(int i=0; i<RmotionNum; i++){
std::cout << " HEIGHT  " << i << "  ......  " <<  RHEIGHT[i] <<std::endl;   
std::cout << " Left Leg  " << i << "  ......  " <<  RLLEG_JOINT6[i] <<std::endl; 
std::cout << " Right Leg " << i << "  ......  " <<  RRLEG_JOINT6[i] <<std::endl;


  }

 initWalk();
m_hmd = NULL;

getAllEntities(m_entities);

//Position_Entities.open("SIGVerse_Record.csv");
//Position_Entities << "timeStamp robotPosx robotPosz handx,handy handz canx cany canz cokex cokey cokez trash_box_redx trash_box_redy trash_box_redz trash_box_greenx trash_box_greeny trash_box_greenz trash_box_bluex trash_box_bluey trash_box_bluez Can_S Cok_S trash_box_red_S trash_box_green_S trash_box_blue_S \n";

init_time = true;
elapsedTime = 0;
Get_range = true;

parts2 = my->getParts("NECK_LINK0");
parts1 = my->getParts("DIR_LINK0"); // On Z
parts3 = my->getParts("DIR_LINK1"); // On X


walk_State = false;
walk_Step = 0;

}

double RobotController::onAction(ActionEvent &evt)
{
//printf("Current State  %d \n ",m_state );
  //printf("Current State  %d \n ",m_state );
bool av_hmd = checkService("SIGORS");
if(av_hmd && m_hmd == NULL){
    // サービスに接続
    m_hmd = connectToService("SIGORS");

  }


  else if (!av_hmd && m_hmd != NULL){
   m_hmd = NULL;
  }


  if(init_time  == true)
    {
      take_time = evt.time();
      init_time  = false;
    }

//elapsedTime = evt.time() -  take_time;
//Record_Position();



   switch(m_state)
    {
 case 1: {
           Robot_speed  = Change_Robot_speed;

          //   chooze_task_arm_left(5);
          //   chooze_task_arm_right(5);
          //   printf("got it in case!1 flag1 \n");
          //if (goTo(m_relayPoint1, 0) == true && moveLeftArm() == true && moveRightArm() == true)
          if (goTo(m_relayPoint1, 0) == true)
          {
           m_state = 2;
         //  printf("got it in case!1 \n");
      
          }
          break;
        }
  case 2:   { 
    Robot_speed  = Change_Robot_speed;
          if (goTo(m_relayPoint2, 0) == true) m_state = 3;
          break;
        }
  case 3: {
    Robot_speed  = Change_Robot_speed;
          if (goTo(m_relayFrontTable, 0) == true) m_state = 4;
          break;
        }
  case 4: {  // Test if the cycle is finished or not
          reset_op = false;
          take_action = true;
               if(cycle > 0)
                    {
                      sendMsg("man_000","On_Take");
                      sendMsg("VoiceReco_Service"," Please show me which object to take ");
                      m_state = 30;
                      break;
              }
                else
              {
                      sendMsg("VoiceReco_Service"," Task finished Please press  button Two to restart the cycle ");
                      
                      sendMsg("man_000","restart_on");
                      m_state = 60;
                      break;
                }
       }


      case 30:
      {
     // printf("Start New task \n");

      }

      case 60:
      {
     // printf("Restart the cycle");

      }

   case 5:   {  //Optional case  !!!
         Robot_speed  = Change_Robot_speed;
        if(m_pointedObject=="petbottle")
        {
        if (goTo(m_BottleFront, 0) == true) m_state = 6;
        }

        else if(m_pointedObject=="can")
        {
        if (goTo(m_CanFront, 0) == true)  m_state = 6;
        }
       break;

             }
  case 6:   { //preparation of the arm for grasp
    Robot_speed  = Change_Robot_speed;
          recognizeObjectPosition(m_Object_togo, m_pointedObject);
          if (goTo(m_Object_togo, 45) == true)
          {
           m_state = 7;
         

        }
          break;
        }

  case 7:   { //preparation of the arm for grasp
          chooze_task_arm_left(1);
          if (moveLeftArm() == true) m_state = 8;
          break;
        }
  case 8:   { //move to the object
          Robot_speed  = 1;
          if (goTo(m_Object_togo, 25) == true) m_state = 9;
          break;
        }
  case 9:   { //move arm to grasp the object
          chooze_task_arm_left(2);
          if (moveLeftArm() == true)
          {
           // grasp_left_hand();
           m_state = 10;
          }
          break;
        }
  case 10:   {
          // broadcastMsg("Object_grasped");
          //LOG_MSG(("Object_grasped"));
          chooze_task_Hand_Left(1);
          if (GraspLeft() == true)
          {
            grasp_left_hand();
           m_state = 11;
          }
          
          break;
        }
  case 11:   { //move arm to place in good position for moving
      //    grasp_left_hand();
          chooze_task_arm_left(3);
          if (moveLeftArm() == true)
          m_state = 12;
          break;
        }
  case 12:  {
         // my->setWheelVelocity(-1.4,-1.4);
      /*    chooze_task_arm_left(5);
          if (moveLeftArm() == true)
          {
            Robot_speed  = Change_Robot_speed;
            sendMsg("VoiceReco_Service"," Now I will go to the trashboxes ");
            m_state = 13;
            sleep(1);
          }*/
            chooze_task_arm_left(1);
          if (moveLeftArm() == true)
          {
           // grasp_left_hand();
           m_state = 999;
          }
          break;
        }
  case 999:   { //move to the object
         // Robot_speed  = 1;
          if (goTo(m_Object_togo, 60) == true== true)
          {

        //    sendMsg("VoiceReco_Service"," Please show me which trashbox to use ");
        //     sendMsg("man_000","okput");
        //    sleep(2);
            m_state = 13;
          }
          break;
        }

  case 13:   { //move to the object
         // Robot_speed  = 1;
          if (goTo(m_relayFrontTrash, 0) == true)
          {

        //    sendMsg("VoiceReco_Service"," Please show me which trashbox to use ");
        //     sendMsg("man_000","okput");
        //    sleep(2);
            m_state = 14;
          }
          break;
        }


  case 14: {

   if (goTo(m_relayFrontTable_reset, 0) == true)
          {

            sendMsg("VoiceReco_Service"," Please show me which trashbox to use ");
            sleep(2);
            reset_op = true;
            put_action = true;
            sendMsg("man_000","On_put");
            m_state = 99;
          }
          break;
        }

   case 99:   { //move to the object
         // Robot_speed  = 1;
          
          break;
        }

  case 15:   { //Optional case  !!!
                  Robot_speed  = Change_Robot_speed;
                  if(m_pointedtrash=="recycle")
                  {
                  if (goTo(m_RecycleFront, 0) == true)  m_state = 16;
                  }
                  else if(m_pointedtrash=="burnable")
                  {
                  if (goTo(m_BurnableFront, 0) == true) m_state = 16;
                  }
                  else if(m_pointedtrash=="unburnable")
                  {
                  if (goTo(m_UnburnableFront, 0) == true) m_state = 16;
                  }
                 
                   break;
   }
  case 16:   { //preparation of the arm for grasp
          recognizeObjectPosition(m_Trash_togo, m_pointedtrash);
          if (goTo(m_Trash_togo, 50) == true)
          {
          m_state = 17;

          }
          break;
        }

  case 17:   { //preparation of the arm for grasp
          chooze_task_arm_left(1);
          if (moveLeftArm() == true) m_state = 18;
          break;
        }
  case 18:   {
          Robot_speed  = 1;
          if (goTo(m_Trash_togo, 40) == true) m_state = 19;
          break;
        }
  case 19:   { //move arm to grasp the object
          chooze_task_arm_left(2);
          chooze_task_Hand_Left(0);
         if (GraspLeft() == true)
         {
           m_state = 20;
           release_left_hand();
         }
          break;
        }
  case 20:  {
        //  my->setWheelVelocity(-1.5,-1.5);
          chooze_task_arm_left(0);
          if (moveLeftArm() == true)
          {

            Robot_speed  = Change_Robot_speed;
            m_state = 21;
          }
          break;
        }
  case 21:   { //move to the object
         // Robot_speed  = 1;
          Robot_speed  = Change_Robot_speed;
          if (goTo(m_relayFrontTrash, 0) == true) m_state = 22;
          break;
        }
  case 22:   { //preparation of the arm for grasp
          chooze_task_arm_left(1);
          if (moveLeftArm() == true) m_state = 23;
          break;
        } 
  case 23:   { //move to the object
         // Robot_speed  = 1;
         Robot_speed  = Change_Robot_speed;
          if (goTo(m_relayFrontTable, 0) == true)
          {
            cycle = cycle-1;
           m_state = 4;

          std::vector<std::string>::iterator it;
          it = std::find(m_trashes.begin(), m_trashes.end(), m_pointedObject);
          m_trashes.erase(it);
          m_pointedtrash = "";
          m_pointedObject = "";
          take_action = true;
          put_action = true;
         }
          break;
        }
/// Cancelation step for object  /////
  // object is grasped //// 
 /// 1. Grasped but don't leave  the front table point ///




 case 100:   { 
          stopRobotMove();
        Robot_speed  = Change_Robot_speed;
        if(m_pointedObject=="petbottle")
          
        {
       // if (goTo(m_BottleFront, 0) == true) m_state = 6;
        Object_reset.x(m_BottleReset.x());
        Object_reset.y(m_BottleReset.y());
        Object_reset.z(m_BottleReset.z());

        }

      
        else if(m_pointedObject=="can")
        {
          //Object_reset = m_CanReset;
        Object_reset.x(m_CanReset.x());
        Object_reset.y(m_CanReset.y());
        Object_reset.z(m_CanReset.z());
      //  if (goTo(m_CanFront, 0) == true)  m_state = 6;
        }
        m_state = 101;
       break;
        }

  case 101:   { //preparation of the arm for grasp
         Robot_speed  = Change_Robot_speed;
    //      chooze_task_arm_left(1);
    //      if (moveLeftArm() == true) m_state = 102;
          break;
        }
  case 102:   {
          Robot_speed  = 1;
          if (goTo(Object_reset, 30) == true) m_state = 103;
          break;
        }
  case 103:   { //move arm to grasp the object
    //      chooze_task_arm_left(2);
         
       //   if (moveLeftArm() == true) m_state = 104;
          break;
        }
  case 104:  {
       //   my->setWheelVelocity(-1.5,-1.5);
          chooze_task_arm_left(2);
           release_left_hand();
          if (moveLeftArm() == true)
          {
            Robot_speed  = Change_Robot_speed;
            m_state = 105;
          }
          break;
        }

  case 105:   { //move to the object
         // Robot_speed  = 1;
         // m_trashes.push_back(m_pointedObject);
          Robot_speed  = Change_Robot_speed;
          //cycle++;
          m_pointedObject = "";
         // m_pointedtrash = "";
          Robot_speed  = Change_Robot_speed;
          if (goTo(m_relayFrontTable_reset, 0) == true) m_state = 106;
          break;
        }
  case 106:   { //preparation of the arm for grasp
        chooze_task_arm_left(0);
          if (moveLeftArm() == true) m_state = 107;
          break;
        }        
 case 107:   { 
 
           Robot_speed  = Change_Robot_speed;
          if (goTo( m_relayFrontTable, 0) == true)
          {
           m_state = 4;
           reset_out = false;
          }
         // m_pointedObject="";
          break;
         }

 /// 2. Grasped and left the front table point ///





// Object is not grapsed yet //
 case 25:   { //preparation of the arm for grasp
             stopRobotMove();
          Robot_speed  = Change_Robot_speed;
     //     chooze_task_arm_left(3);
      //    if (moveLeftArm() == true) m_state = 26;
          break;
        } 
 case 26:   { //preparation of the arm for grasp
          Robot_speed  = Change_Robot_speed;
    //      chooze_task_arm_left(5);
     //     if (moveLeftArm() == true) m_state = 27;
          break;
        } 
  case 27: { //move to the object
         // Robot_speed  = 1;

          Robot_speed  = Change_Robot_speed;
          if (goTo( m_relayFrontTable_reset, 0) == true) m_state = 28;
          m_pointedObject="";
          break;
        }

              
 case 28:  { 
           Robot_speed  = Change_Robot_speed;
          if (goTo( m_relayFrontTable, 0) == true)
          {
           m_state = 4;
           reset_out = false;
          }
         // m_pointedObject="";
        
          break;
         }


/// Cancelation step for trash  /////
 case 200:   { //preparation of the arm for grasp
          Robot_speed  = Change_Robot_speed;
     //     chooze_task_arm_left(5);
     //     if (moveLeftArm() == true) m_state = 201;
          break;
        } 
  case 201:   { //move to the object
         // Robot_speed  = 1;

          Robot_speed  = Change_Robot_speed;
          if (goTo( m_relayFrontTrash, 0) == true) m_state = 202;
         // m_pointedObject="";
          break;
        }

              
 case 202:   { 
 
           Robot_speed  = Change_Robot_speed;
          if (goTo(m_relayFrontTable_reset, 0) == true) 
          {
            m_state = 14;
            reset_out = false;
          }
         // m_pointedObject="";
          m_pointedtrash = "";
          break;
         }

  }




  return 0.002;
}


void RobotController::onRecvMsg(RecvMsgEvent &evt)
{
  std::string sender = evt.getSender();

  // 送信者がゴミ認識サービスの場合
  char *all_msg = (char*)evt.getMsg();
  std::string msg;
  msg= evt.getMsg();

 // printf("all_msg=\n%s\n",all_msg);

  std::string ss = all_msg;
  //ヘッダーの取り出し
  int strPos1 = 0;
  int strPos2;
  std::string headss;
  std::string tmpss;
  strPos2 = ss.find(" ", strPos1);
  headss.assign(ss, strPos1, strPos2-strPos1);
 
 std::string task;
  std::string joint;
 std::string value;
  //LOG_MSG(("%s: %s",sender.c_str(), msg.c_str()));
  std::size_t found=0;
  std::size_t found2=0;
  std::size_t found3= msg.size();  
found = msg.find("::",found2);
if (found != std::string::npos){
    joint = msg.substr(found2,found-found2);
value = msg.substr(found+2,found3);
 //std::cout << "the the value  is "+ value << std::endl ;
int val = atoi(value.c_str());
double angle = (val*3.14) / 180;
//std::cout << "the angle  is  "<< val << std::endl ;
 my->setJointAngle(joint.c_str(),angle);
 }



 else if(msg == "go" && m_state == 0)
  {
        m_state = 1 ;      
        sendMsg("VoiceReco_Service","Let's start the clean up task\n");
        printf("got it in go \n");
  }
  

  else if(msg == "do" && m_state == 30 && take_action == true) 
  {
      //  printf("Kinect is started on object\n");
   take_action = false;
        m_pointedObject = getPointedObjectName(m_avatar);
        std::string msg_ob = " I will take " + m_pointedObject ;
        sendMsg("VoiceReco_Service",msg_ob);
        sleep(2);
        m_state = 5;
  }
  else if(msg == "do" && m_state == 99 && put_action == true )
  {
      //  printf("Kinect is started on trash\n");
       put_action = false;
        m_pointedtrash = getPointedTrashName(m_avatar);
        std::string  msg_trash = " Ok I will put "+ m_pointedObject+" in "+ m_pointedtrash +" trashbox ";
        sendMsg("VoiceReco_Service",msg_trash);
        sleep(2);
       // sleep(2);
        m_state = 15;
  }
  else if(msg == "restart" && m_state == 60)
  {
  m_trashes.push_back("petbottle");
 // m_trashes.push_back("mugcup");
  m_trashes.push_back("can");
  Vector3d  m_BottleReset = Vector3d(40.0, 59.15, 50.0);
 // Vector3d  m_MuccupReset = Vector3d(0.0, 59.15, 50.0);
  Vector3d  m_CanReset = Vector3d(-40.0, 55.250, 50.0);
  Vector3d  m_RobotPos = Vector3d(100.0, 30.0,-100.0);
  Rotation rot;
     rot.setQuaternion(1, 0, 0, 0);
  SimObj *target = this->getObj("petbottle");

  target->setPosition(m_BottleReset);
   target->setRotation(rot);
 // target = this->getObj("mugcup");
  // target->setRotation(rot);
//  target->setPosition(m_MuccupReset);
  target = this->getObj("can");
   target->setRotation(rot);
  target->setPosition(m_CanReset);
  cycle = 3;
  m_state = 4;
  my->setPosition(m_RobotPos);

     my->setWheelVelocity(0.0,0.0);
     my->setRotation(rot);
     /*
     my->setJointVelocity("LARM_JOINT0", 0.0,0.0);
     my->setJointAngle("LARM_JOINT0", 0.0);
     my->setJointVelocity("LARM_JOINT1", 0.0,0.0);
     my->setJointAngle("LARM_JOINT1", 0.0);
     my->setJointVelocity("LARM_JOINT3", 0.0,0.0);
     my->setJointAngle("LARM_JOINT3", 0.0);
     my->setJointVelocity("LARM_JOINT4", 0.0,0.0);
     my->setJointAngle("LARM_JOINT4", -1.57);
     my->setJointVelocity("LARM_JOINT5", 0.0,0.0);
     my->setJointAngle("LARM_JOINT5", 0.0);
     my->setJointVelocity("LARM_JOINT6", 0.0,0.0);
     my->setJointAngle("LARM_JOINT6", 0.0);
     my->setJointVelocity("LARM_JOINT7", 0.0,0.0);
     my->setJointAngle("LARM_JOINT7", 0.0);
   */
  }
  else if(msg == "reset" && m_grasp_left == true && reset_op == false && reset_out == false)
  {
   reset_out = true;
    m_state = 100;
   sendMsg("man_000","reset_put");
/*
  m_trashes.push_back(m_pointedObject);
  Vector3d  m_BottleReset = Vector3d(40.0, 59.15, 50.0);
  Vector3d  m_MuccupReset = Vector3d(0.0, 52.15, 50.0);
  Vector3d  m_CanReset = Vector3d(-40.0, 59.15, 50.0);
  Vector3d  m_RobotPos = Vector3d(100.0, 30.0,-100.0);
   Rotation rot;
     rot.setQuaternion(1, 0, 0, 0);

  SimObj *target = this->getObj(m_pointedObject.c_str());
  target->setRotation(rot);
     if( m_grasp_left == true)
     {
            CParts * parts = my->getParts("LARM_LINK7");
            parts->releaseObj();
             m_grasp_left = false;

     }
  if(m_pointedObject == "petbottle" )
  {
 target->setPosition(m_BottleReset);
  m_trashes.push_back("petbottle");
            
  }
  else if (m_pointedObject == "mugcup")
  {
 target->setPosition(m_MuccupReset);
   m_trashes.push_back("mugcup");
  }
  else if(m_pointedObject == "can")
  {
target->setPosition(m_CanReset);
  m_trashes.push_back("can");
  }

  cycle++;
  m_state = 30;

     my->setWheelVelocity(0.0,0.0);
     my->setPosition(m_RobotPos);

     my->setRotation(rot);
     my->setJointVelocity("LARM_JOINT0", 0.0,0.0);
     my->setJointAngle("LARM_JOINT0", 0.0);
     my->setJointVelocity("LARM_JOINT1", 0.0,0.0);
     my->setJointAngle("LARM_JOINT1", 0.0);
     my->setJointVelocity("LARM_JOINT3", 0.0,0.0);
     my->setJointAngle("LARM_JOINT3", 0.0);
     my->setJointVelocity("LARM_JOINT4", 0.0,0.0);
     my->setJointAngle("LARM_JOINT4", -1.57);
     my->setJointVelocity("LARM_JOINT5", 0.0,0.0);
     my->setJointAngle("LARM_JOINT5", 0.0);
     my->setJointVelocity("LARM_JOINT6", 0.0,0.0);
     my->setJointAngle("LARM_JOINT6", 0.0);
     my->setJointVelocity("LARM_JOINT7", 0.0,0.0);
     my->setJointAngle("LARM_JOINT7", 0.0);
     m_pointedObject = "";
     m_pointedtrash = "";
     */
  }

 

   else if(msg == "reset" && m_grasp_left == false && reset_op == false && reset_out == false)
   {
    reset_out = true;
    m_state = 25;
    sendMsg("man_000","reset_take");

   }
      else if(msg == "reset" && m_grasp_left == true && reset_op == true && reset_out == false)
   {
    reset_out = true;
    m_state = 200;
       sendMsg("man_000","reset_put");
   }


   else if(headss == "ORS_DATA"){
    //HMDデータによる頭部の動き反映
    moveHeadByHMD(ss);
    std::cout << "Oculus Message   is  "<< ss << std::endl ; 
  }

  else
  {
        printf("Message is not accepted\n");
  }



}


std::string RobotController::getPointedObjectName(std::string entName)
{
  // 発話者の名前からSimObjを取得します
  SimObj *tobj = getObj(entName.c_str());

  // メッセージ送信者の左肘関節の位置を取得します
  Vector3d jpos;
  if(!tobj->getJointPosition(jpos, "REYE_JOINT1")) {
    LOG_ERR(("failed to get joint position"));
    return "";
  }
  // メッセージ送信者の左肘から左手首をつなぐベクトルを取得します
  Vector3d jvec;
  if(!tobj->getPointingVector(jvec,"REYE_JOINT1","RARM_JOINT7")) {
    LOG_ERR(("failed to get pointing vector"));
    return "";
  }

  double distance = 0.0;
  std::string objName = "";

  // 全ゴミオブジェクトでループします
  int trashSize = m_trashes.size();
  for(int i = 0; i < trashSize; i++) {

  // エンティティの位置を取得します
  SimObj *obj = getObj(m_trashes[i].c_str());
  Vector3d objVec;
  obj->getPosition(objVec);

  // エンティティと左肘関節を結ぶベクトルを作成します
  objVec -= jpos;

  // cos角度が不の場合（指差した方向と反対側にある場合)は対象から外します
  double cos = jvec.angle(objVec);
  //if(cos < 0)
  //  continue;

  // 指差した方向ベクトルまでの最短距離の計算
  double theta = acos(cos);
  double tmp_distance = sin(theta) * objVec.length();

  // 最小距離の場合は名前、距離を保存しておく
  if(tmp_distance < distance || distance == 0.0){
    distance = tmp_distance;
    objName = obj->name();
    }
  }
  // エンティティでループして最も近いオブジェクトの名前を取得する
  return objName;
}

std::string RobotController::getPointedTrashName(std::string entName)
{
  // 発話者の名前からSimObjを取得します
  SimObj *tobj = getObj(entName.c_str());

  // メッセージ送信者の左肘関節の位置を取得します
  Vector3d jpos;
  if(!tobj->getJointPosition(jpos, "RARM_JOINT4")) {
    LOG_ERR(("failed to get joint position"));
    return "";
  }
  // メッセージ送信者の左肘から左手首をつなぐベクトルを取得します
  Vector3d jvec;
  if(!tobj->getPointingVector(jvec, "RARM_JOINT4", "RARM_JOINT7")) {
    LOG_ERR(("failed to get pointing vector"));
    return "";
  }

  double distance = 0.0;
  std::string objName = "";

  // 全ゴミオブジェクトでループします
  int trashboxSize = m_trashboxs.size();
  for(int i = 0; i < trashboxSize; i++) {

  // エンティティの位置を取得します
  SimObj *obj = getObj(m_trashboxs[i].c_str());
  Vector3d objVec;
  obj->getPosition(objVec);

  // エンティティと左肘関節を結ぶベクトルを作成します
  objVec -= jpos;

  // cos角度が不の場合（指差した方向と反対側にある場合)は対象から外します
  double cos = jvec.angle(objVec);
  //if(cos < 0)
  //  continue;

  // 指差した方向ベクトルまでの最短距離の計算
  double theta = acos(cos);
  double tmp_distance = sin(theta) * objVec.length();

  // 最小距離の場合は名前、距離を保存しておく
  if(tmp_distance < distance || distance == 0.0){
    distance = tmp_distance;
    objName = obj->name();
    }
  }
  // エンティティでループして最も近いオブジェクトの名前を取得する
  return objName;
}

bool RobotController::recognizeTrash(Vector3d &pos, std::string &name)
{
  // 候補のゴミが無い場合
  if(m_trashes.empty()){
    return false;
  }

  // ここでは乱数を使ってゴミを決定します
  int trashNum = rand() % m_trashes.size();

  // ゴミの名前と位置を取得します
  name = m_trashes[trashNum];
  SimObj *trash = getObj(name.c_str());

  // ゴミの位置取得
  trash->getPosition(pos);
  return true;
}
/*
void RobotController::onCollision(CollisionEvent &evt)
{
  if (m_grasp == false){
    typedef CollisionEvent::WithC C;
    //触れたエンティティの名前を得ます
    const std::vector<std::string> & with = evt.getWith();
    // 衝突した自分のパーツを得ます
    const std::vector<std::string> & mparts = evt.getMyParts();
    //　衝突したエンティティでループします
    for(int i = 0; i < with.size(); i++){
      //右手に衝突した場合
      if(mparts[i] == "RARM_LINK7"){
        //自分を取得
        SimObj *my = getObj(myname());
        //自分の手のパーツを得ます
        CParts * parts = my->getParts("RARM_LINK7");
     //   if(parts->graspObj(with[i])){
      //    m_grasp = true;
     //   }
      }
    }
  }
}
*/
double RobotController::rotateTowardObj(Vector3d pos, double velocity, double now)
{
  // 自分の位置の取得
  Vector3d myPos;
  //m_my->getPosition(myPos);
  my->getPartsPosition(myPos,"RARM_LINK2");

  // 自分の位置からターゲットを結ぶベクトル
  Vector3d tmpp = pos;
  tmpp -= myPos;

  // y方向は考えない
  tmpp.y(0);

  // 自分の回転を得る
  Rotation myRot;
  my->getRotation(myRot);

  // エンティティの初期方向
  Vector3d iniVec(0.0, 0.0, 1.0);

  // y軸の回転角度を得る(x,z方向の回転は無いと仮定)
  double qw = myRot.qw();
  double qy = myRot.qy();

  double theta = 2*acos(fabs(qw));

  if(qw*qy < 0)
    theta = -1*theta;

  // z方向からの角度
  double tmp = tmpp.angle(Vector3d(0.0, 0.0, 1.0));
  double targetAngle = acos(tmp);

  // 方向
  if(tmpp.x() > 0) targetAngle = -1*targetAngle;
  targetAngle += theta;

  if(targetAngle == 0.0){
    return 0.0;
  }
  else {
    // 回転すべき円周距離
    double distance = m_distance*PI*fabs(targetAngle)/(2*PI);

    // 車輪の半径から移動速度を得る
    double vel = m_radius*velocity;

    // 回転時間(u秒)
    double time = distance / vel;

    // 車輪回転開始
    if(targetAngle > 0.0){
      my->setWheelVelocity(velocity, -velocity);
    }
    else{
      my->setWheelVelocity(-velocity, velocity);
    }

    return now + time;
  }
}

// object まで移動
double RobotController::goToObj(Vector3d pos, double velocity, double range, double now)
{
  Vector3d myPos;
  //m_my->getPosition(myPos);
  my->getPartsPosition(myPos,"RARM_LINK2");

  // 自分の位置からターゲットを結ぶベクトル
  pos -= myPos;

  // y方向は考えない
  pos.y(0);

  // 距離計算
  double distance = pos.length() - range;

  // 車輪の半径から移動速度を得る
  double vel = m_radius*velocity;

  // 移動開始
  my->setWheelVelocity(velocity, velocity);

  // 到着時間取得
  double time = distance / vel;

  return now + time;
}







void RobotController::Walk_robot()
{


//tab_joint_walk[14][8]

if (  walk_Step > 13 )
{

  walk_Step = 0 ;
}



 bool j0 = false, j1 = false , j3 = false, j4 = false, j5 = false, j6 = false, j2 = false , j7 = false;

  if(joint_walk[0] != tab_joint_walk[walk_Step][0] )
  {
    if(joint_walk[0] < tab_joint_walk[walk_Step][0] && tab_joint_walk[walk_Step][0]-joint_left[0] > error_angle_walk)
    {
      my->setJointVelocity("RLEG_JOINT0", joint_veloc_walk, 0.0);
      joint_walk[0] = my->getJointAngle("RLEG_JOINT0");
    }
    else if(joint_walk[0] > tab_joint_walk[walk_Step][0] && joint_walk[0]-tab_joint_walk[walk_Step][0] > error_angle_walk)
     {
      my->setJointVelocity("RLEG_JOINT0", -joint_veloc_walk, 0.0);
      joint_walk[0] = my->getJointAngle("RLEG_JOINT0");
     }
    else
    {
      my->setJointVelocity("LARM_JOINT0", 0.0, 0.0);
      j0 = true;
    }

  }
  else j0 = true;

  if(joint_walk[1] != tab_joint_walk[walk_Step][1] )
  {
    if(joint_walk[1] < tab_joint_walk[walk_Step][1] && tab_joint_walk[walk_Step][1]-joint_walk[1] > error_angle_walk)
    {
      my->setJointVelocity("RLEG_JOINT3", joint_veloc_walk, 0.0);
      joint_walk[1] = my->getJointAngle("RLEG_JOINT3");
    }
    else if(joint_walk[1] > tab_joint_walk[walk_Step][1] && joint_walk[1]-tab_joint_walk[walk_Step][1] > error_angle_walk)
       {
        my->setJointVelocity("RLEG_JOINT3", -joint_veloc_walk, 0.0);
        joint_walk[1] = my->getJointAngle("RLEG_JOINT3");
       }
       else
       {
         my->setJointVelocity("RLEG_JOINT3", 0.0, 0.0);
         j1 = true;
       }
  }
  else j1 = true;


  if(joint_walk[2] != tab_joint_walk[walk_Step][2] )
  {
    if(joint_walk[2] < tab_joint_walk[walk_Step][2] && tab_joint_walk[walk_Step][2]-joint_walk[2] > error_angle_walk)
    {
      my->setJointVelocity("RLEG_JOINT1", joint_veloc_walk, 0.0);
      joint_walk[2] = my->getJointAngle("RLEG_JOINT1");
    }
    else if(joint_walk[2] > tab_joint_walk[walk_Step][2] && joint_walk[2]- tab_joint_walk[walk_Step][2] > error_angle_walk)
       {
        my->setJointVelocity("LARM_JOINT2", -joint_veloc_walk, 0.0);
        joint_walk[2] = my->getJointAngle("RLEG_JOINT1");
       }
       else
       {
         my->setJointVelocity("RLEG_JOINT1", 0.0, 0.0);
         j2 = true;
       }
  }
  else j2 = true;




  if(joint_walk[3] != tab_joint_walk[walk_Step][3] )
  {
    if(joint_walk[3] < tab_joint_walk[walk_Step][3] && tab_joint_walk[walk_Step][3]-joint_walk[3] > error_angle_walk)
    {
      my->setJointVelocity("LLEG_JOINT0", joint_veloc_walk, 0.0);
      joint_walk[3] = my->getJointAngle("LLEG_JOINT0");
    }
    else if(joint_walk[3] > tab_joint_walk[walk_Step][3] && joint_walk[3]-tab_joint_walk[walk_Step][3] > error_angle_walk)
       {
        my->setJointVelocity("LLEG_JOINT0", -joint_veloc_walk, 0.0);
        joint_walk[3] = my->getJointAngle("LLEG_JOINT0");
       }
       else
       {
         my->setJointVelocity("LLEG_JOINT0", 0.0, 0.0);
         j3 = true;
       }
  }
  else j3 = true;

  if(joint_walk[4] != tab_joint_walk[walk_Step][4] )
  {
    if(joint_walk[4] < tab_joint_walk[walk_Step][4] && tab_joint_walk[walk_Step][4]-joint_walk[4] > error_angle_walk)
    {
      my->setJointVelocity("LLEG_JOINT3", joint_veloc_walk, 0.0);
      joint_walk[4] = my->getJointAngle("LLEG_JOINT3");
    }
    else if(joint_walk[4] > tab_joint_walk[walk_Step][4] && joint_walk[4]-tab_joint_walk[walk_Step][4] > error_angle_walk)
       {
        my->setJointVelocity("LLEG_JOINT3", -joint_veloc_walk, 0.0);
        joint_walk[4] = my->getJointAngle("LLEG_JOINT3");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
         j4 = true;
       }
  }
  else j4 = true;

  if(joint_walk[5] != tab_joint_walk[walk_Step][5] )
  {
    if(joint_walk[5] < tab_joint_walk[walk_Step][5] && tab_joint_walk[walk_Step][5]-joint_walk[5] > error_angle_walk)
    {
      my->setJointVelocity("LLEG_JOINT1", joint_veloc_walk, 0.0);
      joint_walk[5] = my->getJointAngle("LLEG_JOINT1");
    }
    else if(joint_walk[5] > tab_joint_walk[walk_Step][5] && joint_walk[5]-tab_joint_walk[walk_Step][5] > error_angle_walk)
       {
        my->setJointVelocity("LLEG_JOINT1", -joint_veloc_walk, 0.0);
        joint_walk[5] = my->getJointAngle("LLEG_JOINT1");
       }
       else
       {
         my->setJointVelocity("LLEG_JOINT1", 0.0, 0.0);
         j5 = true;
       }
  }
  else j5 = true;

  if(joint_walk[6] != tab_joint_walk[walk_Step][6] )
  {
    if(joint_walk[6] < tab_joint_walk[walk_Step][6] && tab_joint_walk[walk_Step][6]-joint_walk[6] > error_angle_walk)
    {
      my->setJointVelocity("LARM_JOINT3", joint_veloc_walk, 0.0);
      joint_walk[6] = my->getJointAngle("LARM_JOINT3");
    }
    else if(joint_walk[6] > tab_joint_walk[walk_Step][6] && joint_walk[6]-tab_joint_walk[walk_Step][6] > error_angle_walk)
       {
        my->setJointVelocity("LARM_JOINT3", -joint_veloc_walk, 0.0);
        joint_walk[6] = my->getJointAngle("LARM_JOINT3");
       }
       else
       {
         my->setJointVelocity("LARM_JOINT3", 0.0, 0.0);
         j6 = true;
       }
  }
  else j6 = true;

  if(joint_walk[7] != tab_joint_walk[walk_Step][7] )
  {
    if(joint_walk[7] < tab_joint_walk[walk_Step][7] && tab_joint_walk[walk_Step][7]-joint_walk[6] > error_angle_walk)
    {
      my->setJointVelocity("RARM_JOINT3", joint_veloc_walk, 0.0);
      joint_walk[7] = my->getJointAngle("RARM_JOINT3");
    }
    else if(joint_walk[7] > tab_joint_walk[walk_Step][7] && joint_walk[7]-tab_joint_walk[walk_Step][7] > error_angle_walk)
       {
        my->setJointVelocity("RARM_JOINT3", -joint_veloc_walk, 0.0);
        joint_walk[7] = my->getJointAngle("RARM_JOINT3");
       }
       else
       {
         my->setJointVelocity("RARM_JOINT3", 0.0, 0.0);
         j7 = true;
       }
  }
  else j7 = true;

  if (j0 == true && j1 == true && j3 == true && j4 == true && j5 == true && j6 == true && j2 == true  && j7 == true)
   {
   // return true;
   walk_Step += 1 ;

   }


  else
  {
  //  return false;
  }


//  return false;


  

  
}




//自身のインスタンスをSIGVerseに返します
extern "C" Controller * createController() {
  return new RobotController;
}
