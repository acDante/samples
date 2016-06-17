#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <unistd.h>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ros/console.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"



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
    vec_res.x(kvec.y()*svec.z() - kvec.z()*svec.y());                             
    vec_res.y(kvec.z()*svec.x() - kvec.x()*svec.z());                             
    vec_res.z(kvec.x()*svec.y() - kvec.y()*svec.x()); 

    return  vec_res;                           
}

Vector3d NORMAL(Vector3d kvec)          /* Normalize*/              
{
     Vector3d vec_res ;
   double sum = sqrt(kvec.x()*kvec.x()+kvec.y()*kvec.y()+kvec.z()*kvec.z());
   vec_res.x(kvec.x()/sum);
   vec_res.y(kvec.y()/sum);
   vec_res.z(kvec.z()/sum);
  return  vec_res; 
}


Vector3d MINV(Vector3d kvec, Vector3d svec)          /* CROSS Vector Product */              
{  

Vector3d vec_res ;
   vec_res.x(kvec.x()-svec.x());
   vec_res.y(kvec.y()-svec.y());
   vec_res.z(kvec.z()-svec.z());
  return  vec_res;
}


Vector3d PLSV(Vector3d kvec, Vector3d svec)          /* CROSS Vector Product */              
{  
Vector3d vec_res ;
   vec_res.x(kvec.x()+svec.x());
   vec_res.y(kvec.y()+svec.y());
   vec_res.z(kvec.z()+svec.z());
  return  vec_res;
}

Vector3d MULT(Vector3d kvec, double v)          /* CROSS Vector Product */              
{  
Vector3d vec_res ;
   vec_res.x(kvec.x()*v);
   vec_res.y(kvec.y()*v);
   vec_res.z(kvec.z()*v);
  return  vec_res;
}

#define error_angle 0.12
#define error_distance 1.1
#define error_angle_arm 0.07

class MobileController : public Controller {
public:
  void onInit(InitEvent &evt);
  double onAction(ActionEvent&);
  void onRecvMsg(RecvMsgEvent &evt);
 // void onCollision(CollisionEvent &evt); 
  Vector3d intersectLine();
 // void OnCmdVel( const geometry_msgs::TwistConstPtr &msg);
    //function use by other function
  void stopRobotMove(void);

    void recognizeObjectPosition(Vector3d &pos, std::string &name);
    double getAngularXonVect(Vector3d pos, Vector3d mypos);
    double getDistoObj(Vector3d pos, Vector3d pos2);
    double getDist2D(Vector3d pos, Vector3d pos2);
    double getDist3D(Vector3d pos, Vector3d pos2);
    double getRoll(Rotation rot);
    double getPitch(Rotation rot);
    double getYaw(Rotation rot);


    bool goTo(Vector3d pos, double rangeToPoint);
    double goToObj(Vector3d pos, double vel, double range, double now);
private:
  double maxForce;
  RobotObj *my;
  double m_radius;
  double m_distance;
  double wheel_sep;
  double Wheel_one;
  double Wheel_two;
  ros::Subscriber cmd_vel_sub ;
  ros::Publisher cmd_vel_pub;
  geometry_msgs::Twist Vel_pub;
  std::string avatarName;
  SimObj *HumanAvatar ;
  std::string Checkpoint_coo;
  SimObj *Checkpoint;
  Vector3d target;
  double Robot_speed ;
  int m_state;
  Vector3d target_update;
 // ros::NodeHandle Turtle_n;
  double control_speed;
  double control_turn;
}; 



void MobileController::stopRobotMove(void)
{
  my->setWheelVelocity(0.0, 0.0);
}



/************************************************************************************/
/*******Find roll,pitch,yaw, lenght between to point, angle between to point*********/
/************************************************************************************/

double MobileController::getRoll(Rotation rot)
{
  // get angles arround y-axis
  double qw = rot.qw();
  double qx = rot.qx();
  double qy = rot.qy();
  double qz = rot.qz();

  double roll  = atan2(2*qy*qw - 2*qx*qz, 1 - 2*qy*qy - 2*qz*qz);

  return roll;
}

double MobileController::getPitch(Rotation rot)
{
  // get angles arround y-axis
  double qw = rot.qw();
  double qx = rot.qx();
  double qy = rot.qy();
  double qz = rot.qz();

  double pitch = atan2(2*qx*qw - 2*qy*qz, 1 - 2*qx*qx - 2*qz*qz);

  return pitch;
}

double MobileController::getYaw(Rotation rot)
{
    // get angles arround y-axis
  double qw = rot.qw();
  double qx = rot.qx();
  double qy = rot.qy();
  double qz = rot.qz();

  double yaw   = asin(2*qx*qy + 2*qz*qw);

  return yaw;
}

double MobileController::getDistoObj(Vector3d pos, Vector3d pos2)
{
  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= pos2;

  // measure actual distance
  double distance = sqrt(l_pos.x()*l_pos.x()+l_pos.z()*l_pos.z());

  return distance;
}

double MobileController::getAngularXonVect(Vector3d pos, Vector3d mypos)
{
  double targetAngle;

  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= mypos;

  // rotation angle from z-axis to vector
  targetAngle = atan(l_pos.x()/l_pos.z());

  // direction
  if (l_pos.z() < 0 && l_pos.x()>=0)
    targetAngle = M_PI+targetAngle;
  else if (l_pos.z()<0 && l_pos.x()<0)
    targetAngle = targetAngle-M_PI;

  return targetAngle;
}

double MobileController::getDist2D(Vector3d pos, Vector3d pos2)
{
  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= pos2;

  return sqrt(l_pos.x()*l_pos.x()+l_pos.z()*l_pos.z());
}

double MobileController::getDist3D(Vector3d pos, Vector3d pos2)
{
  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= pos2;

  return sqrt(l_pos.x()*l_pos.x()+l_pos.z()*l_pos.z()+l_pos.y()*l_pos.y());
}



/************************************************************************************/


/************************************************************************************/
/***************************Move the robot in the world******************************/
/************************************************************************************/

bool MobileController::goTo(Vector3d pos, double rangeToPoint)
{
  double speed_linear;
  double speed_angular;
  geometry_msgs::Twist vel;
  Vector3d ownPosition;
  my->getPosition(ownPosition);

  Rotation ownRotation;
  my->getRotation(ownRotation);

  double angle = getAngularXonVect(pos, ownPosition);
  double dist = getDistoObj(pos,ownPosition);
  double roll = getRoll(ownRotation);
  
   speed_linear = fabs(dist-rangeToPoint);
     if(speed_linear > 15)
        {
        speed_linear = 0.20;
        }
     else
        {
        speed_linear = speed_linear/100;
        }

 speed_angular = fabs(angle-roll)*4;
/*
      if ()
      {
       
      }
      else ()
      {
       
      }
*/
if(speed_linear > control_speed)
{
control_speed = std::min( speed_linear, control_speed + 0.02 );
}
else if(speed_linear < control_speed)
{
 control_speed = std::max( speed_linear, control_speed - 0.02 );

}
else
{
 control_speed = speed_linear;
}



if(speed_angular/4 > control_turn)
{
control_turn = std::min( speed_angular/4, control_turn + 0.1 );
}
else if(speed_angular/4 < control_speed)
{
 control_turn = std::max( speed_angular/4, control_turn - 0.1 );

}
else
{
 control_turn = speed_angular/4;
}





  if (angle > 3 || angle < -3) angle = M_PI;

  // error on angle
  if ((angle-roll)>-error_angle && (angle-roll)<error_angle)
    // error on distance
    if (dist-rangeToPoint < error_distance && dist-rangeToPoint > -error_distance)
      {
        stopRobotMove();
      vel.angular.z = 0;
      vel.linear.x = 0;
      control_speed = 0;
      control_turn = 0;
      Vel_pub = vel;
    //  cmd_vel_pub.publish(Vel_pub);
        return true;

      }
    else
      {


  std::cout << "the linear speed is  " <<  speed_linear << std::endl;      

        if (dist-rangeToPoint < 5)
          if ( dist-rangeToPoint > 0 ) 
          {
            my->setWheelVelocity(1, 1);
                vel.angular.z = 0;
                vel.linear.x = control_speed;
                Vel_pub = vel;
          }

          else
            {
            my->setWheelVelocity(-1, -1);
                vel.angular.z = 0;
                vel.linear.x = -control_speed;
                Vel_pub = vel;
            }

        else if ( dist-rangeToPoint > 0 )
        {
          my->setWheelVelocity(Robot_speed , Robot_speed );
                vel.angular.z = 0;
                vel.linear.x = control_speed;
                Vel_pub = vel;
        }

        else
        {
          my->setWheelVelocity(-Robot_speed , -Robot_speed );
                vel.angular.z = 0;
                vel.linear.x = -control_speed;
                Vel_pub = vel;
        }

     // Vel_pub = vel;
   //   cmd_vel_pub.publish(Vel_pub);
        return false;
      }
  else
    {
    vel.linear.x = Vel_pub.linear.x;
      if(fabs(vel.linear.x) > 0.03)
      {
        if(vel.linear.x > 0.02)
       vel.linear.x = vel.linear.x - 0.01;
        if(vel.linear.x < - 0.02)
          vel.linear.x = vel.linear.x + 0.01;
      }
      else
        vel.linear.x = 0;

control_speed = vel.linear.x;
 std::cout << "the angular speed is  " <<   speed_angular << std::endl;    
      if (speed_angular/4 > 0.3)
        if (angle < -M_PI_2 && roll > M_PI_2)
        {
          my->setWheelVelocity(-control_turn/2, control_turn/2);
                vel.angular.z = control_turn;
             //   vel.linear.x = 0.05;
                Vel_pub = vel;
        }
        else if (angle > M_PI_2 && roll < -M_PI_2)
        {
          my->setWheelVelocity(control_turn/2, -control_turn/2);
                vel.angular.z = -control_turn;
              //  vel.linear.x = 0.05;
                Vel_pub = vel;
        }
        else if (angle < roll)
        {
          my->setWheelVelocity(control_turn/2, -control_turn/2);
                vel.angular.z = -control_turn;
              //  vel.linear.x = 0.05;
                Vel_pub = vel;
        }
        else
        {
          my->setWheelVelocity(-control_turn/2, control_turn/2);
                vel.angular.z = control_turn;
            //    vel.linear.x = 0.05;
                Vel_pub = vel;
        }
      else if (angle < -M_PI_2 && roll > M_PI_2)
      {
        my->setWheelVelocity(-control_turn/2, control_turn/2);
                vel.angular.z = control_turn;
            //    vel.linear.x = 0.05;
                Vel_pub = vel;
      }
      else if (angle > M_PI_2 && roll < -M_PI_2)
      {
        my->setWheelVelocity(control_turn/2, -control_turn/2);
                vel.angular.z = -control_turn;
          //      vel.linear.x = 0.05;
                Vel_pub = vel;
      }
      else if (angle < roll)
      {
        my->setWheelVelocity(control_turn/2, -control_turn/2);
                vel.angular.z = -control_turn;
           //     vel.linear.x = 0.05;
                Vel_pub = vel;
      }
      else
      {
        my->setWheelVelocity(-control_turn/2, control_turn/2);
                vel.angular.z = control_turn;
        //        vel.linear.x = 0.05;
                Vel_pub = vel;
      }

     // Vel_pub = vel;
    //  cmd_vel_pub.publish(Vel_pub);

      return false;
    }
    //  Vel_pub = vel;
   //   cmd_vel_pub.publish(Vel_pub); 
  return false;
}

/************************************************************************************/



// object まで移動
double MobileController::goToObj(Vector3d pos, double velocity, double range, double now)
{
  Vector3d myPos;
  //my->getPosition(myPos);
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

/************************************************************************************/





// Callback function which is called when messages comes
void MobileController::onRecvMsg(RecvMsgEvent &evt)
{


}
 
Vector3d MobileController::intersectLine() {

    Vector3d a; 
    HumanAvatar->getJointPosition(a, "RARM_JOINT4");
    Vector3d b;
    HumanAvatar->getJointPosition(b, "RARM_JOINT6");
    Vector3d n = Vector3d(0, 1, 0);
    Vector3d zero = Vector3d(0, 0, 0);
    Vector3d ba = MINV(b,a);
    float nDotA = DOT(n, a);
    float nDotBA = DOT(n, ba);
    if (nDotBA == 0)
    {
      return zero;
    }
    else
    {
    float S = -nDotA/nDotBA;
    if (S >= 0 )
    {
      return PLSV(a,MULT(ba,S));
    }
    else
      return zero;
  }
}


void MobileController::onInit(InitEvent &evt)  
 { 
  avatarName = "man_000";
  HumanAvatar = getObj(avatarName.c_str());
  Checkpoint_coo = "checkpoint_000";
  Checkpoint = getObj(Checkpoint_coo.c_str());
  target = Vector3d(0, 0, 0);
  target_update = target;
  maxForce = 500.0; 
  m_radius = 10.0;
  m_distance = 10.0;
 	Wheel_one = 0.0;  
  Wheel_two = 0.0;  
  int argc =0;  
  char** argv = NULL;  
  wheel_sep = 0.73;
  Robot_speed  = 1;
  m_state = 0;
  my = this->getRobotObj(this->myname());  
  my->setWheel(10.0, 10.0);  
  ros::init(argc, argv, "sig_turtle_node");
  control_speed = 0; 
  control_turn = 0;                                                                                         
  ros::NodeHandle n;                            
                                               
// cmd_vel_sub =  n.subscribe<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1, &MobileController::OnCmdVel, this );
 cmd_vel_pub =  n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
 ros::Rate loop_rate(100);
 system ("source ~/catkin_ws_SIG_turtlebot/devel/setup.sh");  
 //system ("rosrun sig_turtle ros_controller &");  

 }  

double MobileController::onAction(ActionEvent &evt)  
{  
// my->setWheelVelocity(Wheel_one,Wheel_two); 
 target = intersectLine();
 Checkpoint->setPosition(target);

if ( fabs(getDistoObj(target,target_update) ) > 3 )
{
  m_state = 1;
 target_update = target; 
}
 switch(m_state)
    {
    case 1: {
      Robot_speed  = 3;
      if (goTo(target, 0) == true )
        {
          m_state = 2;
          //  printf("got it in case!1 \n");

        }
      break;
        }
    }
     cmd_vel_pub.publish(Vel_pub);
   ros::spinOnce();

 //  
 return 0.001;  
}  


/*
void MobileController::OnCmdVel( const geometry_msgs::TwistConstPtr& msg)
{
  //last_cmd_vel_time_ = this->my_world_->GetSimTime();
  double vr, va;
  vr = msg->linear.x*30;
  va = msg->angular.z;
  //printf("The wheel  .... linear : %f  .... angular: %f .....\n",msg->linear.x,msg->angular.z) ;
  Wheel_one = (vr - va) * (wheel_sep) / 2;
  Wheel_two = (vr + va) * (wheel_sep) / 2;
}*/
extern "C" Controller * createController() {
  return new MobileController;
}
/*
void SendController::spin()
{
  while(ros::ok()) ros::spinOnce();
}
*/