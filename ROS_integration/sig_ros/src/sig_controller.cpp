#include "Controller.h"
#include "Logger.h"
#include "ControllerEvent.h"
#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include <sig_ros/Velocity.h>


#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define ARY_SIZE(ARY) ( (int)(sizeof(ARY)/sizeof(ARY[0])) )



class SendController : public Controller
{
public:
  //シミュレーション開始時に一度だけ呼出される関数onInitの利用を宣言します
  void onInit(InitEvent &evt);
  double onAction(ActionEvent &evt);
  void onRecvMsg(RecvMsgEvent &evt);
  void poseCallback(const sig_ros::VelocityConstPtr& vel);

public:
RobotObj *my;

  double velocity;
  int i;
  ros::Publisher chatter_pub;
  ros::Subscriber  pose_sub ;
 int j;

 double Weel_one;
 double Weel_two;
};


void SendController::poseCallback(const sig_ros::VelocityConstPtr& vel)
{
    Weel_one =  vel->linear;
    Weel_two =  vel->angular;

}

void SendController::onInit(InitEvent &evt)
{

 Weel_one = 0.0;
 Weel_two = 0.0;

int argc =0;
char** argv = NULL;
my = this->getRobotObj(this->myname());
my->setWheel(10.0, 10.0);
ros::init(argc, argv, "sig_controller_node");
ros::NodeHandle n;
velocity = 1;

chatter_pub = n.advertise<std_msgs::String>("message_command", 1);

pose_sub = n.subscribe<sig_ros::Velocity>("mycommand_velocity", 1,&SendController::poseCallback,this);
ros::Rate loop_rate(10);

// Running the ros_command node
 system ("source ~/catkin_ws/devel/setup.sh");
 system ("rosrun sig_ros ros_controller &");

}

double SendController::onAction(ActionEvent &evt)
{
my->setWheelVelocity(Weel_one,Weel_two);
 ros::spinOnce();
  //1秒おきにonActionが呼び出されます
  return 0.01;
}



void SendController::onRecvMsg(RecvMsgEvent &evt)
{
 std::string sender = evt.getSender();

  // 文字列を取得
  std::string msg = evt.getMsg();
  LOG_MSG(("message : %s", msg.c_str()));
  std_msgs::String mesg;
  std::stringstream ss;
  //メッセージが"Hello!!"なら手を上げます
  ss << msg;
  mesg.data = ss.str();
  chatter_pub.publish(mesg);
}


extern "C"  Controller * createController ()
{
  return new SendController;
}
