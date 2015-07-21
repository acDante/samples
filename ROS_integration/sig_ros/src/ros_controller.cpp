 #include <ros/ros.h>
 #include <signal.h>
 #include <termios.h>
 #include <stdio.h>
 #include "std_msgs/String.h"
 #include <sig_ros/Velocity.h>
 #define KEYCODE_R 0x43
 #define KEYCODE_L 0x44
 #define KEYCODE_U 0x41
 #define KEYCODE_D 0x42
 #define KEYCODE_Q 0x71

 class CommandSig
 {
 public:
   CommandSig();
   void CommandLoop();
  void chatterCallback (const std_msgs::String::ConstPtr& msg);
 private:


   ros::NodeHandle nh_;
   double linear_, angular_, l_scale_, a_scale_;
   ros::Publisher vel_pub_;
   ros::Subscriber  mess_sub ;

 };

 CommandSig::CommandSig():
   linear_(0),
   angular_(0),
   l_scale_(2.0),
   a_scale_(2.0)
 {
   nh_.param("scale_angular", a_scale_, a_scale_);
   nh_.param("scale_linear", l_scale_, l_scale_);
   vel_pub_ = nh_.advertise<sig_ros::Velocity>("mycommand_velocity", 1);
   mess_sub = nh_.subscribe("message_command",100,&CommandSig::chatterCallback,this);
 }

 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "ros_command_node");
   CommandSig Command_sig;
  ros::Rate loop_rate(100);
  ros::spin();
   return(0);
 }


void CommandSig::chatterCallback (const std_msgs::String::ConstPtr& msg)
{
 sig_ros::Velocity vel;
if(strcmp("moveforward",msg->data.c_str())==0)
{
vel.angular = 3.0;
vel.linear = 3.0;
vel_pub_.publish(vel);
}
    else if(strcmp("movebackward",msg->data.c_str())==0)
    {
vel.angular = -3.0;
vel.linear = -3.0;
vel_pub_.publish(vel);

    }

      else if(strcmp("turnleft",msg->data.c_str())==0)
            {
vel.angular = 0.78;
vel.linear = -0.78;
vel_pub_.publish(vel);
sleep(1.8);
vel.angular = 0.0;
vel.linear =  0.0;
vel_pub_.publish(vel);
            }
                else if(strcmp("turnright",msg->data.c_str())==0)
                {
vel.angular = -0.78;
vel.linear = 0.78;
vel_pub_.publish(vel);
sleep(1.8);
vel.angular = 0.0;
vel.linear =  0.0;
vel_pub_.publish(vel);
                }
                     else if(strcmp("stop",msg->data.c_str())==0)
                     {
vel.angular = 0.0;
vel.linear = 0.0;
vel_pub_.publish(vel);

                     }

 else if(strcmp("square",msg->data.c_str())==0)
                     {
vel.angular = 3.0;
vel.linear = 3.0;

vel_pub_.publish(vel);
sleep(1.8);
vel.angular = -0.78;
vel.linear = 0.78;
vel_pub_.publish(vel);
sleep(1.8);
vel.angular = 0.0;
vel.linear =  0.0;
vel_pub_.publish(vel);
vel.angular = 3.0;
vel.linear = 3.0;

vel_pub_.publish(vel);
sleep(1.8);
vel.angular = -0.78;
vel.linear = 0.78;

vel_pub_.publish(vel);
sleep(1.8);
vel.angular = 0.0;
vel.linear =  0.0;

vel_pub_.publish(vel);
vel.angular = 3.0;
vel.linear = 3.0;

vel_pub_.publish(vel);
sleep(1.8);
vel.angular = -0.78;
vel.linear = 0.78;
vel_pub_.publish(vel);
sleep(1.8);
vel.angular = 0.0;
vel.linear =  0.0;
vel_pub_.publish(vel);

vel.angular = 3.0;
vel.linear = 3.0;
vel_pub_.publish(vel);
sleep(1.8);
vel.angular = -0.78;
vel.linear = 0.78;
vel_pub_.publish(vel);
sleep(1.8);
vel.angular = 0.0;
vel.linear =  0.0;
vel_pub_.publish(vel);
vel.angular = 3.0;
vel.linear = 3.0;
vel_pub_.publish(vel);
sleep(1.8);

vel.angular = 0.0;
vel.linear =  0.0;
vel_pub_.publish(vel);
                     }

else if(strcmp("cercle",msg->data.c_str())==0)
                     {
vel.angular = 2.5;
vel.linear =  2.2;
vel_pub_.publish(vel);

                     }
}

 void CommandSig::CommandLoop()
 {

   ros::Rate loop_rate(10);
while (ros::ok)
{
  ros::spinOnce();
  loop_rate.sleep();
}
   return;
 }
