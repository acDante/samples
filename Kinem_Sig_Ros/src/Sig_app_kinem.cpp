#include <algorithm>
#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <Kinem_Sig_Ros/Velocity.h>
#include <Kinem_Sig_Ros/Robot_Joint.h>
#include <Kinem_Sig_Ros/Init_Pos.h>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <Eigen/Geometry>

#include "sigverse/commonlib/ControllerEvent.h"
#include "sigverse/commonlib/Controller.h"
#include "sigverse/commonlib/Logger.h"
#include "sigverse/commonlib/ViewImage.h"

#define error_angle_arm 0.03

class SendController : public Controller
{
	public:
		//function to use SigVerse
		void onInit(InitEvent &evt);
		double onAction(ActionEvent &evt);
		void onRecvMsg(RecvMsgEvent &evt);
		
		//function to use ROS
		void poseCallback(const Kinem_Sig_Ros::Robot_Joint joint);

		//function to control the robot
		bool moveLeftArm();
		
		//function for KDL
		void sendKDL_LeftArm();
		
		//other function
		double getRoll(Rotation rot);
		Vector3d changeBasis_WorldToRobot(Vector3d pos);
		
	private:
		RobotObj *my;
		
		//for ROS
		ros::Subscriber pose_sub ;
		ros::Publisher init_position;
		
		//define different joint of right and left arm
		double m_joint_left[7];
		bool move_arm_left;
		
		//final point for the arm
		double arm_x;
		double arm_y;
		double arm_z;
		double arm_roll;
		double arm_pitch;
		double arm_yaw;

		//define state for the different action
		int m_state;
};

/****************************function to use ROS*************************************/

void SendController::poseCallback(const Kinem_Sig_Ros::Robot_Joint joint)
{
	m_joint_left[0] = joint.L_JOINT0;
	m_joint_left[1] = joint.L_JOINT1;
	m_joint_left[2] = joint.L_JOINT3;
	m_joint_left[3] = joint.L_JOINT4;
	m_joint_left[4] = joint.L_JOINT5;
	m_joint_left[5] = joint.L_JOINT6;
	m_joint_left[6] = joint.L_JOINT7;
	
	move_arm_left = true;
}

/************************************************************************************/
/****************************function to use SigVerse********************************/
/************************************************************************************/

void SendController::onInit(InitEvent &evt)
{
	move_arm_left = false;

	m_state = 0;

	for (int i=0; i<7; i++)
		m_joint_left[i] = 0;
	
	my = this->getRobotObj(this->myname());
	my->setWheel(10.0, 10.0);
	
	int argc =0;
	char** argv = NULL;

	ros::init(argc, argv, "Sigvers_controller_node");
	ros::NodeHandle n;
	pose_sub = n.subscribe<Kinem_Sig_Ros::Robot_Joint>("mycommand_Joints", 1,&SendController::poseCallback,this);
	init_position  = n.advertise<Kinem_Sig_Ros::Init_Pos>("mycommand_init", 1);
	ros::Rate loop_rate(10);
}

/************************************************************************************/

double SendController::onAction(ActionEvent &evt)
{
	switch(m_state)
	{
		case 1: {   //move the left arm
					if (moveLeftArm() == true)
						m_state = 0;
					break;
				}
	}

	ros::spinOnce();
	return 0.00001;
}

/************************************************************************************/

void SendController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string msg = evt.getMsg();
	LOG_MSG(("message : %s", msg.c_str()));

	if (msg == "move_arm")
	{
		std::cout << "Enter Joint0 : "; std::cin >> m_joint_left[0];
		std::cout << "Enter Joint1 : "; std::cin >> m_joint_left[1];
		std::cout << "Enter Joint3 : "; std::cin >> m_joint_left[2];
		std::cout << "Enter Joint4 : "; std::cin >> m_joint_left[3];
		std::cout << "Enter Joint5 : "; std::cin >> m_joint_left[4];
		std::cout << "Enter Joint6 : "; std::cin >> m_joint_left[5];
		std::cout << "Enter Joint7 : "; std::cin >> m_joint_left[6];
		move_arm_left = true;
		m_state = 1;
	}
	else if (msg == "move_arm_kdl")
	{
		std::cout << "Enter x : "; std::cin >> arm_x;
		std::cout << "Enter y : "; std::cin >> arm_y;
		std::cout << "Enter z : "; std::cin >> arm_z;
		std::cout << "Enter roll : "; std::cin >> arm_roll;
		std::cout << "Enter pitch : "; std::cin >> arm_pitch;
		std::cout << "Enter yaw : "; std::cin >> arm_yaw;
		sendKDL_LeftArm();
		m_state = 1;
	}
	else if (msg == "get_hand")
	{
		Vector3d hand_w, hand_r;
		my->getJointPosition(hand_w,"LARM_JOINT7");
		hand_r = changeBasis_WorldToRobot(hand_w);
		printf("Position End effector in robot basis X:%f Y:%f Z:%f\n", hand_r.x(), hand_r.y(), hand_r.z());
		printf("Position End effector in world basis X:%f Y:%f Z:%f\n", hand_w.x(), hand_w.y(), hand_w.z());
	}
	else if (msg == "get")
	{
		 Vector3d j0,j1,j2,j3,j4,j5,j6,j7;
		 my->getJointPosition(j0, "LARM_JOINT0");
		 j0 = changeBasis_WorldToRobot(j0);
		 std::cout << " For joint 0  X : " << j0.x() << "\tY = " << j0.y() << "\tZ = " << j0.z() << std::endl;
		 my->getJointPosition(j1, "LARM_JOINT1");
		 j1 = changeBasis_WorldToRobot(j1);
		 std::cout << " For joint 1  X : " << j1.x() << "\tY = " << j1.y() << "\tZ = " << j1.z() << std::endl;
		 my->getJointPosition(j2, "LARM_JOINT2");
		 j2 = changeBasis_WorldToRobot(j2);
		 std::cout << " For joint 2  X : " << j2.x() << "\tY = " << j2.y() << "\tZ = " << j2.z() << std::endl;
		 my->getJointPosition(j3, "LARM_JOINT3");
		 j3 = changeBasis_WorldToRobot(j3);
		 std::cout << " For joint 3  X : " << j3.x() << "\tY = " << j3.y() << "\tZ = " << j3.z() << std::endl;
		 my->getJointPosition(j4, "LARM_JOINT4");
		 j4 = changeBasis_WorldToRobot(j4);
		 std::cout << " For joint 4  X : " << j4.x() << "\tY = " << j4.y() << "\tZ = " << j4.z() << std::endl;
		 my->getJointPosition(j5, "LARM_JOINT5");
		 j5 = changeBasis_WorldToRobot(j5);
		 std::cout << " For joint 5  X : " << j5.x() << "\tY = " << j5.y() << "\tZ = " << j5.z() << std::endl;
		 my->getJointPosition(j6, "LARM_JOINT6");
		 j6 = changeBasis_WorldToRobot(j6);
		 std::cout << " For joint 6  X : " << j6.x() << "\tY = " << j6.y() << "\tZ = " << j6.z() << std::endl;
		 my->getJointPosition(j7, "LARM_JOINT7");
		 j7 = changeBasis_WorldToRobot(j7);
		 std::cout << " For joint 7  X : " << j7.x() << "\tY = " << j7.y() << "\tZ = " << j7.z() << std::endl;
	}
}

/************************************************************************************/
/**********************************Move the arm**************************************/
/************************************************************************************/

bool SendController::moveLeftArm()
{
	bool j0 = false, j1 = false , j3 = false, j4 = false, j5 = false, j6 = false, j7 = false;

	if (move_arm_left == true)
	{
		double joint_left[7];
		joint_left[0] = my->getJointAngle("LARM_JOINT0");
		joint_left[1] = my->getJointAngle("LARM_JOINT1");
		joint_left[2] = my->getJointAngle("LARM_JOINT3");
		joint_left[3] = my->getJointAngle("LARM_JOINT4");
		joint_left[4] = my->getJointAngle("LARM_JOINT5");
		joint_left[5] = my->getJointAngle("LARM_JOINT6");
		joint_left[6] = my->getJointAngle("LARM_JOINT7");
	  
		if(joint_left[0] != m_joint_left[0] )
			if(joint_left[0] < m_joint_left[0] && m_joint_left[0]-joint_left[0] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT0", 0.15, 0.0);
			else if(joint_left[0] > m_joint_left[0] && joint_left[0]-m_joint_left[0] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT0", -0.15, 0.0);
			else
			{
				my->setJointVelocity("LARM_JOINT0", 0.0, 0.0);
				j0 = true;
			}
		else j0 = true;

		if(joint_left[1] != m_joint_left[1] )
			if(joint_left[1] < m_joint_left[1] && m_joint_left[1]-joint_left[1] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT1", 0.15, 0.0);
			else if(joint_left[1] > m_joint_left[1] && joint_left[1]-m_joint_left[1] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT1", -0.15, 0.0);
			else
			{
				my->setJointVelocity("LARM_JOINT1", 0.0, 0.0);
				j1 = true;
			}
		else j1 = true;

		if(joint_left[2] != m_joint_left[2] )
			if(joint_left[2] < m_joint_left[2] && m_joint_left[2]-joint_left[2] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT3", 0.15, 0.0);
			else if(joint_left[2] > m_joint_left[2] && joint_left[2]-m_joint_left[2] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT3", -0.15, 0.0);
			else
			{
				my->setJointVelocity("LARM_JOINT3", 0.0, 0.0);
				j3 = true;
			}
		else j3 = true;

		if(joint_left[3] != m_joint_left[3] )
			if(joint_left[3] < m_joint_left[3] && m_joint_left[3]-joint_left[3] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT4", 0.15, 0.0);
			else if(joint_left[3] > m_joint_left[3] && joint_left[3]-m_joint_left[3] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT4", -0.15, 0.0);
			else
			{
				my->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
				j4 = true;
			}
		else j4 = true;

		if(joint_left[4] != m_joint_left[4] )
			if(joint_left[4] < m_joint_left[4] && m_joint_left[4]-joint_left[4] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT5", 0.15, 0.0);
			else if(joint_left[4] > m_joint_left[4] && joint_left[4]-m_joint_left[4] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT5", -0.15, 0.0);
			else
			{
				my->setJointVelocity("LARM_JOINT5", 0.0, 0.0);
				j5 = true;
			}
		else j5 = true;

		if(joint_left[5] != m_joint_left[5] )
			if(joint_left[5] < m_joint_left[5] && m_joint_left[5]-joint_left[5] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT6", 0.15, 0.0);
			else if(joint_left[5] > m_joint_left[5] && joint_left[5]-m_joint_left[5] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT6", -0.15, 0.0);
			else
			{
				my->setJointVelocity("LARM_JOINT6", 0.0, 0.0);
				j6 = true;
			}
		else j6 = true;

		if(joint_left[6] != m_joint_left[6] )
			if(joint_left[5] < m_joint_left[6] && m_joint_left[6]-joint_left[5] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT7", 0.15, 0.0);
			else if(joint_left[5] > m_joint_left[6] && joint_left[5]-m_joint_left[6] > error_angle_arm)
				my->setJointVelocity("LARM_JOINT7", -0.15, 0.0);
			else
			{
				my->setJointVelocity("LARM_JOINT7", 0.0, 0.0);
				j7 = true;
			}
		else j7 = true;

		if (j0 == true && j1 == true && j3 == true && j4 == true && j5 == true && j6 == true && j7 == true)
		{
			move_arm_left = false;
			return true;
		}
		else
			return false;
	}
	return false;
}

/************************************************************************************/
/*******************************Function for KDL*************************************/
/************************************************************************************/

void SendController::sendKDL_LeftArm()
{
	Kinem_Sig_Ros::Init_Pos initial;
	
	initial.L_JOINT0 = my->getJointAngle("LARM_JOINT0");
	initial.L_JOINT1 = my->getJointAngle("LARM_JOINT1");
	initial.L_JOINT3 = my->getJointAngle("LARM_JOINT3");
	initial.L_JOINT4 = my->getJointAngle("LARM_JOINT4");
	initial.L_JOINT5 = my->getJointAngle("LARM_JOINT5");
	initial.L_JOINT6 = my->getJointAngle("LARM_JOINT6");
	initial.L_JOINT7 = my->getJointAngle("LARM_JOINT7");
	initial.x = arm_x;
	initial.y = arm_y;
	initial.z = arm_z;
	initial.Roll = arm_roll;
	initial.Pitch = arm_pitch;
	initial.Yaw = arm_yaw;
	
	init_position.publish(initial);
}

/************************************************************************************/
/**********************************Other function************************************/
/************************************************************************************/

double SendController::getRoll(Rotation rot)
{
	// get angles arround y-axis
	double qw = rot.qw();  
	double qx = rot.qx(); 
	double qy = rot.qy(); 
	double qz = rot.qz();
	
	return atan2(2*qy*qw - 2*qx*qz, 1 - 2*qy*qy - 2*qz*qz);;
}

Vector3d SendController::changeBasis_WorldToRobot(Vector3d pos)
{
	Vector3d pos_my, ret;
	Rotation rot_my;
	
	my->getPosition(pos_my); 
	my->getRotation(rot_my);

	/* Calculation of RPY for the robot */
	double rollMy = getRoll(rot_my);

	double x_o = pos.x()-pos_my.x();
	double y_o = -(pos.z()-pos_my.z());

	if (rollMy >= 0)
		ret = Vector3d(x_o*cos(rollMy) - y_o*sin(rollMy), x_o*sin(rollMy) + y_o*cos(rollMy), pos.y());
	else
		ret = Vector3d(x_o*cos(rollMy) + y_o*sin(rollMy), y_o*cos(rollMy) - x_o*sin(rollMy), pos.y());
		
	return ret;
}


extern "C" Controller * createController() {
  return new SendController;
}
