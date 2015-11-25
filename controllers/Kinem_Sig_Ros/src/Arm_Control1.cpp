#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include <Kinem_Sig_Ros/Robot_Joint.h>
#include <Kinem_Sig_Ros/Init_Pos.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>

using namespace KDL;
using namespace std;

class CommandSig
{
	public:
		CommandSig();
		void CommandLoop();
		void poseCallback(const Kinem_Sig_Ros::Init_Pos initial);
		
	private:
		ros::NodeHandle n;
		ros::Publisher vel_pub_;
		ros::Subscriber init_position_sub;
		KDL::Chain L_Arm ;

		float m_x;
		float m_y;
		float m_z;
		float m_pitch;
		float m_roll;
		float m_yaw;
 };
 
 /************************************************************************************/
 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Sigvers_Command_node");
	CommandSig Command_sig;

	ros::Rate loop_rate(10);
	ros::spin();
	return(0);
}


/************************************************************************************/

CommandSig::CommandSig()
{
	vel_pub_ = n.advertise<Kinem_Sig_Ros::Robot_Joint>("mycommand_Joints", 1);
	init_position_sub  = n.subscribe<Kinem_Sig_Ros::Init_Pos>("mycommand_init", 1,&CommandSig::poseCallback,this);
	
	//here,indicate the vector between each joint
	L_Arm.addSegment(Segment(Joint("Joint0",Joint::RotZ),Frame(Vector(9.6, -6.0, 83.6))));
	L_Arm.addSegment(Segment(Joint("Joint1",Joint::RotX),Frame(Vector(7.04, 0.0, -0.4))));
	L_Arm.addSegment(Segment(Joint("Joint3",Joint::RotZ),Frame(Vector(0.32, -0.08, -17.6))));
	L_Arm.addSegment(Segment(Joint("Joint4",Joint::RotX),Frame(Vector(0.0, -1.28, -1.92))));
	L_Arm.addSegment(Segment(Joint("Joint5",Joint::RotZ),Frame(Vector(-0.32, 2.0, -11.2))));
	L_Arm.addSegment(Segment(Joint("Joint6",Joint::RotX)));
	//L_Arm.addSegment(Segment(Joint("Joint7",Joint::RotY)));
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

void CommandSig::poseCallback(const Kinem_Sig_Ros::Init_Pos initial)
{
	Kinem_Sig_Ros::Robot_Joint Joint;
	unsigned int nj = L_Arm.getNrOfJoints();
	
	//we init rotation and position matrix of the hand
	Frame cartPos;
	Frame F_dest;
	
	//value for each joint
	JntArray jointpositions = JntArray(nj);
	JntArray jointArrayMin = JntArray(nj);
	JntArray jointArrayMax = JntArray(nj);
	JntArray theta(nj);
	
	//initialisation of each joint, send by sigserver
	jointpositions(0) = 0.00000001 + initial.L_JOINT0;
	jointpositions(1) = 0.00000001 + initial.L_JOINT1;
	jointpositions(2) = 0.00000001 + initial.L_JOINT3;
	jointpositions(3) = 0.00000001 + initial.L_JOINT4;
	jointpositions(4) = 0.00000001 + initial.L_JOINT5;
	jointpositions(5) = 0.00000001 + initial.L_JOINT6;
	//jointpositions(6) = 0.00000001 + initial.L_JOINT7;
	
	jointArrayMin(0) =  -3.5*M_PI/8;
	jointArrayMin(1) =  -M_PI;
	jointArrayMin(2) =  -M_PI;
	jointArrayMin(3) =  -2*M_PI/3;
	jointArrayMin(4) =  -M_PI;
	jointArrayMin(5) =  -M_PI/4;
	//jointArrayMin(6) =  -M_PI/4;
	
	jointArrayMax(0) =  3.5*M_PI/8;
	jointArrayMax(1) =  M_PI;
	jointArrayMax(2) =  M_PI;
	jointArrayMax(3) =  2*M_PI/3;
	jointArrayMax(4) =  M_PI;
	jointArrayMax(5) =  M_PI/4;
	//jointArrayMax(6) =  M_PI/4;
	
//end-effector
	m_x = initial.x;
	m_y = initial.y;
	m_z = initial.z;
	m_pitch = initial.Pitch;
	m_roll = initial.Roll;
	m_yaw = initial.Yaw;
	
	F_dest.p = Vector(m_x, m_y, m_z); 
	F_dest.M = Rotation::RPY(m_roll+M_PI_2, m_pitch, m_yaw);
	
	cout << " My vector is :  X : " << m_x <<  "   Y : "<<  m_y  <<  "   Z  : " <<  m_z  << endl;
	cout << " My angle is :  Roll : " << m_roll <<  "   Pitch : "<<  m_pitch  <<  "   Yaw  : " <<  m_yaw  << endl;
	
//	FK
	ChainFkSolverPos_recursive fkSolver = ChainFkSolverPos_recursive(L_Arm);

// Calculate forward position kinematics
	if(fkSolver.JntToCart(jointpositions, cartPos) >= 0)
		cout << cartPos << endl << "IK theta calculation succeeded!" << endl;
	else
		printf("%s \n", "[Error] FK could not be calculated!\n");
	
	ChainFkSolverPos_recursive fkSolver1 = ChainFkSolverPos_recursive(L_Arm);
	ChainIkSolverVel_pinv ikSolverVel1 = ChainIkSolverVel_pinv(L_Arm);
//ChainIkSolverPos_NR_JL(chain, minimum joint, maximum joint, ..., ..., number of iteration, accurate)
	ChainIkSolverPos_NR_JL ikSolverPos = ChainIkSolverPos_NR_JL(L_Arm, jointArrayMin, jointArrayMax, fkSolver1, ikSolverVel1, 1000, 4);
	
//run the solver
	if (int ret = ikSolverPos.CartToJnt(jointpositions, F_dest, theta) < 0 )
		cout << "error calculation : " << ret << endl;
	else
	{ 
		if(fkSolver.JntToCart(theta, cartPos) >= 0) 
			cout << cartPos << endl << "IK theta calculation succeeded!" << endl;
		else
			printf("%s \n", "[Error] FK theta  could not be calculated!\n");			
		
		Joint.L_JOINT0 = theta(0);
		Joint.L_JOINT1 = theta(1);
		Joint.L_JOINT3 = theta(2);
		Joint.L_JOINT4 = theta(3);
		Joint.L_JOINT5 = theta(4);
		Joint.L_JOINT6 = theta(5);
		//Joint.L_JOINT7 = theta(6);

		vel_pub_.publish(Joint);
	}
}

/************************************************************************************/


