#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include "ViewImage.h"
#include <math.h>
#include <fstream>
#include <iostream>
#include <string>

#define error_angle 0.05
#define error_distance 0.4
#define error_angle_arm 0.05
#define time_avoid 20

double tab_joint_left[5][7] = {	{0,0,0,0,0,0,0},
								{0.5,-0.5,-0.2,-1.5,0,0,0},
								{-0.2,-1,-1.27,-0.7,-0.1,0,0},
								{0,-1.1,-1.57,-1.57,-0.3,0,0},
								{0,-1,-1,-1,0,0,0}	};

double tab_joint_right[5][7]= { {0,0,0,0,0,0,0},
								{-0.5,-0.5,0.2,-1.5,0,0,0},
								{0.2,-1,1.27,-0.7,0.1,0,0},
								{0,-1.1,1.57,-1.57,0.3,0,0},
								{0,-1,1,-1,0,0,0}	};

class ControlRobot : public Controller
{
	public:
		//function to use SIGVerse
		void onInit(InitEvent &evt);
		double onAction(ActionEvent &evt);
		void onRecvMsg(RecvMsgEvent &evt);

		//function to move the robot
		void stopRobotMove(void);
		bool goTo(Vector3d pos, double rangeToPoint = 0, double Robot_speed = 3);
		bool goTo_AvoidObstacle(Vector3d pos, double rangeToPoint = 0, double Robot_speed = 3);
		
		//function to move the left arm
		bool moveLeftArm();
		void grasp_left_hand(std::string &object);
		void release_left_hand();
		void chooze_task_arm_left(int task);
		
		//function to move the right arm
		bool moveRightArm();
		void grasp_right_hand(std::string &object);
		void release_right_hand();
		void chooze_task_arm_right(int task);
		
		//Find the point near the object and the table
		Vector3d PointApproachObj(std::string &table, std::string &object, double rangeToTable);
		int defineCoteTable(std::string &table, std::string &object);
		Vector3d ObjectPosition(std::string &name);
		
		//function use by other function
		double getAngularXonVect(Vector3d pos, Vector3d mypos);
		double getDist2D(Vector3d pos, Vector3d pos2);
		double getDist3D(Vector3d pos, Vector3d pos2);
		double getRoll(Rotation rot);
		double getPitch(Rotation rot);
		double getYaw(Rotation rot);
		unsigned char distance_sensor();
		Vector3d changeBasis_WorldToRobot(Vector3d pos);
		
		//how control the robot for an application (here, example te grasp an object)
		void planning_control();
		
	private:
		RobotObj *my;
		ViewService *m_view;
		
		Vector3d go_to;
		std::string object,table;
		double RangeToPoint;
		
		//define different joint of right and left arm
		double m_joint_left[7];
		double m_joint_right[7];

		//define state for the different action
		int control_state;
		int planning_state;

		//define the different collision detect by the robot
		bool m_obstacle;
		bool m_grasp_right;
		bool m_grasp_left;
		
		FILE * fichier;
};

/************************************************************************************/
/*************************** Function to use SIGVerse *******************************/
/************************************************************************************/

void ControlRobot::onInit(InitEvent &evt)
{
	m_obstacle = false;
	m_grasp_right = false;
	m_grasp_left = false;

	control_state  = 0;
	planning_state = 0;
	
	for (int i=0; i<7; i++)
	{
		m_joint_left[i]  = 0;
		m_joint_right[i] = 0;	
	}
	
	my = getRobotObj(this->myname());
	my->setWheel(10.0, 10.0);
	RangeToPoint = 0;
	
	m_view = (ViewService*)connectToService("SIGViewer");
	
    fichier = fopen("left_arm.xls", "wb");
}

double ControlRobot::onAction(ActionEvent &evt)
{
	switch(control_state)
	{
		case 0: {
					stopRobotMove();
					planning_control();
					release_left_hand();
					release_right_hand();
					RangeToPoint = 0;
					break;
				}
				
		case 1: {   //place the robot at a position in the world and avoid obstacle
					if (goTo_AvoidObstacle(go_to, RangeToPoint) == true) planning_control();
					break;
				}
				
		case 2: {	//place the robot at a position in the world 
					if (goTo(go_to, RangeToPoint) == true) planning_control();
					break;
				}
		case 3: {   //move left arm
					if (moveLeftArm() == true) planning_control();
					break;
				}
				
		case 4: {   //move right arm
					if (moveRightArm() == true) planning_control();
					break;
				}
		case 5: {   //move left arm and grasp
					if (moveLeftArm() == true) planning_control();
					grasp_left_hand(object);
					break;
				}
		case 6: {   //move right arm and grasp
					if (moveRightArm() == true) planning_control();
					grasp_right_hand(object);
					break;
				}
		
	}
	
	// The next time [sec] when onAction will be called
	return 0.001;
}

void ControlRobot::onRecvMsg(RecvMsgEvent &evt)
{
	std::string msg = evt.getMsg();
	double x,z;

	if ( msg == "move")
	{
		std::cout << "Enter x : "; std::cin >> x;
		std::cout << "Enter z : "; std::cin >> z;
		std::cout << "Avoidance : With (1) Without (2) : "; std::cin >> control_state;
		go_to = Vector3d(x,30,z);
	}
	else if (msg == "move_left_arm")
	{
		for (int i=0; i<7; i++)
		{
			std::cout << "Enter Joint" << i << " : "; 
			std::cin >> m_joint_left[i];
		}
		control_state = 3;
	}
	else if (msg == "move_right_arm")
	{
		for (int j=0; j<7; j++)
		{
			std::cout << "Enter Joint" << j << " : "; 
			std::cin >> m_joint_right[j];
		}
		control_state = 4;
	}
	else if (msg == "grasp_planning")
	{
		planning_control();
		planning_state = 1;
	}
	else if (msg == "get_LeftHand")
	{
		Vector3d pos,pos2;
		my->getJointPosition(pos,"LARM_JOINT7");
		pos2 = changeBasis_WorldToRobot(pos);
		std::cout << "Coordinate left hand\nx : " << pos2.x() << "\ty : " << pos2.y() << "\tz : " << pos2.z() << std::endl;
	}
	else if (msg == "get_RightHand")
	{
		Vector3d pos,pos2;
		my->getJointPosition(pos,"RARM_JOINT7");
		pos2 = changeBasis_WorldToRobot(pos);
		std::cout << "Coordinate right hand\nx : " << pos2.x() << "\ty : " << pos2.y() << "\tz : " << pos2.z() << std::endl;
	}
	else if (msg == "grasp_left")
	{
		std::cout << "Object : "; std::cin >> object;
		grasp_left_hand(object);
	}
	else if (msg == "grasp_right")
	{
		std::cout << "Object : "; std::cin >> object;
		grasp_right_hand(object);
	}
	else if (msg == "release_left")
		release_left_hand();
	else if (msg == "release_right")
		release_right_hand();
	else control_state = 0;
}

void ControlRobot::planning_control()
{
	switch (planning_state)
	{
		case 1 : std::cout << "Enter Table name: "; std::cin >> table;
				 std::cout << "Enter Object name: "; std::cin >> object;
				 go_to = PointApproachObj(table, object, 50);
				 control_state = 2;
				 planning_state++;
				 break;
		case 2 : chooze_task_arm_left(1);
				 control_state = 3;
				 planning_state++;
				 break;
		case 3 : go_to = ObjectPosition(object);
				 RangeToPoint = 37;
				 control_state = 2;
				 planning_state++;
				 break;
		case 4 : chooze_task_arm_left(2);
				 control_state = 5;
				 planning_state++;
				 break;
		case 5 : chooze_task_arm_left(3);
				 control_state = 5;
				 planning_state++;
				 break;
		case 6 : go_to = Vector3d(0, 0, 0);
				 RangeToPoint = 0;
				 control_state = 2;
				 planning_state++;
				 break;
		case 7 : chooze_task_arm_left(4);
				 control_state = 3;
				 planning_state = 0;
				 break;
		default: control_state = 0;
				 break;
	}
}

/************************************************************************************/
/***************************Move the robot in the world******************************/
/************************************************************************************/

bool ControlRobot::goTo(Vector3d pos, double rangeToPoint, double Robot_speed)
{
  double speed_droit,speed_tourne;
  static double time = 0;
  static double angle;

  Vector3d ownPosition;
  my->getPosition(ownPosition);

  Rotation ownRotation;
  my->getRotation(ownRotation);

  double dist = getDist2D(pos,ownPosition);
  if (dist > 1) angle = getAngularXonVect(pos, ownPosition);
  double roll = getRoll(ownRotation);

  if (angle > 3 || angle < -3) angle = M_PI;

  // error on angle
  if ((angle-roll)>-error_angle && (angle-roll)<error_angle)
  // error on distance
    if (dist-rangeToPoint < error_distance && dist-rangeToPoint > -error_distance)
    {
      stopRobotMove();
      time = 0;
      fprintf(fichier, "destination : \t%f\t%f\n",pos.x(),pos.z());
      return true;
    }
    else
    {
      speed_tourne = 0;
      if (dist-rangeToPoint < 3)
      {
		speed_droit = dist-rangeToPoint;
        if( dist-rangeToPoint > 0 )
          my->setWheelVelocity(speed_droit, speed_droit);
        else
          my->setWheelVelocity(-speed_droit, -speed_droit);
	  }
      else 
      {
		speed_droit = Robot_speed;
		if( dist-rangeToPoint > 0 )
			my->setWheelVelocity(speed_droit,speed_droit);
        else
			my->setWheelVelocity(-speed_droit,-speed_droit);
	  }
    }
  else
  {
	speed_droit = 0;
    speed_tourne = fabs(angle-roll);
    if (speed_tourne > 0.4)
    {
	  speed_tourne = 0.4;
      if (angle < -M_PI_2 && roll > M_PI_2)
        my->setWheelVelocity(-speed_tourne, speed_tourne);
      else if (angle > M_PI_2 && roll < -M_PI_2)
          my->setWheelVelocity(speed_tourne, -speed_tourne);
      else if (angle < roll)
          my->setWheelVelocity(speed_tourne, -speed_tourne);
      else
        my->setWheelVelocity(-speed_tourne, speed_tourne);
    }
    else if (angle < -M_PI_2 && roll > M_PI_2)
        my->setWheelVelocity(-speed_tourne, speed_tourne);
       else if (angle > M_PI_2 && roll < -M_PI_2)
          my->setWheelVelocity(speed_tourne, -speed_tourne);
       else if (angle < roll)
          my->setWheelVelocity(speed_tourne, -speed_tourne);
       else
        my->setWheelVelocity(-speed_tourne, speed_tourne);
  }
  
  fprintf(fichier, "%f\t%f\t%f\t%f\t%f\t%f\t%f\n",time,angle*180/M_PI,roll*180/M_PI,0.0,dist-rangeToPoint,speed_droit,speed_tourne);
  time += 0.001;
  
  return false;
}

bool ControlRobot::goTo_AvoidObstacle(Vector3d pos, double rangeToPoint, double Robot_speed)
{
	unsigned char distance;
	static unsigned int time_avoid_obstacle = 0;
	bool OK = false;
	
	distance = distance_sensor();
	
	if (distance < 250 || m_obstacle == true)
	{
		m_obstacle = true;
			if (distance >= 255)
			{
				my->setWheelVelocity(2, 2);
				if (time_avoid_obstacle == time_avoid)
				{
					time_avoid_obstacle = 0;
					m_obstacle = false;
				}
				time_avoid_obstacle++;
			}
			else
				my->setWheelVelocity(0.2, -0.2);
	}
	else
		OK = goTo(pos, rangeToPoint, Robot_speed);
	
	return OK;
}

void ControlRobot::stopRobotMove()
{
	my->setWheelVelocity(0.0, 0.0);
	my->setJointVelocity("LARM_JOINT0", 0.0, 0.0);
	my->setJointVelocity("LARM_JOINT1", 0.0, 0.0);
	my->setJointVelocity("LARM_JOINT3", 0.0, 0.0);
	my->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
	my->setJointVelocity("LARM_JOINT5", 0.0, 0.0);
	my->setJointVelocity("LARM_JOINT6", 0.0, 0.0);
	my->setJointVelocity("LARM_JOINT7", 0.0, 0.0);
	my->setJointVelocity("RARM_JOINT0", 0.0, 0.0);
	my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
	my->setJointVelocity("RARM_JOINT3", 0.0, 0.0);
	my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
	my->setJointVelocity("RARM_JOINT5", 0.0, 0.0);
	my->setJointVelocity("RARM_JOINT6", 0.0, 0.0);
	my->setJointVelocity("RARM_JOINT7", 0.0, 0.0);
  
}

/************************************************************************************/
/****************************Function for the left arm*******************************/
/************************************************************************************/

bool ControlRobot::moveLeftArm()
{
	static double time;
	bool j0 = false, j1 = false , j3 = false, j4 = false, j5 = false, j6 = false, j7 = false;

	double joint_left[7];
	joint_left[0] = my->getJointAngle("LARM_JOINT0");
	joint_left[1] = my->getJointAngle("LARM_JOINT1");
	joint_left[2] = my->getJointAngle("LARM_JOINT3");
	joint_left[3] = my->getJointAngle("LARM_JOINT4");
	joint_left[4] = my->getJointAngle("LARM_JOINT5");
	joint_left[5] = my->getJointAngle("LARM_JOINT6");
	joint_left[6] = my->getJointAngle("LARM_JOINT7");
	time += 0.001;
	
	fprintf(fichier, "%f\t%f\t%f\t",time,m_joint_left[0], joint_left[0]);
	fprintf(fichier, "%f\t%f\t",m_joint_left[1], joint_left[1]);
	fprintf(fichier, "%f\t%f\t",m_joint_left[2], joint_left[2]);
	fprintf(fichier, "%f\t%f\t",m_joint_left[3], joint_left[3]);
	fprintf(fichier, "%f\t%f\t",m_joint_left[4], joint_left[4]);
	fprintf(fichier, "%f\t%f\t",m_joint_left[5], joint_left[5]);
	fprintf(fichier, "%f\t%f\n",m_joint_left[6], joint_left[6]);
  
	if(joint_left[0] != m_joint_left[0] )
		if(joint_left[0] < m_joint_left[0] && m_joint_left[0]-joint_left[0] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT0", 0.2, 0.0);
		else if(joint_left[0] > m_joint_left[0] && joint_left[0]-m_joint_left[0] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT0", -0.2, 0.0);
		else
		{
			my->setJointVelocity("LARM_JOINT0", 0.0, 0.0);
			j0 = true;
		}
	else j0 = true;

	if(joint_left[1] != m_joint_left[1] )
		if(joint_left[1] < m_joint_left[1] && m_joint_left[1]-joint_left[1] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT1", 0.2, 0.0);
		else if(joint_left[1] > m_joint_left[1] && joint_left[1]-m_joint_left[1] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT1", -0.2, 0.0);
		else
		{
			my->setJointVelocity("LARM_JOINT1", 0.0, 0.0);
			j1 = true;
		}
	else j1 = true;

	if(joint_left[2] != m_joint_left[2] )
		if(joint_left[2] < m_joint_left[2] && m_joint_left[2]-joint_left[2] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT3", 0.2, 0.0);
		else if(joint_left[2] > m_joint_left[2] && joint_left[2]-m_joint_left[2] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT3", -0.2, 0.0);
		else
		{
			my->setJointVelocity("LARM_JOINT3", 0.0, 0.0);
			j3 = true;
		}
	else j3 = true;

	if(joint_left[3] != m_joint_left[3] )
		if(joint_left[3] < m_joint_left[3] && m_joint_left[3]-joint_left[3] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT4", 0.2, 0.0);
		else if(joint_left[3] > m_joint_left[3] && joint_left[3]-m_joint_left[3] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT4", -0.2, 0.0);
		else
		{
			my->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
			j4 = true;
		}
	else j4 = true;

	if(joint_left[4] != m_joint_left[4] )
		if(joint_left[4] < m_joint_left[4] && m_joint_left[4]-joint_left[4] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT5", 0.2, 0.0);
		else if(joint_left[4] > m_joint_left[4] && joint_left[4]-m_joint_left[4] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT5", -0.2, 0.0);
		else
		{
			my->setJointVelocity("LARM_JOINT5", 0.0, 0.0);
			j5 = true;
		}
	else j5 = true;

	if(joint_left[5] != m_joint_left[5] )
		if(joint_left[5] < m_joint_left[5] && m_joint_left[5]-joint_left[5] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT6", 0.2, 0.0);
		else if(joint_left[5] > m_joint_left[5] && joint_left[5]-m_joint_left[5] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT6", -0.2, 0.0);
		else
		{
			my->setJointVelocity("LARM_JOINT6", 0.0, 0.0);
			j6 = true;
		}
	else j6 = true;

	if(joint_left[6] != m_joint_left[6] )
		if(joint_left[6] < m_joint_left[6] && m_joint_left[6]-joint_left[6] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT7", 0.2, 0.0);
		else if(joint_left[6] > m_joint_left[6] && joint_left[6]-m_joint_left[6] > error_angle_arm)
			my->setJointVelocity("LARM_JOINT7", -0.2, 0.0);
		else
		{
			my->setJointVelocity("LARM_JOINT7", 0.0, 0.0);
			j7 = true;
		}
	else j7 = true;

	if (j0 == true && j1 == true && j3 == true && j4 == true && j5 == true && j6 == true && j7 == true)
	{
		time = 0;
		return true;
	}
	else
		return false;

  return false;
}

void ControlRobot::grasp_left_hand(std::string &object)
{
	Vector3d hand, obje;
	SimObj *obj = getObj(object.c_str());

	obj->getPosition(obje);
	my->getJointPosition(hand, "LARM_JOINT7");

	double distance = getDist3D(hand,obje);

	if(distance < 11 &&  m_grasp_left == false)
	{
		CParts * parts = my->getParts("LARM_LINK7");
		if (parts->graspObj(object))
			m_grasp_left = true;
	}
}

void ControlRobot::release_left_hand()
{
	CParts * parts = my->getParts("LARM_LINK7");
	if( m_grasp_left == true)
   	{
		broadcastMsg("Object_released");
		parts->releaseObj();
		m_grasp_left = false;
	}
}

void ControlRobot::chooze_task_arm_left(int task)
{
  for (int i=0; i<7; i++)
    m_joint_left[i] = tab_joint_left[task][i];
}

/************************************************************************************/
/***************************Function for the right arm*******************************/
/************************************************************************************/

bool ControlRobot::moveRightArm()
{	
	bool j0 = false, j1 = false , j3 = false, j4 = false, j5 = false, j6 = false, j7 = false;
	
	double joint_right[7];
	joint_right[0] = my->getJointAngle("RARM_JOINT0");
	joint_right[1] = my->getJointAngle("RARM_JOINT1");
	joint_right[2] = my->getJointAngle("RARM_JOINT3");
	joint_right[3] = my->getJointAngle("RARM_JOINT4");
	joint_right[4] = my->getJointAngle("RARM_JOINT5");
	joint_right[5] = my->getJointAngle("RARM_JOINT6");
	joint_right[6] = my->getJointAngle("RARM_JOINT7");
  
	if(joint_right[0] != m_joint_right[0] )
		if(joint_right[0] < m_joint_right[0] && m_joint_right[0]-joint_right[0] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT0", 0.2, 0.0);
		else if(joint_right[0] > m_joint_right[0] && joint_right[0]-m_joint_right[0] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT0", -0.2, 0.0);
		else 
		{
			my->setJointVelocity("RARM_JOINT0", 0.0, 0.0);
			j0 = true;
		}
	else j0 = true;

	if(joint_right[1] != m_joint_right[1] )
		if(joint_right[1] < m_joint_right[1] && m_joint_right[1]-joint_right[1] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT1", 0.2, 0.0);
		else if(joint_right[1] > m_joint_right[1] && joint_right[1]-m_joint_right[1] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT1", -0.2, 0.0);
		else 
		{
			my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			j1 = true;
		}
	else j1 = true;

	if(joint_right[2] != m_joint_right[2] )
		if(joint_right[2] < m_joint_right[2] && m_joint_right[2]-joint_right[2] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT3", 0.2, 0.0);
		else if(joint_right[2] > m_joint_right[2] && joint_right[2]-m_joint_right[2] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT3", -0.2, 0.0);
		else 
		{
			my->setJointVelocity("RARM_JOINT3", 0.0, 0.0);
			j3 = true;
		}
	else j3 = true;

	if(joint_right[3] != m_joint_right[3] )
		if(joint_right[3] < m_joint_right[3] && m_joint_right[3]-joint_right[3] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT4", 0.2, 0.0);
		else if(joint_right[3] > m_joint_right[3] && joint_right[3]-m_joint_right[3] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT4", -0.2, 0.0);
		else 
		{
			my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			j4 = true;
		}
	else j4 = true;

	if(joint_right[4] != m_joint_right[4] )
		if(joint_right[4] < m_joint_right[4] && m_joint_right[4]-joint_right[4] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT5", 0.2, 0.0);
		else if(joint_right[4] > m_joint_right[4] && joint_right[4]-m_joint_right[4] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT5", -0.2, 0.0);
		else 
		{
			my->setJointVelocity("RARM_JOINT5", 0.0, 0.0);
			j5 = true;
		}
	else j5 = true;

	if(joint_right[5] != m_joint_right[5] )
		if(joint_right[5] < m_joint_right[5] && m_joint_right[5]-joint_right[5] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT6", 0.2, 0.0);
		else if(joint_right[5] > m_joint_right[5] && joint_right[5]-m_joint_right[5] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT6", -0.2, 0.0);
		else 
		{
			my->setJointVelocity("RARM_JOINT6", 0.0, 0.0);
			j6 = true;
		}
	else j6 = true;

	if(joint_right[6] != m_joint_right[6] )
		if(joint_right[5] < m_joint_right[6] && m_joint_right[6]-joint_right[5] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT7", 0.2, 0.0);
		else if(joint_right[5] > m_joint_right[6] && joint_right[5]-m_joint_right[6] > error_angle_arm)
			my->setJointVelocity("RARM_JOINT7", -0.2, 0.0);
		else 
		{
			my->setJointVelocity("RARM_JOINT7", 0.0, 0.0);
			j7 = true;
		}
	else j7 = true;

	if (j0 == true && j1 == true && j3 == true && j4 == true && j5 == true && j6 == true && j7 == true)
		return true;
	else
		return false;

	return false;
}

void ControlRobot::grasp_right_hand(std::string &object)
{
	Vector3d hand, obje;
	SimObj *obj = getObj(object.c_str());
	
	obj->getPosition(obje);
	my->getJointPosition(hand, "RARM_JOINT7");
	
	double distance = getDist3D(hand,obje);
	
	if(distance < 11 &&  m_grasp_right == false)
	{
		CParts * parts = my->getParts("RARM_LINK7");
        if (parts->graspObj(object))
        	m_grasp_right = true;
	}
}

void ControlRobot::release_right_hand()
{
	CParts * parts = my->getParts("RARM_LINK7");
   	if( m_grasp_right == true)
   	{
		broadcastMsg("Object_released");
		parts->releaseObj();
		m_grasp_right = false;
	}
}

void ControlRobot::chooze_task_arm_right(int task)
{
  for (int i=0; i<7; i++)
    m_joint_right[i] = tab_joint_right[task][i];
}

/************************************************************************************/
/*******************Find the point near the object and the table*********************/
/************************************************************************************/

Vector3d ControlRobot::PointApproachObj(std::string &table, std::string &object, double rangeToTable)
{
	Vector3d intermediatePoint, can_pos;
	SimObj *obj = getObj(object.c_str());

	obj->getPosition(can_pos);

	int obj_side = defineCoteTable(table, object);

	switch (obj_side)
	{
		case 1 :  	intermediatePoint =  Vector3d(can_pos.x() + rangeToTable, 0, can_pos.z());
					break;
		case 2 :  	intermediatePoint =  Vector3d(can_pos.x(), 0, can_pos.z() - rangeToTable);
					break;
		case 3 :  	intermediatePoint =  Vector3d(can_pos.x() - rangeToTable, 0, can_pos.z());
					break;
		case 4 :  	intermediatePoint =  Vector3d(can_pos.x(), 0, can_pos.z() + rangeToTable);
					break;
		default : 	intermediatePoint =  Vector3d(0, 0, 0);
					break;
	}
	return intermediatePoint;
}

int ControlRobot::defineCoteTable(std::string &table, std::string &object)
{
	Vector3d table_pos, can_pos;
	double TabObj_angle;
	int obj_side;

	double temp = M_PI/4;
	double temp2 = 3*M_PI/4;

	//initialisation
	SimObj *tab = getObj(table.c_str());
	SimObj *obj = getObj(object.c_str());

	tab->getPosition(table_pos);
	obj->getPosition(can_pos);

	//calculate angle vector between table and obj with X-axis
	TabObj_angle = getAngularXonVect(can_pos, table_pos);

	if (table == "Couch_Table" || table == "Kitchen_Table" || table == "Dinner_Table")
	{
		temp = 3*M_PI/8;
		temp2 = 5*M_PI/8;
	}
	else if ( table == "Buffet")
	{
		temp = M_PI/4;
		temp2 = 3*M_PI/4;
	}

	if ((TabObj_angle >= temp) && (TabObj_angle < temp2))
		obj_side = 1;
	else if ((TabObj_angle >= -temp) && (TabObj_angle < temp))
		obj_side = 4;
	else if ((TabObj_angle >= -temp2) && (TabObj_angle < -temp))
		obj_side = 3;
	else
		obj_side = 2;
		
  return obj_side;
}

Vector3d ControlRobot::ObjectPosition(std::string &name)
{
	Vector3d pos;
	
	// get object of trash selected
	SimObj *trash = getObj(name.c_str());

	// get trash's position
	trash->getPosition(pos);
	
	return pos;
}

/************************************************************************************/
/*******Find roll,pitch,yaw, lenght between to point, angle between to point*********/
/************************************************************************************/

double ControlRobot::getRoll(Rotation rot)
{
	// get angles arround y-axis
	double qw = rot.qw();  
	double qx = rot.qx(); 
	double qy = rot.qy(); 
	double qz = rot.qz();
	
	return atan2(2*qy*qw - 2*qx*qz, 1 - 2*qy*qy - 2*qz*qz);
}

double ControlRobot::getPitch(Rotation rot)
{
	// get angles arround y-axis
	double qw = rot.qw();  
	double qx = rot.qx(); 
	double qy = rot.qy(); 
	double qz = rot.qz();
	
	return atan2(2*qx*qw - 2*qy*qz, 1 - 2*qx*qx - 2*qz*qz);
}

double ControlRobot::getYaw(Rotation rot)
{
	// get angles arround y-axis
	double qw = rot.qw();  
	double qx = rot.qx(); 
	double qy = rot.qy(); 
	double qz = rot.qz();
	
	return asin(2*qx*qy + 2*qz*qw);
}

double ControlRobot::getDist2D(Vector3d pos, Vector3d pos2)
{
  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= pos2;

  return sqrt(l_pos.x()*l_pos.x()+l_pos.z()*l_pos.z());
}

double ControlRobot::getDist3D(Vector3d pos, Vector3d pos2)
{
  // pointing vector for target
  Vector3d l_pos = pos;
  l_pos -= pos2;

  return sqrt(l_pos.x()*l_pos.x()+l_pos.z()*l_pos.z()+l_pos.y()*l_pos.y());
}

double ControlRobot::getAngularXonVect(Vector3d pos, Vector3d mypos)
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

unsigned char ControlRobot::distance_sensor()
{  
	unsigned char distance = 255;
	
	// Calculate vertical angle view (radian) of a camera.  
	double fovy = my->getCamFOV() * M_PI / 180.0;  

	// Get aspect ratio  
	double ar = my->getCamAS();  

	// Calculate horizontal angle view (degree) of a camera.  
	double fovx = 2 * atan(tan(fovy*0.5)*ar) * 180.0 / M_PI;  
    
	if(m_view != NULL) 
	{    

		// Get distance vector along a horizontal plane  
		ViewImage *img = m_view->distanceSensor1D(0, 25, 3);    
		char *buf = img->getBuffer();    

		// Get length of the data array  
		int length = img->getBufferLength();    

		double theta = 0.0;  

		// Calculate minimum distance value in the array  
		for (int i = 0; i < length; i++)
		{    
			unsigned char tmp_distance = (unsigned char)buf[i];    
			if(tmp_distance < distance)
			{
		// Calculate view angle from index of an array  
				theta = fovx*i/319.0 - fovx/2.0;  
				distance = buf[i];
			}
		}
		delete img; 
	}

	std::cout << "Distance : " << (int)(distance) << std::endl;
	
	return distance;
}

Vector3d ControlRobot::changeBasis_WorldToRobot(Vector3d pos)
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
}

extern "C" Controller * createController() {
  return new ControlRobot;
}
