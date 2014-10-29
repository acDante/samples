#include <Controller.h>
#include <ControllerEvent.h>
#include <Logger.h>
#include <ViewImage.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class UserController : public Controller
{
public:
	void moveBodyByKINECT(char* msg);
	void moveHeadByHMD(const std::string ss);
	void onRecvMsg(RecvMsgEvent &evt);
	void onInit(InitEvent &evt);
	double onAction(ActionEvent &evt);
	bool slerp(dQuaternion qtn1, dQuaternion qtn2, double time, dQuaternion *dest);

private:
	double vel;  // Moving velocity
	ViewService *m_view;
	BaseService *m_kinect;
	BaseService *m_hmd;
	BaseService *m_wii;
	
	double m_posx, m_posy, m_posz; // Initial position
	double m_yrot;
	double m_range;
	
	int m_maxsize;                 // Maximum number of joints
	
	double m_qw, m_qy, m_qx, m_qz; // Quaternion(orientation) of body
	
	double pyaw, ppitch, proll;    // Sent value of roll, pitch, yaw in the previous loop
	
	std::string robotName;         // Name of robot

	bool init_flag;
	dQuaternion bodypartsQ_pre[5], bodypartsQ_now[5], bodypartsQ_middle[5];
};


void UserController::onInit(InitEvent &evt)
{
	robotName = "robot_000";

	m_kinect = NULL;
	m_hmd    = NULL;
	m_wii    = NULL;
	
	vel      = 10.0;
	srand(time(NULL));
	
	// Setting of initial position
	SimObj *my = this->getObj(this->myname());
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();
	m_range = 0.1;
	m_maxsize = 15;
	double qw = my->qw();
	double qy = my->qy();
	m_yrot = acos(fabs(qw))*2;
	if (qw*qy > 0)
		m_yrot = -1*m_yrot;
	
	// Orientation of whole body
	m_qw = 1.0;
	m_qx = 0.0;
	m_qy = 0.0;
	m_qz = 0.0;

	// Add by inamura on 28th June 2013
	my->setJointAngle ("RLEG_JOINT2", DEG2RAD(0));
	my->setJointAngle ("LLEG_JOINT2", DEG2RAD(0));
	my->setJointAngle ("RLEG_JOINT4", DEG2RAD(0));
	my->setJointAngle ("LLEG_JOINT4", DEG2RAD(0));

	// Add by inamura on 5th March 2014
	for (int i=0; i<5; i++) {
		for (int j=1; j<4; j++) {
			// Initial quaternion
			bodypartsQ_pre[i][0] = 1.0;
			bodypartsQ_pre[i][j] = 0.0;
		}
	}
	pyaw = 0.0; ppitch = 0.0; proll = 0.0;
	init_flag = true;
}


double UserController::onAction(ActionEvent &evt)
{
	// Check whether the serivces are available
	bool av_kinect = checkService("SIGKINECT");
	bool av_hmd =    checkService("SIGORS");
	bool av_wii =    checkService("Wii_Service");

	// Setup of Kinect
	if (av_kinect && m_kinect == NULL) {
		m_kinect = connectToService("SIGKINECT");
	}
	else if (!av_kinect && m_kinect != NULL) {
		m_kinect = NULL;
	}

	// Setup of Oculus Rift
	if (av_hmd && m_hmd == NULL) {
		// サービスに接続
		m_hmd = connectToService("SIGORS");
	}
	else if (!av_hmd && m_hmd != NULL) {
		m_hmd = NULL;
	}

	// Setup of WiiRemoteController
	if (av_wii && m_wii == NULL) {
		// サービスに接続
		m_wii = connectToService("Wii_Service");
	}
	else if (!av_wii && m_wii != NULL) {
		m_wii = NULL;
	}

	return 1.5;
}


void UserController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();

	SimObj *my = getObj(myname());

	char *all_msg = (char*)evt.getMsg();

	std::string ss = all_msg;
	// Extract header information
	int strPos1 = 0;
	int strPos2;
	std::string headss;
	std::string tmpss;
	strPos2 = ss.find(" ", strPos1);
	headss.assign(ss, strPos1, strPos2-strPos1);

	if (headss == "ORS_DATA") {
		// Control of head by HMD
		moveHeadByHMD(ss);
	}
	else if (headss == "KINECT_DATA") {
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
	else if (ss == "go") {
		sendMsg(robotName.c_str(), "go");
		LOG_MSG(("Starting the clean up task"));
		std::cout<<"go"<<std::endl;
	}
	else if (ss == "take" ) {
		sendMsg(robotName.c_str(), "take");
		LOG_MSG(("Taking the trash"));
		std::cout<<"take"<<std::endl;
	}
	else if (ss == "put" ) {
		sendMsg(robotName.c_str(), "put");
		LOG_MSG(("Putting the trash in the trash box"));
		std::cout<<"put"<<std::endl;
	}
	else if (ss == "init") {
		sendMsg(robotName.c_str(), "init");
	}
}


void UserController::moveHeadByHMD(const std::string ss)
{
	SimObj *my = this->getObj(this->myname());

	// Extract header information
	int strPos1 = 0;
	int strPos2;
	std::string headss;
	std::string tmpss;
	strPos2 = ss.find(" ", strPos1);
	headss.assign(ss, strPos1, strPos2-strPos1);

	if (headss == "ORS_DATA") {
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

		if (yaw == pyaw && pitch == ppitch && roll == proll)  return;
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


void UserController::moveBodyByKINECT(char* all_msg)
{
	SimObj *my = this->getObj(this->myname());
	char* msg = strtok(all_msg," ");
	if (strcmp(msg,"KINECT_DATA") == 0) {
		int i = 0;
		while (true) {
			i++;
			if (i == m_maxsize+1)
				break;
			char *type = strtok(NULL,":");
			if (strcmp(type,"POSITION") == 0) {
				// Position of body
				double x = atof(strtok(NULL,","));
				double y = atof(strtok(NULL,","));
				double z = atof(strtok(NULL," "));
				// Transformation from agent coordinate to global coordinate
				double gx = cos(m_yrot)*x - sin(m_yrot)*z;
				double gz = sin(m_yrot)*x + cos(m_yrot)*z;
				my->setPosition(m_posx+gx,m_posy+y,m_posz+gz);
				//limitedSetPosition(my, m_posx+gx,m_posy+y,m_posz+gz);
				continue;
			} 
			else if(strcmp(type,"WAIST") == 0) {
				static dQuaternion bodyQ_pre, bodyQ_now, bodyQ_middle;
				// Rotation of whole body
				double w = atof(strtok(NULL,","));
				double x = atof(strtok(NULL,","));
				double y = atof(strtok(NULL,","));
				double z = atof(strtok(NULL," "));
				// Spherical linear interpolation on quaternion

				bodyQ_pre[0] = m_qw; bodyQ_pre[1] = m_qx; bodyQ_pre[2] = m_qy; bodyQ_pre[3] = m_qz;
				bodyQ_now[0] =    w; bodyQ_now[1] =    x; bodyQ_now[2] =    y; bodyQ_now[3] =    z;
				slerp(bodyQ_pre, bodyQ_now, 0.5, &bodyQ_middle);

				my->setJointQuaternion("ROOT_JOINT0", bodyQ_middle[0], bodyQ_middle[1], bodyQ_middle[2], bodyQ_middle[3]);
				m_qw = bodyQ_middle[0];
				m_qx = bodyQ_middle[1];
				m_qy = bodyQ_middle[2];
				m_qz = bodyQ_middle[3];
				continue;
			}
			else if (strcmp(type,"END") == 0) {
				break;
			}
#if 0
			else {
				int    index;
				LOG_MSG(("smooth change"));
				if      (strcmp(type, "WAIST_JOINT1")==0) index = 0;
				else if (strcmp(type, "RARM_JOINT2" )==0) index = 1;
				else if (strcmp(type, "RARM_JOINT3" )==0) index = 2;
				else if (strcmp(type, "LARM_JOINT2" )==0) index = 3;
				else if (strcmp(type, "LARM_JOINT3" )==0) index = 4;
				else    continue;   // No need to control here. HEAD_JOINT1 is controlled by HMD
				// Rotation of each joint
				double w = atof(strtok(NULL,","));				double x = atof(strtok(NULL,","));
				double y = atof(strtok(NULL,","));				double z = atof(strtok(NULL," "));
				double angle = acos(w)*2;
				double tmp = sin(angle/2);
				double vx = x/tmp;				double vy = y/tmp;				double vz = z/tmp;
				double len = sqrt(vx*vx + vy*vy + vz*vz);
				if(len < (1 - m_range) || (1 + m_range) < len) continue;

				bodypartsQ_now[index][0] = w;   bodypartsQ_now[index][1] = x;   bodypartsQ_now[index][2] = y;  bodypartsQ_now[index][3] = z;
				slerp(bodypartsQ_pre[index], bodypartsQ_now[index], 0.5, &bodypartsQ_middle[index]);

				if (init_flag==false) {
					// Use interpolation from the 2nd time
					my->setJointQuaternion(type, bodypartsQ_middle[index][0], bodypartsQ_middle[index][1], bodypartsQ_middle[index][2], bodypartsQ_middle[index][3]);
					bodypartsQ_pre[index][0] = bodypartsQ_middle[index][0];
					bodypartsQ_pre[index][1] = bodypartsQ_middle[index][1];
					bodypartsQ_pre[index][2] = bodypartsQ_middle[index][2];
					bodypartsQ_pre[index][3] = bodypartsQ_middle[index][3];
					LOG_MSG(("%s, init", type));
				}
				else {
					// Use direct quaternion at the first time
					my->setJointQuaternion(type, w, x, y, z);
					bodypartsQ_pre[index][0] = w;
					bodypartsQ_pre[index][1] = x;
					bodypartsQ_pre[index][2] = y;
					bodypartsQ_pre[index][3] = z;
					init_flag = false;
					LOG_MSG(("%s, from 2nd", type));
				}
				continue;
			}
#else
			// Rotation of each joint
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
			// HEAD_JOINT1 is controlled by HMD
			if(strcmp(type,"HEAD_JOINT1") != 0 ){
				my->setJointQuaternion(type,w,x,y,z);
			}
#endif

		}
	}
}


bool UserController::slerp(dQuaternion qtn1, dQuaternion qtn2, double time, dQuaternion *dest)
{
	double ht = qtn1[0] * qtn2[0] + qtn1[1] * qtn2[1] + qtn1[2] * qtn2[2] + qtn1[3] * qtn2[3];
	double hs = 1.0 - ht * ht;
	if (hs <= 0.0) {
		*dest[0] = qtn1[0];
		*dest[1] = qtn1[1];
		*dest[2] = qtn1[2];
		*dest[3] = qtn1[3];
		return false;
	}
	else {
		hs = sqrt(hs);
		if (fabs(hs) < 0.0001) {
			*dest[0] = (qtn1[0] * 0.5 + qtn2[0] * 0.5);
			*dest[1] = (qtn1[1] * 0.5 + qtn2[1] * 0.5);
			*dest[2] = (qtn1[2] * 0.5 + qtn2[2] * 0.5);
			*dest[3] = (qtn1[3] * 0.5 + qtn2[3] * 0.5);
		}
		else {
			double ph = acos(ht);
			double pt = ph * time;
			double t0 = sin(ph - pt) / hs;
			double t1 = sin(pt) / hs;
			*dest[0] = qtn1[0] * t0 + qtn2[0] * t1;
			*dest[1] = qtn1[1] * t0 + qtn2[1] * t1;
			*dest[2] = qtn1[2] * t0 + qtn2[2] * t1;
			*dest[3] = qtn1[3] * t0 + qtn2[3] * t1;
		}
	}
	return true;
}


extern "C" Controller * createController ()
{
  return new UserController;
}
