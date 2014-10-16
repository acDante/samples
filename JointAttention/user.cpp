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
	void control_partner_eye(void);
	void onRecvMsg(RecvMsgEvent &evt);
	void onInit(InitEvent &evt);
	double onAction(ActionEvent &evt);

private:
	BaseService *m_kinect;
	BaseService *m_hmd;

	//Position of avatar
	double m_posx, m_posy, m_posz;
	double m_yrot;
	double m_range;

	//Max number of joints
	int m_maxsize;

	//Quaternion of body
	double m_qw, m_qy, m_qx, m_qz;

	//yaw, pitch and roll in the previous moment
	double pyaw, ppitch, proll;
};


void UserController::onInit(InitEvent &evt)
{
	m_kinect = NULL;
	m_hmd    = NULL;

	srand(time(NULL));

	//Initinal setting for position and so on
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

	//Set orientation of body
	m_qw = 1.0;
	m_qx = 0.0;
	m_qy = 0.0;
	m_qz = 0.0;

	//Do not move the legs
	my->setJointAngle ("RLEG_JOINT2", DEG2RAD(0));
	my->setJointAngle ("LLEG_JOINT2", DEG2RAD(0));
	my->setJointAngle ("RLEG_JOINT4", DEG2RAD(0));
	my->setJointAngle ("LLEG_JOINT4", DEG2RAD(0));

	pyaw = ppitch = proll = 0.0;
}


double UserController::onAction(ActionEvent &evt)
{
	// Check the availability of service providers
	bool av_kinect = checkService("SIGKINECT");
	bool av_hmd    = checkService("SIGORS");

	if (av_kinect && m_kinect == NULL) {
		// Connect to the Kinect service
		m_kinect = connectToService("SIGKINECT");

	}
	else if (!av_kinect && m_kinect != NULL) {
		m_kinect = NULL;
	}


	if (av_hmd && m_hmd == NULL) {
		// Connect to the Oculus Rift service
		m_hmd = connectToService("SIGORS");

	}
	else if (!av_hmd && m_hmd != NULL){
		m_hmd = NULL;
	}

	return 1.5;
}


void UserController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	SimObj *my = getObj(myname());

	//Receive message
	char *all_msg = (char*)evt.getMsg();

	std::string ss = all_msg;
	//Extract header information
	int strPos1 = 0;
	int strPos2;
	std::string headss;
	std::string tmpss;
	strPos2 = ss.find(" ", strPos1);
	headss.assign(ss, strPos1, strPos2-strPos1);


	//std::cout<<ss<<std::endl;

	if (headss == "ORS_DATA") {
		//Control the head based on the data from HMD
		moveHeadByHMD(ss);
	}
	else if (headss == "KINECT_DATA") {
		//Control the body based on the data from KINECT
		moveBodyByKINECT(all_msg);

		//Do not move the legs
		my->setJointAngle ("RLEG_JOINT2", DEG2RAD(0));
		my->setJointAngle ("LLEG_JOINT2", DEG2RAD(0));
		my->setJointAngle ("RLEG_JOINT4", DEG2RAD(0));
		my->setJointAngle ("LLEG_JOINT4", DEG2RAD(0));
		// Do not collide with a desk
		if (my->y() < 60)  my->y(60);
	}

	control_partner_eye();
}


void UserController::moveHeadByHMD(const std::string ss)
{
	SimObj *my = this->getObj(this->myname());

	//Extract header information
	int strPos1 = 0;
	int strPos2;
	std::string headss;
	std::string tmpss;
	strPos2 = ss.find(" ", strPos1);
	headss.assign(ss, strPos1, strPos2-strPos1);

	if (headss == "ORS_DATA") {
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
			if (i == m_maxsize+1) break;
			char *type = strtok(NULL,":");
			if (strcmp(type,"POSITION") == 0) {
				//Position of body
				double x = atof(strtok(NULL,","));
				double y = atof(strtok(NULL,","));
				double z = atof(strtok(NULL," "));
				//Transfer from agent coordinate to global coordinate
				double gx = cos(m_yrot)*x - sin(m_yrot)*z;
				double gz = sin(m_yrot)*x + cos(m_yrot)*z;
				// Do not move in yz-plane (??? Really?)
				// my->setPosition(m_posx+gx,m_posy+y,m_posz+gz); // No limitation
				my->setPosition(m_posx, m_posy, m_posz+gz);    	  // Move only for z-axis
				continue;
			}
			else if (strcmp(type,"WAIST") == 0) {
				//Orientation of body
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
			}
			else if (strcmp(type,"END") == 0)
				{
					break;
			}
			else {
				//Rotation of joints
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
				if (len < (1 - m_range) || (1 + m_range) < len) continue;
				// HEAD_JOINT1 is controlled by HMD
				if (strcmp(type,"HEAD_JOINT1") != 0 ) {
					my->setJointQuaternion(type,w,x,y,z);
				}
				continue;
			}
		}
	}

}


void UserController::control_partner_eye(void)
{
	char     msg_buf[64];
	Vector3d pos;
	SimObj *user = getObj("man_000");

	if (!user->getJointPosition(pos, "RARM_JOINT4"))
		LOG_ERR(("failed to get joint position"));

	//LOG_MSG(("position of right wrist = <%g, %g, %g>",pos.x(), pos.y(), pos.z() ));
	
	sprintf (msg_buf, "%d %d", (int)((93-pos.y())/6.0*20), (int)((pos.x()-17)/7.5*20));
	sendMsg("partner", msg_buf);
}

extern "C" Controller * createController ()
{
	return new UserController;
}
