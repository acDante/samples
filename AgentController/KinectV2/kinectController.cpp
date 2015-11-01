#include <string>
#include <math.h>
#include "sigverse/commonlib/Controller.h"
#include "sigverse/commonlib/Logger.h"
#include "sigverse/commonlib/ControllerEvent.h"
 
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )
 
using namespace std;
 
class AgentController : public Controller
{
public:
	double onAction(ActionEvent &evt);
	void onRecvMsg(RecvMsgEvent &evt);
	void onInit(InitEvent &evt);
 
private:
	//Initial position
	double m_posx, m_posy, m_posz;
	double m_yrot;
	double m_range;
	//Joint number maximum
	int m_maxsize;
 
	BaseService *m_kinect;
};
 
void AgentController::onInit(InitEvent &evt)
{
	//Initialize
	m_kinect = NULL;
	SimObj *my = getObj(myname());
   
	//get Initial position
	Vector3d pos;
	my->getPosition(pos);
	m_posx = pos.x();
	m_posy = pos.y();
	m_posz = pos.z();
   
	//Get Initial rotation
	Rotation rot;
	my->getRotation(rot);
	double qw = rot.qw();
	double qy = rot.qy();
   
	m_yrot = acos(fabs(qw))*2;
	if (qw*qy > 0)
		m_yrot = -1*m_yrot;
   
	m_range = 0.1;
	m_maxsize = 15;
}
 
double AgentController::onAction(ActionEvent &evt)
{
	//regular check of availability of the service
	bool available = checkService("SIGKINECT");
   
	//Service available
	if (available && m_kinect == NULL) {
		//connect to the service
		m_kinect = connectToService("SIGKINECT");
    }
    //Service unavailable
    else if (!available && m_kinect != NULL) 
		{
			m_kinect = NULL;
		}
	return 1.0;
}
 
void AgentController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
   
	//Get myself
	SimObj *my = getObj(myname());
   
	//Get message
	char *all_msg = (char*)evt.getMsg();
	//rintf("all_msg=\n%s\n",all_msg);

	char *msg = strtok(all_msg," ");
	if (strcmp(msg,"KINECT_DATA") == 0) {
		int i = 0;
		while (true) {
			i++;
			if (i == m_maxsize+1)
				break;
			char *type = strtok(NULL,":");
			//rintf("type=%s\n",type);
			//Body position
			if (strcmp(type,"POSITION") == 0) 
				{
					double x = atof(strtok(NULL,","));
					double y = atof(strtok(NULL,","));
					double z = atof(strtok(NULL," "));
					//Tlanslate Kinece coordinate system to SIGVerse coordinate system.
					double gx = cos(m_yrot)*x - sin(m_yrot)*z;
					double gz = sin(m_yrot)*x + cos(m_yrot)*z;
					//rintf("x=%f, y=%f, z=%f\n",x,y,z);
					//printf("m_yrot=%f, cos(m_yrot)=%f, sin(m_yrot)=%f\n",m_yrot,cos(m_yrot),sin(m_yrot));
					//printf("gx=%f, gz=%f\n",gx,gz);
					//printf("m_posx+gx=%f, m_posy+y=%f, m_posz+gz=%f\n",m_posx+gx,m_posy+y,m_posz+gz);
					my->setPosition(m_posx+gx,m_posy+y,m_posz+gz);
					//my->setPosition(m_posx+x,m_posy+y,m_posz+z);
					continue;
				}
			//Entire body rotation.
			else if (strcmp(type,"WAIST") == 0) 
				{
					double w = atof(strtok(NULL,","));
					double x = atof(strtok(NULL,","));
					double y = atof(strtok(NULL,","));
					double z = atof(strtok(NULL," "));
					my->setJointQuaternion("ROOT_JOINT0",w,x,y,z);
					continue;
				}
			else if (strcmp(type,"END") == 0)
				{
					printf("end of recvmsg\n");
					break;
					//Joint rotation
				}
			else 
				{
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
					//printf("type=%s,w=%f,x=%f,y=%f,z=%f\n",type,w,x,y,z);
					my->setJointQuaternion(type,w,x,y,z);
					continue;
				}
		}
	}
}
 
extern "C" Controller * createController ()
{
	return new AgentController;
} 
