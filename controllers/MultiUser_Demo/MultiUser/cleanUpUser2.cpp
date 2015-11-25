#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <exception>

#include "sigverse/commonlib/Controller.h"
#include "sigverse/commonlib/ControllerEvent.h"
#include "sigverse/commonlib/Logger.h"
#include "sigverse/commonlib/ViewImage.h"

#define SOCKET_ERROR        -1
typedef struct sockaddr 	*LPSOCKADDR;

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class UserController : public Controller
{
public:
	void moveBodyByKINECT(char* all_msg);
	void moveHeadByHMD(const std::string ss);
	void onRecvMsg(RecvMsgEvent &evt);
	void onInit(InitEvent &evt);
	double onAction(ActionEvent &evt);
	bool slerp(dQuaternion qtn1, dQuaternion qtn2, double time, dQuaternion *dest);
    double getDis(Vector3d pos);
 //   void onCollision(CollisionEvent &evt);
private:

	//移動速度
	double vel;
	ViewService *m_view;
	BaseService *m_kinect;
	BaseService *m_hmd;
	BaseService *m_wii;

	//初期位置
	double m_posx, m_posy, m_posz;
	double m_yrot;
	double m_range;

	//データ数（関節数）最大値
	int m_maxsize;

	// 体全体の角度
	double m_qw, m_qy, m_qx, m_qz;

	// 前回送信したyaw, pitch roll
	double pyaw, ppitch, proll;

	// ロボットの名前
	std::string robotName;

	bool init_flag;
	dQuaternion bodypartsQ_pre[5], bodypartsQ_now[5], bodypartsQ_middle[5];
  int priority;
  int inc;
  std::vector<std::string> IDS;
  std::string F_ID;
  bool activation;
  std::vector<std::string> service_ID;
  std::vector<std::string> att_service_ID;
  bool id_init;
  bool m_grasp;
  bool m_grasp1;  
  bool m_grasp2; 
  SimObj *other; 
  SimObj *bottle; 
    bool back1;
    bool back2;

   bool condition_grasp ;
   bool condition_release ;
};


double UserController::getDis(Vector3d pos)
{
	
	// measure actual distance
	double distance = sqrt((pos.x()*pos.x())+(pos.y()*pos.y())+(pos.z()*pos.z()));
	
	return distance;
}


void UserController::onInit(InitEvent &evt)
{

	condition_grasp = true;
	condition_release = true;


	other = getObj("man_blue");
    bottle = getObj("petbottle_0");

	id_init = true;
    F_ID = "";
    IDS.push_back("");
	priority = 1;
	inc = 0 ;
	activation = false;
	robotName = "robot_000";

	service_ID.push_back("SIGORS ");
	service_ID.push_back("SIGKINECT ");
    service_ID.push_back("Wii_Service ");
    att_service_ID.push_back("");
    att_service_ID.push_back("");
    att_service_ID.push_back("");


	//m_kinect = connectToService("SIGKINECT");
	//m_hmd = connectToService("SIGHMD");
	m_kinect = NULL;
	m_hmd = NULL;
	m_wii = NULL;

	vel      = 10.0;
	srand(time(NULL));

	//初期位置の設定
	SimObj *my = this->getObj(this->myname());
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();
	m_range = 0.1;
	m_maxsize = 15;
	double qw = my->qw();
	double qy = my->qy();
	m_yrot = acos(fabs(qw))*2;
	if(qw*qy > 0)
		m_yrot = -1*m_yrot;

	// 体全体の向き
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

	pyaw = ppitch = proll = 0.0;
	init_flag = true;
    m_grasp = false;
    back1 = false;
    back2 = false;

}

//定期的に呼び出される関数
double UserController::onAction(ActionEvent &evt)
{
	Vector3d pos_Obj;
	Vector3d pos_hand;
	Vector3d pos_hand_me;

	CParts * parts = other->getParts("RARM_LINK7");

	bottle->getPosition(pos_Obj);
	parts->getPosition(pos_hand);
	pos_hand -= pos_Obj;

	double distance = getDis(pos_hand);

 

	//std::cout<<"Man pink" <<  distance <<std::endl;
	SimObj *my = this->getObj(this->myname());
	double angL1 = my->getJointAngle("RARM_JOINT0")*180.0/(M_PI);

	CParts * parts2 = my->getParts("RARM_LINK7");
	parts2->getPosition(pos_hand_me);
	pos_hand_me -= pos_Obj;
	double distance2 = getDis(pos_hand_me);


	if(distance < 6 && m_grasp == true && condition_release == true)
		{
			CParts * parts2 = my->getParts("RARM_LINK7");
			parts2->releaseObj();
			//   m_grasp = false;    
			//  m_grasp1 = false;
			m_grasp = false;
			condition_grasp = false;
			//
		}

	// condition de rerelease object
	if(distance > 11  && m_grasp == true )
		{
			//m_grasp1 = false;
			//m_grasp = false; 
			condition_release = true;
		}


	// condtion to regrasp object

	if(distance2 > 11  && m_grasp == false )
		{
			//m_grasp1 = false;
			//m_grasp = false; 
			condition_grasp = true;
		}




	if(distance2 < 5 &&  m_grasp == false && condition_grasp == true )
		{

			//std::cout<<"Grasping blue" << std::endl;
			CParts * parts = my->getParts("RARM_LINK7");
			if(parts->graspObj("petbottle_0"))
				{

					m_grasp = true;
				}
			m_grasp = true;
			condition_release = false;
		}




	double thetaL1 = -120+angL1;

   

	bool av_kinect = checkService(service_ID[1]);
	bool av_hmd = checkService(service_ID[0]);
	bool av_wii = checkService(service_ID[2]);

	// }

	// 使用可能
	if(av_kinect && m_kinect == NULL){
		// サービスに接続

    
		m_kinect = connectToService(service_ID[1]);
		sendMsg("multiuserctr_0","OK");

	}
	else if (!av_kinect && m_kinect != NULL){
		m_kinect = NULL;
	}

	// 使用可能
	if(av_hmd && m_hmd == NULL){
		// サービスに接続
		m_hmd = connectToService(service_ID[0]);
  
	}
	else if (!av_hmd && m_hmd != NULL){
		m_hmd = NULL;
	}

	// 使用可能
	if(av_wii && m_wii == NULL){
		// サービスに接続
		m_wii = connectToService(service_ID[2]);
	}
	else if (!av_wii && m_wii != NULL){
		m_wii = NULL;
	}

	return 0.01;
}

void UserController::onRecvMsg(RecvMsgEvent &evt)
{

	std::string sender = evt.getSender();

	//自分自身の取得
	SimObj *my = getObj(myname());

	//メッセージ取得
	char *all_msg = (char*)evt.getMsg();

	std::string ss = all_msg;
	//ヘッダーの取り出し
	int strPos1 = 0;
	int strPos2,strPos3;
	std::string headss,cmd;
	std::string tmpss,id;

	strPos2 = ss.find(" ", strPos1);
	headss.assign(ss, strPos1, strPos2-strPos1);
	int jj = ss.size();
    cmd.assign(ss, strPos2+1,jj);

	strPos3 = cmd.find("|", strPos1);
	id.assign(cmd, strPos1, strPos3-strPos1+1);
    std::cout<<"The ID is :"<< id <<std::endl;
	// attrId(priority,id);


	if(sender == "multiuserctr_0" &&  id_init == true)
		{

			for(int i=0;i< service_ID.size() ;i++)
				{

					service_ID[i] += ss;
					//std::cout<<"The service ID :"<< service_ID[i] <<std::endl;
					F_ID = ss;
					//std::cout<<" F_ID is :"<< F_ID <<std::endl;
				}
			id_init = false;
			std::string mf = "MultiUsersMenu ";
			mf += F_ID;
			sendMsg(mf,"Connected to Avatar 2");

		}
	//std::cout<<ss<<std::endl;

	else if(headss == "ORS_DATA" && F_ID == id){
		//HMDデータによる頭部の動き反映

		moveHeadByHMD(ss);
	}
	else if(headss == "KINECT_DATA" && F_ID == id) {
		//KINECTデータによる頭部以外の体の動き反映
  	    std::cout<<"Hello world :)" <<std::endl;
		moveBodyByKINECT(all_msg);
		// Add by inamura on 2014-03-02
		my->setJointAngle ("RLEG_JOINT2", DEG2RAD(0));
		my->setJointAngle ("LLEG_JOINT2", DEG2RAD(0));
		my->setJointAngle ("RLEG_JOINT4", DEG2RAD(0));
		my->setJointAngle ("LLEG_JOINT4", DEG2RAD(0));
		// Do not collide with a desk
		if (my->y() < 60)  my->y(60);
	}
  
}


void UserController::moveHeadByHMD(const std::string ss)
{
	//自分自身の取得
	SimObj *my = this->getObj(this->myname());

	//ヘッダーの取り出し
	int strPos1 = 0;
	int strPos2,strPos3;
	std::string headss, cmd;
	std::string tmpss,id;
	strPos2 = ss.find("|", strPos1);
	headss.assign(ss, strPos1, strPos2-strPos1+1);
	std:: string str = "ORS_DATA ";
	str+= F_ID;

	std::cout<<"head :"<<  headss << std::endl;
	std::cout<<"str  :"<<  str << std::endl;


	if(headss == str){
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

		if(yaw == pyaw && pitch == ppitch && roll == proll)  return;
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
	//自分自身の取得
	SimObj *my = this->getObj(this->myname());
	//char* msg = strtok(all_msg," ");
	int strPos1 = 0;
	int strPos2,strPos3;
	std::string headss, cmd;
	char * msg;
	std::string tmpss,id;
	//strPos2 = all_msg.find("|", strPos1);
	msg = strtok(all_msg,"|");
	//msg.assign(all_msg, strPos1, strPos2-strPos1+1);
	std:: string str = "KINECT_DATA ";
	str+= F_ID;
	str = str.erase(str.size() - 1, 1);
    // std::cout<<"msg :"<<  msg << std::endl;
	//  std::cout<<"str :"<<  str << std::endl;

	if (strcmp(msg,str.c_str()) == 0) {
		std::cout<<"Next hello world" <<std::endl;
		int i = 0;
		while (true) {
			i++;
			if (i == m_maxsize+1)
				break;
			char *type = strtok(NULL,":");
			if (strcmp(type,"POSITION") == 0) {
				//体の位置
				double x = atof(strtok(NULL,","));
				double y = atof(strtok(NULL,","));
				double z = atof(strtok(NULL," "));
				//エージェント座標からグローバル座標への変換
				double gx = cos(m_yrot)*x - sin(m_yrot)*z;
				double gz = sin(m_yrot)*x + cos(m_yrot)*z;
				//my->setPosition(m_posx+gx,m_posy+y,m_posz+gz);
				//limitedSetPosition(my, m_posx+gx,m_posy+y,m_posz+gz);
				if ( gx < 30 &&  gx > -30  && gz <  40  &&  gz >  -40  )
					{ 
						my->setPosition(m_posx+gx,m_posy,m_posz+gz);
					}
                else if (gx > 30  && gz <  40  &&  gz >  -40 )
					{
						my->setPosition(m_posx+ 29,m_posy,m_posz+gz);
					}
				else if (gx < -30  && gz <  40  &&  gz >  -40)
					{
						my->setPosition(m_posx - 29,m_posy,m_posz+gz);
					}
                else if (gz < -40 && gx < 30 &&  gx > -30) 
					{
						my->setPosition(m_posx+gx,m_posy,m_posz-39);
					}  
				else if (gz > 40 && gx < 30 &&  gx > -30)
					{
						my->setPosition(m_posx+gx,m_posy,m_posz+39);

					} 
                else if (gx > 30 && gz < -40)
					{
						my->setPosition(m_posx+ 29,m_posy,m_posz-39);
					}
                else if (gx > 30 && gz >40)
					{
						my->setPosition(m_posx+ 29,m_posy,m_posz+39);
					}
                else if (gx < -30 && gz < -40 )
					{
						my->setPosition(m_posx - 29,m_posy,m_posz - 39);
					}
				else if (gx < -30 && gz > 40 )
					{
						my->setPosition(m_posx - 29,m_posy,m_posz + 39);
					}



				continue;
			}
			else if(strcmp(type,"WAIST") == 0) {
				static dQuaternion bodyQ_pre, bodyQ_now, bodyQ_middle;
				//体全体の回転
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
				else    continue;   // HEAD_JOINT1はHMDにより回転させるのでここで処理する必要なし
				//関節の回転
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
			//関節の回転
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
			// HEAD_JOINT1はHMDにより回転
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
/*
  void UserController::onCollision(CollisionEvent &evt)
  {
  //m_grasp = bottle->getIsGrasped();
  if (m_grasp == false && m_grasp2 == false){
  //	sendMsg("man_blue","graping");
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
  if(parts->graspObj("petbottle_0")){
  m_grasp2 = true;
  m_grasp1 = false;
  //m_grasp = bottle->getIsGrasped();
  m_grasp = true;
  }
  }
  }
  }
  }

*/


extern "C" Controller * createController ()
{
	return new UserController;
}
