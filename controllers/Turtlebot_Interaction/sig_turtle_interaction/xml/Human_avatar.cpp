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
	bool slerp(dQuaternion qtn1, dQuaternion qtn2, double time, dQuaternion dest);
    void moveByORSDK2(std::string ss);
private:

	//移動速度
	double vel;
	ViewService *m_view;
	BaseService *m_kinect;
	BaseService *m_hmd;
	BaseService *m_wii;
    BaseService *m_orsDK2;
    BaseService *m_Sensor;

   bool take;
   bool put;


	//初期位置
	double m_posx, m_posy, m_posz;
	double m_yrot;
	double m_range;
	
	//データ数（関節数）最大値
	int m_maxsize;
	
	// 体全体の角度
	//double m_qw, m_qy, m_qx, m_qz;
	
	// 前回送信したyaw, pitch roll
	double pyaw, ppitch, proll;
	

	double m_qw, m_qy, m_qx, m_qz;
	double o_qw, o_qx, o_qy, o_qz;
	double po_qw, po_qx, po_qy, po_qz;

	bool chk_orsDK2;
    bool restart_on;




	// ロボットの名前
	std::string robotName;

	bool init_flag;
	dQuaternion bodypartsQ_pre[5], bodypartsQ_now[5], bodypartsQ_middle[5];
};


void UserController::onInit(InitEvent &evt)
{
	robotName = "robot_000";
    take = true;
    put = false;

	//m_kinect = connectToService("SIGKINECT");
	//m_hmd = connectToService("SIGHMD");
	m_kinect = NULL;
	m_hmd = NULL;
	m_wii = NULL;
	m_orsDK2 = NULL;
	m_Sensor = NULL;
	
	vel      = 10.0;
	srand(time(NULL));
	
	//初期位置の設定
	SimObj *my = this->getObj(this->myname());
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();
	m_range = 0.1;
	//m_maxsize = 15;
	m_maxsize = 19;//20141126-tome-ikeda
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


	
	o_qw = 1.0;
	o_qx = 0.0;
	o_qy = 0.0;
	o_qz = 0.0;

   restart_on = false;



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
}

//定期的に呼び出される関数
double UserController::onAction(ActionEvent &evt)
{

  // サービスが使用可能か定期的にチェックする
  bool av_kinect = checkService("SIGKINECT");
  //bool av_hmd = checkService("SIGORS");
  bool av_wii = checkService("Wii_Service");

    bool av_Sensor = checkService("SIGSENSOR");

  bool chk_orsDK2 = checkService("SIGORSDK2");
SimObj *my = this->getObj(this->myname());
  // 使用可能
  if(av_kinect && m_kinect == NULL){
    // サービスに接続
    m_kinect = connectToService("SIGKINECT");

  }
  else if (!av_kinect && m_kinect != NULL){
    m_kinect = NULL;
  }


if(chk_orsDK2  && m_orsDK2 == NULL) {
		

		if(chk_orsDK2) {
			m_orsDK2 = connectToService("SIGORSDK2");
		}
}

  if(av_Sensor && m_Sensor == NULL){
    // サービスに接続
    m_Sensor = connectToService("SIGSENSOR");

  }
  // 使用可能

 //if(av_hmd && m_hmd == NULL){
    // サービスに接続
 //   m_hmd = connectToService("SIGORS");

 // }
//  else if (!av_hmd && m_hmd != NULL){
//  m_hmd = NULL;
//  }

  // 使用可能
  if(av_wii && m_wii == NULL){
    // サービスに接続
    m_wii = connectToService("Wii_Service");
  }
  else if (!av_wii && m_wii != NULL){
    m_wii = NULL;
  }
//printf("OnAction test \n");
  //my->setJointQuaternion("RARM_JOINT2", 0.707, 0, 0, 0.707);
  return 0.01;
}

void UserController::onRecvMsg(RecvMsgEvent &evt)
{




  std::string sender = evt.getSender();

  //自分自身の取得
  SimObj *my = getObj(myname());

  //メッセージ取得
  char *all_msg = (char*)evt.getMsg();
 // printf("all_msg=\n%s\n",all_msg);

  std::string ss = all_msg;
  //ヘッダーの取り出し
  int strPos1 = 0;
  int strPos2;
  std::string headss;
  std::string tmpss;
  strPos2 = ss.find(" ", strPos1);
  headss.assign(ss, strPos1, strPos2-strPos1);
std::cout << "the status of Hesssss " <<  headss << std::endl;

   if (strcmp(all_msg,"go") == 0 ) {
    	
    	//take == false;
    	//put == false;
        //printf("Man is taking \n");
    	//sendMsg("VoiceReco_Service","Cancel the action ..");
    	//sleep(4);
    	sendMsg("robot_000",all_msg);
         
    }

  else if (strcmp(all_msg,"restart") == 0 && restart_on == true) {
    	
    	//take == false;
    	//put == false;
        //printf("Man is taking \n");
    	sendMsg("VoiceReco_Service","Restart the clean up task ..");
    	//sleep(4);
    	sendMsg("robot_000",all_msg);
         
    }


   else if (strcmp(all_msg,"reset") == 0 ) {
    	
    	//take == false;
    	//put == false;
        //printf("Man is taking \n");
    	sendMsg("VoiceReco_Service","Cancel the action ..");
    	//sleep(4);
    	sendMsg("robot_000",all_msg);
         
    }
/*
   else if (strcmp(all_msg,"reset_take") == 0 ) { 	
    	take == true;
    	put == false;
        //printf("Man is taking \n");
    	//sendMsg("VoiceReco_Service","Take this Object");
    	//sleep(4);
    	//sendMsg("robot_000",all_msg);    
    }

   else if (strcmp(all_msg,"reset_put") == 0 ) {
    	
    	//take == false;
    	put == true;
        //printf("Man is taking \n");
    	//sendMsg("VoiceReco_Service","Take this Object");
    	//sleep(4);
    	//sendMsg("robot_000",all_msg);  
    }
    */
   else if (strcmp(all_msg,"On_Take") == 0 ) {
    	
    	//take == false;
    	take = true;
        //printf("Man is taking \n");
    	//sendMsg("VoiceReco_Service","Take this Object");
    	//sleep(4);
    	//sendMsg("robot_000",all_msg);      
    }
       else if (strcmp(all_msg,"On_put") == 0 ) {
    	
    	//take == false;
    	put = true;
        //printf("Man is taking \n");
    	//sendMsg("VoiceReco_Service","Take this Object");
    	//sleep(4);
    	//sendMsg("robot_000",all_msg);    
    }


   else if (strcmp(all_msg,"restart_on") == 0  ) {
    	
    	//take == false;
   	    restart_on = true;
    	//put == true;
        //printf("Man is taking \n");
    	//sendMsg("VoiceReco_Service","Take this Object");
    	//sleep(4);
    	//sendMsg("robot_000",all_msg);    
    }


   else if (strcmp(all_msg,"do") == 0 && take == true) {
    	
    	take = false;
       // printf("Man is taking \n");
    	sendMsg("VoiceReco_Service","Take this Object");
    	//sleep(4);
    	sendMsg("robot_000",all_msg);
         
    }


   else  if (strcmp(all_msg,"do") == 0 && put == true) {
    	put = false;
       //  printf("Man is putting \n");
    	sendMsg("VoiceReco_Service","Put it in that trash ");
    	//sleep(4);
    	sendMsg("robot_000",all_msg);
    	
    }




  //std::cout<<ss<<std::endl;

 // if(headss == "ORS_DATA"){
    //HMDデータによる頭部の動き反映
//    moveHeadByHMD(ss);
 // }




	std::string bodyss;
	bodyss.assign(ss, strPos2 + 1, ss.length() - strPos2);

	if(headss == "ORS_DATA") {
		moveByORSDK2(bodyss);
	}


  else if(headss == "KINECT_DATA") {
	  //KINECTデータによる頭部以外の体の動き反映
	  moveBodyByKINECT(all_msg);
	  // Add by inamura on 2014-03-02
	  // comment out by tome-ikeda 2014-11-06
	  //my->setJointAngle ("RLEG_JOINT2", DEG2RAD(0));
	  //my->setJointAngle ("LLEG_JOINT2", DEG2RAD(0));
	  //my->setJointAngle ("RLEG_JOINT4", DEG2RAD(0));
	  //my->setJointAngle ("LLEG_JOINT4", DEG2RAD(0));
	  // Do not collide with a desk
	  //f (my->y() < 60)  my->y(60);
  }

  else if(ss == "go") {
    sendMsg("robot_000","go");
    LOG_MSG(("Starting the clean up task"));
    std::cout<<"go"<<std::endl;
  }

  else if(ss == "take" ) {
//    sendMsg("robot_000","take");
//    LOG_MSG(("Taking the trash"));
    std::cout<<"take"<<std::endl;
  }

  else if(ss == "put" ) {
  //  sendMsg("robot_000","put");
 //   LOG_MSG(("Putting the trash in the trash box"));
    std::cout<<"put"<<std::endl;
  }

  else if(ss == "init") {
    sendMsg("robot_000","init");
  }
}


void UserController::moveHeadByHMD(const std::string ss)
{
	//自分自身の取得
	SimObj *my = this->getObj(this->myname());

	//ヘッダーの取り出し
	int strPos1 = 0;
	int strPos2;
	std::string headss;
	std::string tmpss;
	strPos2 = ss.find(" ", strPos1);
	headss.assign(ss, strPos1, strPos2-strPos1);

	if(headss == "ORS_DATA"){
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






void UserController::moveByORSDK2(std::string ss)
{
	SimObj *my = getObj(myname());

	int strPos1 = 0;
	int strPos2;
	std::string tmpss;

	strPos2 = ss.find(",", strPos1);
	tmpss.assign(ss, strPos1, strPos2 - strPos1);
	o_qw = atof(tmpss.c_str());

	strPos1 = strPos2 + 1;
	strPos2 = ss.find(",", strPos1);
	tmpss.assign(ss, strPos1, strPos2 - strPos1);
	o_qx = atof(tmpss.c_str());

	strPos1 = strPos2 + 1;
	strPos2 = ss.find(",", strPos1);
	tmpss.assign(ss, strPos1, strPos2 - strPos1);
	o_qy = atof(tmpss.c_str());

	strPos1 = strPos2 + 1;
	strPos2 = ss.find(",", strPos1);
	tmpss.assign(ss, strPos1, strPos2 - strPos1);
	o_qz = atof(tmpss.c_str());

	if (o_qw == po_qw && o_qx == po_qx && o_qy == po_qy && o_qz == po_qz)  return;
	else {
		po_qw = o_qw;
		po_qx = o_qx;
		po_qy = o_qy;
		po_qz = o_qz;
	}

	dQuaternion headQ;
	headQ[0] = o_qw;
	headQ[1] = o_qx;
	headQ[2] = o_qy;
	headQ[3] = o_qz;

	dQuaternion bodyQ;
	bodyQ[0] = m_qw;
	bodyQ[1] = m_qx;
	bodyQ[2] = m_qy;
	bodyQ[3] = m_qz;

	dQuaternion tmpQ;
	dQMultiply1(tmpQ, bodyQ, headQ);

	my->setJointQuaternion("HEAD_JOINT0", tmpQ[0], tmpQ[1], tmpQ[2], tmpQ[3]);

}


















void UserController::moveBodyByKINECT(char* all_msg)
{
	//自分自身の取得
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
				//体の位置
				double x = atof(strtok(NULL,","));
				double y = atof(strtok(NULL,","));
				double z = atof(strtok(NULL," "));
				//エージェント座標からグローバル座標への変換
				double gx = cos(m_yrot)*x - sin(m_yrot)*z;
				double gz = sin(m_yrot)*x + cos(m_yrot)*z;
        	//	printf("x=%f, y=%f, z=%f\n",x,y,z);
        	///	printf("m_yrot=%f, cos(m_yrot)=%f, sin(m_yrot)=%f\n",m_yrot,cos(m_yrot),sin(m_yrot));
        	///	printf("gx=%f, gz=%f\n",gx,gz);
        	//	printf("m_posx=%f, m_posy=%f, m_posz=%f\n",m_posx,m_posy,m_posz);
        		//printf("m_posx+gx=%f, m_posy+y=%f, m_posz+gz=%f\n",m_posx+gx,m_posy+y,m_posz+gz);
				my->setPosition(m_posx+gx,m_posy,m_posz+gz);
				//printf("the avatar postion is  X : %f  ----- Y: %f ------ Z :  %f ---- end \n",m_posx+gx,m_posy+y,m_posz+gz);
				continue;
			} 
			else if(strcmp(type,"WAIST") == 0) {
				static dQuaternion bodyQ_pre, bodyQ_now, bodyQ_middle;
				//体全体の回転 rotation of whole body
				double w = atof(strtok(NULL,","));
				double x = atof(strtok(NULL,","));
				double y = atof(strtok(NULL,","));
				double z = atof(strtok(NULL," "));
				m_qw = w;
				m_qx = x;
				m_qy = y;
				m_qz = z; 
				// Spherical linear interpolation on quaternion
				/*
				bodyQ_pre[0] = m_qw; bodyQ_pre[1] = m_qx; bodyQ_pre[2] = m_qy; bodyQ_pre[3] = m_qz;
				bodyQ_now[0] =    w; bodyQ_now[1] =    x; bodyQ_now[2] =    y; bodyQ_now[3] =    z;
				//slerp(bodyQ_pre, bodyQ_now, 0.5, &bodyQ_middle);
				slerp(bodyQ_pre, bodyQ_now, 0.5, bodyQ_middle);
				printf("WAIST    =(%f,%f,%f,%f)\n",w,               x,               y,               z);
				printf("bodyQ_middle=(%f,%f,%f,%f)\n",bodyQ_middle[0], bodyQ_middle[1], bodyQ_middle[2], bodyQ_middle[3]);//wrong
				my->setJointQuaternion("ROOT_JOINT0", bodyQ_middle[0], bodyQ_middle[1], bodyQ_middle[2], bodyQ_middle[3]);
				m_qw = bodyQ_middle[0];
				m_qx = bodyQ_middle[1];
				m_qy = bodyQ_middle[2];
				m_qz = bodyQ_middle[3];
				*/
				my->setJointQuaternion("ROOT_JOINT0", w, x, y, z);
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


bool UserController::slerp(dQuaternion qtn1, dQuaternion qtn2, double time, dQuaternion dest)
{
	double ht = qtn1[0] * qtn2[0] + qtn1[1] * qtn2[1] + qtn1[2] * qtn2[2] + qtn1[3] * qtn2[3];
	double hs = 1.0 - ht * ht;
	if (hs <= 0.0) {//when hs is over 1.0
		dest[0] = qtn1[0];
		dest[1] = qtn1[1];
		dest[2] = qtn1[2];
		dest[3] = qtn1[3];
		printf("false\n");
		return false;
	}
	else {//when hs is under 1.0
		hs = sqrt(hs);
		if (fabs(hs) < 0.0001) { //Absolute value of hs is near 0
			dest[0] = (qtn1[0] * 0.5 + qtn2[0] * 0.5);
			dest[1] = (qtn1[1] * 0.5 + qtn2[1] * 0.5);
			dest[2] = (qtn1[2] * 0.5 + qtn2[2] * 0.5);
			dest[3] = (qtn1[3] * 0.5 + qtn2[3] * 0.5);
			printf("near 0\n");
		}
		else { //Absolute value of hs is not near 0
			double ph = acos(ht);
			double pt = ph * time;
			double t0 = sin(ph - pt) / hs;
			double t1 = sin(pt) / hs;
			dest[0] = qtn1[0] * t0 + qtn2[0] * t1;
			dest[1] = qtn1[1] * t0 + qtn2[1] * t1;
			dest[2] = qtn1[2] * t0 + qtn2[2] * t1;
			dest[3] = qtn1[3] * t0 + qtn2[3] * t1;
			printf("w=%fx=%fy=%fz=%f\t\n",dest[0],dest[1],dest[2],dest[3]);
		}
	}
	return true;
}



extern "C" Controller * createController ()
{
  return new UserController;
}
