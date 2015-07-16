#include <Controller.h>
#include <ControllerEvent.h>
#include <Logger.h>
#include <ViewImage.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <string>
#include <unistd.h>

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class UserController : public Controller
{
public:
	void moveBodyByKINECT(char* msg);
	void moveHeadByHMD(const std::string ss);
	void onRecvMsg(RecvMsgEvent &evt);
	void onInit(InitEvent &evt);
	double onAction(ActionEvent &evt);
   double getRoll(Rotation rot);
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

	FILE* fp;
	std::string file_name;
	//std::vector<std::string> object;
	float stepWidth;
	int sleeptime;
	const static int SIZE = 14;
	int motionNum;
	float HEIGHT[SIZE];
	float LARM_JOINT1[SIZE]; // left shoulder
	float LARM_JOINT3[SIZE]; // left elbow
	float RARM_JOINT1[SIZE]; // right shoulder
	float RARM_JOINT3[SIZE]; // right elbow
	float LLEG_JOINT2[SIZE]; // left groin(leg)
	float LLEG_JOINT4[SIZE]; // left knee
	float LLEG_JOINT6[SIZE]; // left ankle
	float RLEG_JOINT2[SIZE]; // right groin(leg)
	float RLEG_JOINT4[SIZE]; // right knee
	float RLEG_JOINT6[SIZE]; // right ankle

	bool start;
	bool end;
	bool stop;
	bool move;

	int count;
	int step;
 SimObj *my ;
	double angle;

 	double dx,dy,dz;
	int i;
	int motion ;
bool next ;
bool rotate ;
};

double UserController::getRoll(Rotation rot)
{
  // get angles arround y-axis
  double qw = rot.qw();
  double qx = rot.qx();
  double qy = rot.qy();
  double qz = rot.qz();

  double roll  = atan2(2*qy*qw - 2*qx*qz, 1 - 2*qy*qy - 2*qz*qz);

  return roll;
}

void UserController::onInit(InitEvent &evt)
{
	robotName = "robot_000";
    my = getObj(myname());
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

my->setAxisAndAngle(0,1.0, 0, -1.57);


	pyaw = ppitch = proll = 0.0;
	init_flag = true;
// Put arms down
	my->setJointAngle("LARM_JOINT2", DEG2RAD(-90));
	my->setJointAngle("RARM_JOINT2", DEG2RAD(90));

	stepWidth = 15;

	if((fp = fopen("motion.txt", "r")) == NULL) {
		printf("File do not exist.\n");
		exit(0);
	}
	else{
		fscanf(fp, "%d", &motionNum);
		fscanf(fp, "%d", &sleeptime);
		for(int i=0; i<motionNum; i++){
			fscanf(fp, "%f %f %f %f %f %f %f %f %f %f %f",
					   &HEIGHT[i],

					   &LARM_JOINT1[i],
					   &LARM_JOINT3[i],
					   &RARM_JOINT1[i],
					   &RARM_JOINT3[i],
					   &LLEG_JOINT2[i],
					   &LLEG_JOINT4[i],
					   &LLEG_JOINT6[i],
					   &RLEG_JOINT2[i],
					   &RLEG_JOINT4[i],
					   &RLEG_JOINT6[i]);
		}
	}
for(int i=0; i<motionNum; i++){
//std::cout << " Height   " << i <<  "    ......   " <<  HEIGHT[i] <<std::endl;


	}

	start = false;
	end = false;
	stop = true;
    move = false;
    next = true;
    dx=0;
	dy=0;
	dz=0;

angle = 0;
motion = 0;
rotate = false;
}

//定期的に呼び出される関数
double UserController::onAction(ActionEvent &evt)
{

   Vector3d pos;

	Vector3d ownPosition;

	my->getPosition(pos);

  // サービスが使用可能か定期的にチェックする
  bool av_kinect = checkService("XBox_Service");
  bool av_hmd = checkService("SIGORS");


  // 使用可能
  if(av_kinect && m_kinect == NULL){
    // サー
    m_kinect = connectToService("XBox_Service");

  }
  else if (!av_kinect && m_kinect != NULL){
    m_kinect = NULL;
  }

  // 使用可能
  if(av_hmd && m_hmd == NULL){
    // サービスに接続
    m_hmd = connectToService("SIGORS");

  }
  else if (!av_hmd && m_hmd != NULL){
    m_hmd = NULL;
  }


        if (rotate)
         {
            move = false;
            stop = true ;
         	my->setAxisAndAngle(0,1.0, 0, angle);

          if(next){
				for(int j=0; j<motionNum; j++){
         	        usleep(500);
					my->setPosition(pos.x(), HEIGHT[j], pos.z());
					my->setJointAngle("LARM_JOINT1", DEG2RAD(LARM_JOINT1[j]));
					my->setJointAngle("LARM_JOINT3", DEG2RAD(LARM_JOINT3[j]));
					my->setJointAngle("RARM_JOINT1", DEG2RAD(RARM_JOINT1[j]));
					my->setJointAngle("RARM_JOINT3", DEG2RAD(RARM_JOINT3[j]));
					my->setJointAngle("LLEG_JOINT2", DEG2RAD(LLEG_JOINT2[j]));
					my->setJointAngle("LLEG_JOINT4", DEG2RAD(LLEG_JOINT4[j]));
					my->setJointAngle("LLEG_JOINT6", DEG2RAD(LLEG_JOINT6[j]));
					my->setJointAngle("RLEG_JOINT2", DEG2RAD(RLEG_JOINT2[j]));
					my->setJointAngle("RLEG_JOINT4", DEG2RAD(RLEG_JOINT4[j]));
					my->setJointAngle("RLEG_JOINT6", DEG2RAD(RLEG_JOINT6[j]));


				}
			next = false;
			}
			else{
				for(int j=0; j<motionNum; j++){
                      usleep(500);
					my->setPosition(pos.x(), HEIGHT[j], pos.z());
					my->setJointAngle("RARM_JOINT1", DEG2RAD(LARM_JOINT1[j]));
					my->setJointAngle("RARM_JOINT3", DEG2RAD(-LARM_JOINT3[j]));
					my->setJointAngle("LARM_JOINT1", DEG2RAD(RARM_JOINT1[j]));
					my->setJointAngle("LARM_JOINT3", DEG2RAD(-RARM_JOINT3[j]));
					my->setJointAngle("RLEG_JOINT2", DEG2RAD(LLEG_JOINT2[j]));
					my->setJointAngle("RLEG_JOINT4", DEG2RAD(LLEG_JOINT4[j]));
					my->setJointAngle("RLEG_JOINT6", DEG2RAD(LLEG_JOINT6[j]));
					my->setJointAngle("LLEG_JOINT2", DEG2RAD(-RLEG_JOINT2[j]));
					my->setJointAngle("LLEG_JOINT4", DEG2RAD(RLEG_JOINT4[j]));
					my->setJointAngle("LLEG_JOINT6", DEG2RAD(RLEG_JOINT6[j]));

				}

				next = true;
			}
	     	motion = motion + 1 ;

      // std::cout << "The Count is  " << count <<std::endl;
			// 目標点に到着
			if(motion > 2){
				rotate = false;
				std::cout << "I stop rotation "  <<std::endl;
			    motion = 0;
			}
         }



if(move){


			dx=(m_posx-pos.x());
			dz=(m_posz-pos.z());

			double r = sqrt(pow(dx,2)+pow(dz,2));
			step = 2 * (int)r / stepWidth;
			dx /= step*motionNum*2;
			dz /= step*motionNum*2;
      //      std::cout << " The dx ... " << dx  <<  " The dz ... " << dz <<  " R ... "  << r << std::endl;

            angle = atan2(dx,dz);
			my->setAxisAndAngle(0,1.0, 0, angle);
            stop = false;

	}



		if(stop != true  ){
			double addx = 0.0;
			double addz = 0.0;


//std::cout << " The Step " << step << " The motionNum " <<  motionNum << " The count " << count << " The dx " << dx  <<  " The dz " << dz << " Height   "<< HEIGHT[i]  <<" And I is    " <<  i << std::endl;
			if(count%2){
				for(int j=0; j<motionNum; j++){
					addx += dx;
					addz += dz;
					//usleep(sleeptime);
					usleep(2000);
					my->setPosition(pos.x()+addx, HEIGHT[j], pos.z()+addz);
					my->setJointAngle("LARM_JOINT1", DEG2RAD(LARM_JOINT1[j]));
					my->setJointAngle("LARM_JOINT3", DEG2RAD(LARM_JOINT3[j]));
					my->setJointAngle("RARM_JOINT1", DEG2RAD(RARM_JOINT1[j]));
					my->setJointAngle("RARM_JOINT3", DEG2RAD(RARM_JOINT3[j]));
					my->setJointAngle("LLEG_JOINT2", DEG2RAD(LLEG_JOINT2[j]));
					my->setJointAngle("LLEG_JOINT4", DEG2RAD(LLEG_JOINT4[j]));
					my->setJointAngle("LLEG_JOINT6", DEG2RAD(LLEG_JOINT6[j]));
					my->setJointAngle("RLEG_JOINT2", DEG2RAD(RLEG_JOINT2[j]));
					my->setJointAngle("RLEG_JOINT4", DEG2RAD(RLEG_JOINT4[j]));
					my->setJointAngle("RLEG_JOINT6", DEG2RAD(RLEG_JOINT6[j]));
				}
			}
			else{
				for(int j=0; j<motionNum; j++){
					addx += dx;
					addz += dz;
					//usleep(sleeptime);
					usleep(2000);
					my->setPosition(pos.x()+addx, HEIGHT[j], pos.z()+addz);
					my->setJointAngle("RARM_JOINT1", DEG2RAD(LARM_JOINT1[j]));
					my->setJointAngle("RARM_JOINT3", DEG2RAD(-LARM_JOINT3[j]));
					my->setJointAngle("LARM_JOINT1", DEG2RAD(RARM_JOINT1[j]));
					my->setJointAngle("LARM_JOINT3", DEG2RAD(-RARM_JOINT3[j]));
					my->setJointAngle("RLEG_JOINT2", DEG2RAD(LLEG_JOINT2[j]));
					my->setJointAngle("RLEG_JOINT4", DEG2RAD(LLEG_JOINT4[j]));
					my->setJointAngle("RLEG_JOINT6", DEG2RAD(LLEG_JOINT6[j]));
					my->setJointAngle("LLEG_JOINT2", DEG2RAD(RLEG_JOINT2[j]));
					my->setJointAngle("LLEG_JOINT4", DEG2RAD(RLEG_JOINT4[j]));
					my->setJointAngle("LLEG_JOINT6", DEG2RAD(RLEG_JOINT6[j]));
				}
			}
			count++;
      // std::cout << "The Count is  " << count <<std::endl;
			// 目標点に到着
			if(step == 1){
				count = 0;
				step = 0;
				move = false;
				stop = true;
				std::cout << "I stop "  <<std::endl;
			//	walking = false;
				i++;
			}
		}





  return 0.1;
}

void UserController::onRecvMsg(RecvMsgEvent &evt)
{

  	double _posx;
	double _posy;
	double _posz;
  std::string sender = evt.getSender();

  //自分自身の取得
  SimObj *my = getObj(myname());

  //メッセージ取得
  char *all_msg = (char*)evt.getMsg();

  std::string ss = all_msg;
  //ヘッダーの取り出し
  int strPos1 = 0;
  int strPos2;
  std::string headss;
  std::string endss;
  std::string tmpss;
  strPos2 = ss.find(" ", strPos1);
  headss.assign(ss, strPos1, strPos2-strPos1);
  endss.assign(ss, strPos2+1, ss.size());
  std::cout << "The Message is " << ss <<std::endl;
  //std::cout<<ss<<std::endl;

  if(headss == "ORS_DATA"){
    //HMDデータによる頭部の動き反映
    moveHeadByHMD(ss);
  }
else if (headss == "XBox")
{
/// Xbox controller motion
/*	if(endss == "Move")
	{
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();
     move = true;
     m_posz = m_posz - 20;
  //   my->setPosition(m_posx, m_posy,m_posz);
	}
  if(endss == "Back")
	{
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();
     move = true;
     m_posz = m_posz + 20;
   //  my->setPosition(m_posx, m_posy,m_posz);
	}
	if(endss == "Right")
	{
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();
     move = true;
     m_posx = m_posx + 20;
    // my->setPosition(m_posx, m_posy,m_posz);
	}
  if(endss == "Left")
	{
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();
     move = true;
     m_posx = m_posx - 20;



   //  my->setPosition(m_posx, m_posy,m_posz);
	}*/
	if(endss == "Left_Rotation")
	{
	Rotation ownRotation;
	my->getRotation(ownRotation);
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();

     double roll = getRoll(ownRotation);

   //  m_posx = m_posx - 20;
    // angle =  fmod(roll+0.1,3.15);
     angle = roll + 0.1;
     if(angle > 3.15)
     {
     	angle = angle - 6.28;
     }

    // my->setAxisAndAngle(0,1.0, 0, angle);
    // move = true;
      rotate = true;
 //      std::cout << "the roll is  ...   "<< roll  << "The angle  ...   " << angle  << std::endl;
   //  my->setPosition(m_posx, m_posy,m_posz);
	}
if(endss == "Right_Rotation")
	{
	Rotation ownRotation;
	my->getRotation(ownRotation);
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();

     double roll = getRoll(ownRotation);

   //  m_posx = m_posx - 20;
    // angle =  fmod(roll-0.1,3.15);
     angle = roll - 0.1;
     if(angle < -3.15)
     {
     	angle = angle + 6.28;
     }
   //  my->setAxisAndAngle(0,1.0, 0, angle);
    // move = true;
     rotate = true;
//       std::cout << "the roll is  ...   "<< roll  << "The angle  ...   " << angle  << std::endl;
   //  my->setPosition(m_posx, m_posy,m_posz);
	}

 if(endss == "Back")
	{
	Rotation ownRotation;
	my->getRotation(ownRotation);
	 double roll = getRoll(ownRotation);
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();
	//	roll = -roll;

     m_posx = m_posx - 3*sin(roll);
     m_posz = m_posz - 3*cos(roll);


   //  m_posx = m_posx - 20;
   //  angle =  fmod(roll-0.1,3.14);
   //  my->setAxisAndAngle(0,1.0, 0, angle);
     move = false;
     my->setPosition(m_posx, m_posy,m_posz);
	}

if(endss == "Front")
	{
	Rotation ownRotation;
	my->getRotation(ownRotation);
	 double roll = getRoll(ownRotation);
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();
	//	roll = -roll;

     m_posx = m_posx + 20*sin(roll);
     m_posz = m_posz + 20*cos(roll);


   //  m_posx = m_posx - 20;
   //  angle =  fmod(roll-0.1,3.14);
   //  my->setAxisAndAngle(0,1.0, 0, angle);
    move = true;
 //      std::cout << "the roll is  ...   "<< roll  << "The angle  ...   " << angle  << "The X   " << m_posz << "The Z   " << m_posx <<std::endl;
   //  my->setPosition(m_posx, m_posy,m_posz);
	}



}


  else if(ss == "go") {
    sendMsg("robot_000","go");
    LOG_MSG(("Starting the clean up task"));
    std::cout<<"go"<<std::endl;
  }

  else if(ss == "take" ) {
    sendMsg("robot_000","take");
    LOG_MSG(("Taking the trash"));
    std::cout<<"take"<<std::endl;
  }

  else if(ss == "put" ) {
    sendMsg("robot_000","put");
    LOG_MSG(("Putting the trash in the trash box"));
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





extern "C" Controller * createController ()
{
  return new UserController;
}
