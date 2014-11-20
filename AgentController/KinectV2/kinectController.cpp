#include <string>
#include <math.h>
#include "Controller.h"
#include "Logger.h"
#include "ControllerEvent.h"
 
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )
 
using namespace std;
 
class AgentController : public Controller
{
public:
double onAction(ActionEvent &evt);
void onRecvMsg(RecvMsgEvent &evt);
void onInit(InitEvent &evt);
 
private:
//初期位置
double m_posx, m_posy, m_posz;
double m_yrot;
double m_range;
//データ数（関節数）最大値
int m_maxsize;
 
BaseService *m_kinect;
};
 
void AgentController::onInit(InitEvent &evt)
{
// 初期化
m_kinect = NULL;
SimObj *my = getObj(myname());
 
// 初期位置取得
Vector3d pos;
my->getPosition(pos);
m_posx = pos.x();
m_posy = pos.y();
m_posz = pos.z();
 
// 初期姿勢（回転）取得
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
// サービスが使用可能か定期的にチェックする
bool available = checkService("SIGKINECT");
 
// 使用可能
if (available && m_kinect == NULL) {
// サービスに接続
m_kinect = connectToService("SIGKINECT");
}
// 使用不可能
else if (!available && m_kinect != NULL) {
m_kinect = NULL;
}
return 1.0;
}
 
void AgentController::onRecvMsg(RecvMsgEvent &evt)
{
std::string sender = evt.getSender();
 
//自分自身の取得
SimObj *my = getObj(myname());
 
//メッセージ取得
char *all_msg = (char*)evt.getMsg();
 printf("all_msg=\n%s\n",all_msg);

char *msg = strtok(all_msg," ");
if (strcmp(msg,"KINECT_DATA") == 0) {
int i = 0;
while (true) {
i++;
if (i == m_maxsize+1)
break;
char *type = strtok(NULL," ");
printf("type=%s\n",type);
//体の位置
if (strcmp(type,"POSITION") == 0) {
double x = atof(strtok(NULL,","))*10;
double y = atof(strtok(NULL,","))*10;
double z = atof(strtok(NULL," "))*10;
//キネクト座標からSIGVerse座標への変換
double gx = cos(m_yrot)*x - sin(m_yrot)*z;
double gz = sin(m_yrot)*x + cos(m_yrot)*z;
printf("gx=%f, gz=%f\n",gx,gz);
my->setPosition(m_posx+gx,m_posy+y,m_posz+gz);
continue;
}
//体全体の回転
else if (strcmp(type,"WAIST") == 0) {
double w = atof(strtok(NULL,","));
double x = atof(strtok(NULL,","));
double y = atof(strtok(NULL,","));
double z = atof(strtok(NULL," "));
my->setJointQuaternion("ROOT_JOINT0",w,x,y,z);
continue;
}
else if (strcmp(type,"END:") == 0){
	printf("end of recvmsg\n");
	break;
//関節の回転
}else {
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
