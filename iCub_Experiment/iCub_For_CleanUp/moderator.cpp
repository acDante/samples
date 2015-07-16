#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h" 
#include <unistd.h>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <time.h> // for clock
  
#define eps 0


int sgn(double d){ 
    return d<-eps?-1:d>eps;
}


double MAG(Vector3d kvec)
{
double sum = sqrt(kvec.x()*kvec.x()+kvec.y()*kvec.y()+kvec.z()*kvec.z());
 return  sum; 
}

Vector3d MINV(Vector3d kvec, Vector3d svec)          /* CROSS Vector Product */              
{  

Vector3d vec_res ;
   vec_res.x(kvec.x()-svec.x()) ;
   vec_res.y(kvec.y()-svec.y()) ;
   vec_res.z(kvec.z()-svec.z()) ;
  return  vec_res;
}


Vector3d PLSV(Vector3d kvec, Vector3d svec)          /* CROSS Vector Product */              
{  
Vector3d vec_res ;
   vec_res.x(kvec.x()+svec.x()) ;
   vec_res.y(kvec.y()+svec.y()) ;
   vec_res.z(kvec.z()+svec.z()) ;
  return  vec_res;
}

Vector3d MULT(Vector3d kvec, double v)          /* CROSS Vector Product */              
{  
Vector3d vec_res ;
   vec_res.x(kvec.x()*v) ;
   vec_res.y(kvec.y()*v) ;
   vec_res.z(kvec.z()*v) ;
  return  vec_res;
}


double DOT(Vector3d kvec, Vector3d svec)            /* DOT Vector Product */                
{   
double s;                                                                    
    s = kvec.x()*svec.x() + kvec.y()*svec.y() + kvec.z()*svec.z();  

    return s;           
}

double NORM(Vector3d kvec)          /* Normalize*/              
{
  double s;                                                                    
    s = sqrt(kvec.x()*kvec.x()+kvec.y()*kvec.y()+kvec.z()*kvec.z()); 

    return s;           
}


Vector3d CROSS(Vector3d kvec, Vector3d svec)          /* CROSS Vector Product */              
{            
    Vector3d vec_res ;                                                       
    vec_res.x(kvec.y()*svec.z() - kvec.z()*svec.y()) ;                             
    vec_res.y(kvec.z()*svec.x() - kvec.x()*svec.z()) ;                             
    vec_res.z(kvec.x()*svec.y() - kvec.y()*svec.x()) ; 

    return  vec_res;                           
}

Vector3d NORMAL(Vector3d kvec)          /* Normalize*/              
{
     Vector3d vec_res ;
   double sum = sqrt(kvec.x()*kvec.x()+kvec.y()*kvec.y()+kvec.z()*kvec.z());
   vec_res.x(kvec.x()/sum) ;
   vec_res.y(kvec.y()/sum) ;
   vec_res.z(kvec.z()/sum) ;
  return  vec_res; 
}

class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);  
  void onRecvMsg(RecvMsgEvent &evt); 
  bool Image_Range();
  void Record_Position();
  bool Image_Range_val(Vector3d vec);
private:
  SimObj *m_my;
  std::vector<std::string> m_entities;
  
  // ゴミ箱のサイズ(この範囲でreleaseしなければゴミを捨てられない)
  double tboxSize_x, tboxSize_z;

  // ゴミが入ったとされる高さ方向の範囲(y方向)
  double tboxMin_y, tboxMax_y;

  // 審判サービス
  BaseService *m_ref;
  
  bool   colState;      // 衝突中かどうか
  double retValue;
  std::string roboName;
CParts * parts2;
CParts * parts1;
CParts * parts3;
std::ofstream Position_Entities;

 double take_time;
 bool init_time ;
double elapsedTime;
Vector3d Coord_Pos[7];
bool States_image[5];
bool trigger;
};  
  
void MyController::onInit(InitEvent &evt) {  
  m_my = getObj(myname());
  getAllEntities(m_entities);
  m_ref = NULL;
  retValue = 0.5;
  roboName = "robot_000";

  colState = false;

  // ゴミ箱
  tboxSize_x  = 400.0;
  tboxSize_z  = 90.5; 
  tboxMin_y    = 30.0;
  tboxMax_y    = 1000.0;
  trigger = false;


Position_Entities.open("SIGVerse_Record.csv");
Position_Entities << "timeStamp robotPosx robotPosz handx,handy handz canx cany canz cokex cokey cokez trash_box_redx trash_box_redy trash_box_redz trash_box_greenx trash_box_greeny trash_box_greenz trash_box_bluex trash_box_bluey trash_box_bluez Can_S Cok_S trash_box_red_S trash_box_green_S trash_box_blue_S \n";
init_time = true;
elapsedTime = 0;

}  
  
double MyController::onAction(ActionEvent &evt) 
{ 
  // サービスが使用可能か定期的にチェックする  
elapsedTime = evt.time() -  take_time;
Record_Position();
  // 自分の位置取得
  Vector3d myPos;
  m_my->getPosition(myPos);

  // 衝突中の場合,衝突が継続しているかチェック
  if(colState){
    CParts *parts = m_my->getMainParts();
    bool state = parts->getCollisionState();
    
    // 衝突していない状態に戻す
    if(!state) colState = false;
  }
  
  int entSize = m_entities.size();
  for(int i = 0; i < entSize; i++){

    // ロボットまたはゴミ箱の場合は除く
    if(m_entities[i] == "robot_000"  ||
       m_entities[i] == "recycle" ||
       m_entities[i] == "burnable" ||
       m_entities[i] == "unburnable"){
      continue;
    }
    // エンティティ取得
    SimObj *ent = getObj(m_entities[i].c_str());

    // 位置取得
    Vector3d tpos;
    ent->getPosition(tpos);

    // ゴミ箱からゴミを結ぶベクトル
    Vector3d vec(tpos.x()-myPos.x(), tpos.y()-myPos.y(), tpos.z()-myPos.z());
    
    // ゴミがゴミ箱の中に入ったかどうか判定
    if(abs(vec.x()) < tboxSize_x/2.0 &&
       abs(vec.z()) < tboxSize_z/2.0 &&
       tpos.y() < tboxMax_y     &&
       tpos.y() > tboxMin_y     ){

      // ゴミがリリースされているか確認
      if(!ent->getIsGrasped()){

	// ゴミを捨てる
	tpos.y(tpos.y() /2);
	tpos.x(myPos.x());
	tpos.z(myPos.z());
	ent->setAxisAndAngle(1.0, 0.0, 0.0, 0.0);
	ent->setAxisAndAngle(1.0, 0.0, 0.0, 0.0);
	ent->setPosition(tpos);
	ent->setPosition(tpos);
	ent->setPosition(tpos);
	//usleep(500000);
	tpos.y(0.0);
	ent->setPosition(tpos);
	ent->setPosition(tpos);
	ent->setPosition(tpos);

	std::string msg;
	// ゴミが所定のゴミ箱に捨てられているかチェック
	// リサイクル
	

	//LOG_MSG((msg.c_str()));
      }
    }
  }

  return 0;      
}  
  



void MyController::Record_Position()
{
Vector3d Can_pos;
Vector3d Pet_pos;
Vector3d Mug_pos;
Vector3d Robot_pos;
Vector3d Red_pos;
Vector3d Blue_pos;
Vector3d Green_pos;
Vector3d Hand_pos;
bool States[5]; // Can Pet Red Green Blue

//std::cout <<  " Time  " << elapsedTime << std::endl;
//Entitie_Pos = getObj(myname());
/*
int entSize = m_entities.size();
  for(int i = 0; i < entSize; i++){

    // ロボットまたはゴミ箱の場合は除く
    if(m_entities[i] == "table_0"  ||
       m_entities[i] == "man_000" ||
       m_entities[i] == "moderator_0" ||
       m_entities[i] == "room"){
      continue;
    }
    if(m_entities[i] == "robot_000")
    {
    SimObj *ent = getObj(m_entities[i].c_str());
    ent->getPosition(Robot_pos);
    CParts * parts = ent->getParts("GRASP_LINK");
    parts->getPosition(Hand_pos);
    
   // Coord_Pos[7]
    }
    if(m_entities[i] == "can")
    {
    SimObj *ent = getObj(m_entities[i].c_str());
    ent->getPosition(Can_pos);
  //  States[0] = Image_Range(Can_pos);
    }
    if(m_entities[i] == "petbottle")
    {
    SimObj *ent = getObj(m_entities[i].c_str());
    ent->getPosition(Pet_pos);
 //    States[1] = Image_Range(Pet_pos);
    }
    if(m_entities[i] == "recycle")
    {
    SimObj *ent = getObj(m_entities[i].c_str());
    ent->getPosition(Green_pos);
 //   States[3] = Image_Range(Green_pos);
    }

    if(m_entities[i] == "unburnable")
    {
    SimObj *ent = getObj(m_entities[i].c_str());
    ent->getPosition(Blue_pos);
  //   States[4] = Image_Range(Blue_pos);
    }

    if(m_entities[i] == "burnable")
    {
    SimObj *ent = getObj(m_entities[i].c_str());
    ent->getPosition(Red_pos);
  //  States[2] = Image_Range(Red_pos);
    }
    // エンティティ取得
    // SimObj *ent = getObj(m_entities[i].c_str());

    // 位置取得
    // Vector3d tpos;
    // ent->getPosition(tpos);

//std::cout<< m_entities[i] <<std::endl;
  }

*/


    SimObj *ent = getObj("robot_000");
    ent->getPosition(Robot_pos);
    CParts * parts = ent->getParts("GRASP_LINK");
    parts->getPosition(Hand_pos);
    parts2 = ent->getParts("NECK_LINK0");
    parts1 = ent->getParts("DIR_LINK0"); // On Z
    parts3 = ent->getParts("DIR_LINK1"); // On X
   // Coord_Pos[7]


    ent = getObj("can");
    ent->getPosition(Can_pos);
  //  States[0] = Image_Range(Can_pos);


    ent = getObj("petbottle");
    ent->getPosition(Pet_pos);
 //    States[1] = Image_Range(Pet_pos);


    ent = getObj("recycle");
    ent->getPosition(Green_pos);
 //   States[3] = Image_Range(Green_pos);



    ent = getObj("unburnable");
    ent->getPosition(Blue_pos);
  //   States[4] = Image_Range(Blue_pos);



    ent = getObj("burnable");
    ent->getPosition(Red_pos);
  //  States[2] = Image_Range(Red_pos);


/*
    States_image[0] = Image_Range_val(Can_pos);
    States_image[1] = Image_Range_val(Pet_pos);
    States_image[2] = Image_Range_val(Red_pos);
    States_image[3] = Image_Range_val(Green_pos);
    States_image[4] = Image_Range_val(Blue_pos);
*/

Coord_Pos[0]= Robot_pos; 

Coord_Pos[1]= Hand_pos;

Coord_Pos[2]= Can_pos;

Coord_Pos[3]= Pet_pos;

Coord_Pos[4]= Red_pos;

Coord_Pos[5]= Green_pos;

Coord_Pos[6]= Blue_pos;

/*
Data_vec.push_back (Robot_pos);
Data_vec.push_back (Hand_pos);
Data_vec.push_back (Can_pos);
Data_vec.push_back (Pet_pos);
Data_vec.push_back (Red_pos);
Data_vec.push_back (Green_pos);
Data_vec.push_back (Blue_pos);
Data_record.push_back (Data_vec);
Data_vec.clear();
*/

if(trigger == true)
{
Image_Range();
}

//Image_Range();

Position_Entities << elapsedTime << " " << Robot_pos.x()<< " " << Robot_pos.y()<< " " << Robot_pos.z()<< " " << Hand_pos.x()<< " " << Hand_pos.y()<< " " << Hand_pos.z()<< " " << Can_pos.x()<< " " << Can_pos.y()<< " " << Can_pos.z()<< " " << Pet_pos.x()<< " " << Pet_pos.y()<< " " << Pet_pos.z()<< " " << Red_pos.x()<< " " << Red_pos.y()<< " " << Red_pos.z()<< " " << Green_pos.x()<< " " << Green_pos.y()<< " " << Green_pos.z()<< " " << Blue_pos.x()<< " " << Blue_pos.y()<< " " << Blue_pos.z()<< " " << States_image[0] << " " << States_image[1] << " " << States_image[2] << " " << States_image[3] << " " << States_image[4] <<  std::endl ;
std::cout <<  " object status  "  << States_image[0] << " " << States_image[1] << " " << States_image[2] << " " << States_image[3] << " " << States_image[4] <<  std::endl ;
}


bool MyController::Image_Range()
{

Vector3d ZVec_pos;
Vector3d NeckVec_pos;
Vector3d XVec_pos;

//Vector3d ;
//bool States_image[5];


for(int i = 2; i < 7;i++ )
{
   



    parts2->getPosition(NeckVec_pos);
    parts1->getPosition(ZVec_pos);
    parts3->getPosition(XVec_pos);

     Vector3d CamDirZ_pos;
     Vector3d CamDirX_pos;
    CamDirZ_pos =  MINV(NeckVec_pos,ZVec_pos) ;
    Vector3d vecN = MINV(Coord_Pos[i],NeckVec_pos);

 double sum =  MAG(CROSS(CamDirZ_pos,vecN));
 double c = DOT(CamDirZ_pos,vecN);
 double ang = atan2(sum, c);

if(ang > -0.68  && ang < 0.68)
{
CamDirZ_pos = NORMAL(CamDirZ_pos);
vecN = NORMAL(vecN);
    CamDirX_pos =  NORMAL(MINV(NeckVec_pos,XVec_pos))  ;

    Vector3d CamDirY_pos;
    CamDirY_pos = CROSS(CamDirZ_pos,CamDirX_pos);
   // ent->getCamera1ViewPoint(CamPoint_pos);
   // ent->getCamera1ViewVector(CanVec_pos);
    //vec -= NeckVec_pos;

//Vector3d nnX = NORMAL(CROSS(CamDirX_pos,CROSS(vecN,CamDirX_pos)));
//Vector3d nnY = NORMAL(CROSS(CamDirY_pos,CROSS(vecN,CamDirY_pos)));

Vector3d nnX = CROSS(CamDirX_pos,CROSS(vecN,CamDirX_pos));
Vector3d nnY = CROSS(CamDirY_pos,CROSS(vecN,CamDirY_pos));    

//angle = acos(dotProduct(Va.normalize(), Vb.normalize()));
//cross = crossProduct(Va, Vb);
//*sgn(dotProduct(Vn,cross))
 double sumX =  MAG(CROSS(CamDirZ_pos,nnX));
 double cX = DOT(CamDirZ_pos,nnX);
 double angleX = atan2(sumX, cX);

 double sumY =  MAG(CROSS(CamDirZ_pos,nnY));
 double cY = DOT(CamDirZ_pos,nnY);
 double angleY = atan2(sumY, cY);

/*
Vector3d ppX = CROSS(CamDirZ_pos,nnX);
double angleX = acos( DOT(CamDirZ_pos,nnX))*sgn(DOT(CamDirX_pos,ppX));

Vector3d ppY = CROSS(CamDirZ_pos,nnY);
double angleY = acos( DOT(CamDirZ_pos,nnY))*sgn(DOT(CamDirY_pos,ppY));
*/



//std::cout << "nnX  : " << angleX << "nnY  : " << angleY << std::endl;
//  std::cout << "nnX   x : " << nnX.x() << "  Y : " <<   nnX.y() << "  Z : " << nnX.z() << std::endl;
//  std::cout << "nnY   x : " << nnY.x() << "  Y : " <<   nnY.y() << "  Z : " << nnY.z() << std::endl;
//std::cout << "  AngleX  : " << angleX << "   AngleY  : " << angleY << std::endl;


if(angleX > -0.4  &&   angleX  < 0.4  && angleY > -0.55  && angleY < 0.55 )
{
//return true;
  States_image[i-2] = true;
}
else
{
 States_image[i-2] = false;
//return false;

}

}

else
{

 States_image[i-2] = false;

}

}
 //  std::cout << "CamPoint x : " << CamPoint_pos.x() << "  Y : " <<   CamPoint_pos.y() << "  Z : " << CamPoint_pos.z() << std::endl;
 //  std::cout << "CanVec   x : " << CanVec_pos.x() << "  Y : " <<   CanVec_pos.y() << "  Z : " << CanVec_pos.z() << std::endl;
 //  std::cout << "CamDir   x : " << CamDir_pos.x() << "  Y : " <<   CamDir_pos.y() << "  Z : " << CamDir_pos.z() << std::endl;
 //  std::cout << "CamDir2   x : " << CamDir1_pos.x() << "  Y : " <<   CamDir1_pos.y() << "  Z : " << CamDir1_pos.z() << std::endl;
 //  std::cout << "CamDir3   x : " << CamDir2_pos.x() << "  Y : " <<   CamDir2_pos.y() << "  Z : " << CamDir2_pos.z() << std::endl; 

}

bool MyController::Image_Range_val(Vector3d vec)
{

Vector3d ZVec_pos;
Vector3d NeckVec_pos;
Vector3d XVec_pos;

//Vector3d ;
//bool States_image[5];






    parts2->getPosition(NeckVec_pos);
    parts1->getPosition(ZVec_pos);
    parts3->getPosition(XVec_pos);

     Vector3d CamDirZ_pos;
     Vector3d CamDirX_pos;
    CamDirZ_pos =  MINV(NeckVec_pos,ZVec_pos) ;
    Vector3d vecN = MINV(vec,NeckVec_pos);

 double sum =  MAG(CROSS(CamDirZ_pos,vecN));
 double c = DOT(CamDirZ_pos,vecN);
 double ang = atan2(sum, c);

if(ang > -0.89  && ang < 0.89)
{
CamDirZ_pos = NORMAL(CamDirZ_pos);
vecN = NORMAL(vecN);
    CamDirX_pos =  NORMAL(MINV(NeckVec_pos,XVec_pos))  ;

    Vector3d CamDirY_pos;
    CamDirY_pos = CROSS(CamDirZ_pos,CamDirX_pos);
   // ent->getCamera1ViewPoint(CamPoint_pos);
   // ent->getCamera1ViewVector(CanVec_pos);
    //vec -= NeckVec_pos;

//Vector3d nnX = NORMAL(CROSS(CamDirX_pos,CROSS(vecN,CamDirX_pos)));
//Vector3d nnY = NORMAL(CROSS(CamDirY_pos,CROSS(vecN,CamDirY_pos)));

Vector3d nnX = CROSS(CamDirX_pos,CROSS(vecN,CamDirX_pos));
Vector3d nnY = CROSS(CamDirY_pos,CROSS(vecN,CamDirY_pos));    

//angle = acos(dotProduct(Va.normalize(), Vb.normalize()));
//cross = crossProduct(Va, Vb);
//*sgn(dotProduct(Vn,cross))
 double sumX =  MAG(CROSS(CamDirZ_pos,nnX));
 double cX = DOT(CamDirZ_pos,nnX);
 double angleX = atan2(sumX, cX);

 double sumY =  MAG(CROSS(CamDirZ_pos,nnY));
 double cY = DOT(CamDirZ_pos,nnY);
 double angleY = atan2(sumY, cY);

/*
Vector3d ppX = CROSS(CamDirZ_pos,nnX);
double angleX = acos( DOT(CamDirZ_pos,nnX))*sgn(DOT(CamDirX_pos,ppX));

Vector3d ppY = CROSS(CamDirZ_pos,nnY);
double angleY = acos( DOT(CamDirZ_pos,nnY))*sgn(DOT(CamDirY_pos,ppY));
*/



//std::cout << "nnX  : " << angleX << "nnY  : " << angleY << std::endl;
//  std::cout << "nnX   x : " << nnX.x() << "  Y : " <<   nnX.y() << "  Z : " << nnX.z() << std::endl;
//  std::cout << "nnY   x : " << nnY.x() << "  Y : " <<   nnY.y() << "  Z : " << nnY.z() << std::endl;
//std::cout << "  AngleX  : " << angleX << "   AngleY  : " << angleY << std::endl;


if(angleX > -0.4  &&   angleX  < 0.4  && angleY > -0.9  && angleY < 0.9 )
{
//return true;
  return true;
}
else
{
 return false;
//return false;

}

}

else
{

return false;

}


 //  std::cout << "CamPoint x : " << CamPoint_pos.x() << "  Y : " <<   CamPoint_pos.y() << "  Z : " << CamPoint_pos.z() << std::endl;
 //  std::cout << "CanVec   x : " << CanVec_pos.x() << "  Y : " <<   CanVec_pos.y() << "  Z : " << CanVec_pos.z() << std::endl;
 //  std::cout << "CamDir   x : " << CamDir_pos.x() << "  Y : " <<   CamDir_pos.y() << "  Z : " << CamDir_pos.z() << std::endl;
 //  std::cout << "CamDir2   x : " << CamDir1_pos.x() << "  Y : " <<   CamDir1_pos.y() << "  Z : " << CamDir1_pos.z() << std::endl;
 //  std::cout << "CamDir3   x : " << CamDir2_pos.x() << "  Y : " <<   CamDir2_pos.y() << "  Z : " << CamDir2_pos.z() << std::endl; 

}






void MyController::onRecvMsg(RecvMsgEvent &evt) {  
std::string sender = evt.getSender();

  // 送信者がゴミ認識サービスの場合
  char *all_msg = (char*)evt.getMsg();
  std::string msg;
  msg= evt.getMsg();

 // printf("all_msg=\n%s\n",all_msg);

  std::string ss = all_msg;
  //ヘッダーの取り出し
  int strPos1 = 0;
  int strPos2;
  std::string headss;
  std::string tmpss;
  strPos2 = ss.find(" ", strPos1);
  headss.assign(ss, strPos1, strPos2-strPos1);

    if (msg == "switch_range")
  {
    if(trigger == false)
    trigger = true ;
    else
      trigger = false ;
  }


}  


  
extern "C" Controller * createController() {  
  return new MyController;  
}  

