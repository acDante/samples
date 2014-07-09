#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"

class MyController : public Controller {
public:
  void onInit(InitEvent &evt);
  double onAction(ActionEvent&);
  void onRecvMsg(RecvMsgEvent &evt);

private:


struct avatar
{
std::string name;
std::string ID;
};

/*
struct avatar2
{
std::string name;
std::string ID;
};

struct avatar3
{
std::string name;
std::string ID;
};
*/
std::string current_avatar;
std::string id;
int m_state;
std::vector <avatar>  list;
int ind;
std::string avatars_number;
BaseService *m_ref;


std::vector<std::string> names;

};


void MyController::onInit(InitEvent &evt)
{
  avatars_number = "2";
m_state = 20;
ind =20;
getAllEntities(names);


  int sstrPos1 = 0;
  int sstrPos2 = 0;
  int sstrPos3 = 0;
  std::string sheadss = "";
  std::string scmd = "";
  std::string stmpss = "";

for(int j = 0; j< names.size();j++)
  {

std::cout<<"the name is :"<< names[j] <<std::endl;
  sstrPos1 = 0;
  sstrPos2 = 0;
  sstrPos3 = 0;
  sheadss = "";
  scmd = "";
  stmpss = "";

  sstrPos2 = names[j].find("_", sstrPos1);
  sheadss.assign(names[j], sstrPos1, sstrPos2-sstrPos1);
avatar avatar1;

if(sheadss == "man")
{
avatar1.name = names[j];
avatar1.ID = "";
list.push_back(avatar1);


}

}

for(int j = 0; j< list.size();j++)
  {

std::cout<<"the list content:"<< list[j].name <<std::endl;

  }


/*
avatar avatar4;
avatar4.name ="man_000";
avatar4.ID = "";
list.push_back(avatar4);

avatar avatar1;
avatar1.name = "man_001";
avatar1.ID = "";
list.push_back(avatar1);
*/

m_ref = NULL;
}

double MyController::onAction(ActionEvent &evt)
{
 switch(m_state){

  // 初期姿勢を設定 seting initial pose
  case 0: {
 // sendMsg(current_avatar,id);
break;
}

}
return 1;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{

  std::string sender = evt.getSender();

  //自分自身の取得
  SimObj *my = getObj(myname());

  //メッセージ取得
  char *all_msg = (char*)evt.getMsg();

  std::string ss = all_msg;
 int strPos1 = 0;
  int strPos2,strPos3;
  std::string headss,cmd;
  std::string tmpss;
  std::cout<<"the massage is :"<< ss <<std::endl;
  std::cout<<"the sender is :"<< sender <<std::endl;
  strPos2 = ss.find(" ", strPos1);
  headss.assign(ss, strPos1, strPos2-strPos1);
  int jj = ss.size();
  cmd.assign(ss, strPos2+1,jj);
  strPos3 = cmd.find("|", strPos1);
  id.assign(cmd, strPos1, strPos3-strPos1+1);
  //std::cout<<"The ID is :  "<< id <<std::endl;
  bool aff = true;
  bool comp = false;

if (headss == "MultiUsersMenu")
{
  bool available = checkService(ss);
  if(!available && m_ref != NULL) m_ref = NULL;
  else if(available && m_ref == NULL){
    m_ref = connectToService(ss);
}
 int nn = 0;
for(int j = 0; j< list.size();j++)
  {

  if (id != list[j].ID)
  {
  nn++;
  }
   else
  {
   sendMsg(list[j].name,list[j].ID);
   m_state = 0;
   ind = j;
 current_avatar = list[j].name;
  }

  }

if (nn == list.size())
  {
for(int i = 0; i< list.size();i++)
{


if(list[i].ID == "" && aff == true )
{

list[i].ID = id;
sendMsg(list[i].name,list[i].ID);
   m_state = 0;
   ind = i;
   current_avatar = list[i].name;
aff = false;

                }

          }

    }

}


if (ss == "Info")

{
std::cout<<"Sending info to:"<< sender <<std::endl;

std::string mess = "";
mess +=  "AvatarInformation/" ;
mess += avatars_number;
mess +=  "/" ;
for(int j = 0; j< list.size();j++)
  {

   mess += list[j].name ;
   mess +=  "/" ;
}
std::cout<<"The message is :"<< mess <<std::endl;
sendMsg(sender,mess);

     }
}


extern "C" Controller * createController ()
{
  return new MyController;
}
