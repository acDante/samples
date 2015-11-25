#include <Controller.h>
#include <ControllerEvent.h>
#include <Logger.h>
#include <ViewImage.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class ORSDK2Controller : public Controller
{
public:
	void onRecvMsg(RecvMsgEvent &evt);
	void onInit(InitEvent &evt);
	double onAction(ActionEvent &evt);
private:
	void moveByORSDK2(std::string ss);
	BaseService *m_orsDK2;

	int m_maxsize;

	double m_qw, m_qy, m_qx, m_qz;
	double o_qw, o_qx, o_qy, o_qz;
	double po_qw, po_qx, po_qy, po_qz;

	bool chk_orsDK2;
};

void ORSDK2Controller::onInit(InitEvent &evt)
{
	srand(time(NULL));

	SimObj *my = this->getObj(this->myname());

	m_qw = my->qw();
	m_qx = my->qx();
	m_qy = my->qy();
	m_qz = my->qz();
	o_qw = 1.0;
	o_qx = 0.0;
	o_qy = 0.0;
	o_qz = 0.0;

	m_orsDK2 = NULL;
	chk_orsDK2 = false;
}

double ORSDK2Controller::onAction(ActionEvent &evt)
{
	
	chk_orsDK2 = checkService("SIGORSDK2");

	if (chk_orsDK2) {
		if (m_orsDK2 == NULL){
			m_orsDK2 = connectToService("SIGORSDK2");
		}
	}
	else{
		m_orsDK2 = NULL;
	}
	

	return 1.0;
}

void ORSDK2Controller::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	char *all_msg = (char*)evt.getMsg();

	std::string ss = all_msg;
	int strPos1 = 0;
	int strPos2;
	std::string headss;
	std::string bodyss;
	strPos2 = ss.find(" ", strPos1);
	headss.assign(ss, strPos1, strPos2 - strPos1);
	bodyss.assign(ss, strPos2 + 1, ss.length() - strPos2);

	if(headss == "ORS_DATA") {
		moveByORSDK2(bodyss);
	}
}

void ORSDK2Controller::moveByORSDK2(std::string ss)
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

extern "C" Controller * createController ()
{
	return new ORSDK2Controller;
}
