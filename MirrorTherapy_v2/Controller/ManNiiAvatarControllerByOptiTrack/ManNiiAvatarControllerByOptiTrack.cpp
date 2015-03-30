/*
 * ManNiiAvatarController.cpp
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#include "ManNiiAvatarControllerByOptiTrack.h"
#include "../../Common/OculusRiftDK1SensorData.h"

///@brief Initialize this controller.
void ManNiiAvatarControllerByOptiTrack::onInit(InitEvent &evt)
{
	this->optiTrackService = NULL;

	//this->defaultHeadJoint0Quaternion[0] = 1.0;
	//this->defaultHeadJoint0Quaternion[1] = 0.0;
	//this->defaultHeadJoint0Quaternion[2] = 0.0;
	//this->defaultHeadJoint0Quaternion[3] = 0.0;

	//this->prevYaw = this->prevPitch = this->prevRoll = 0.0;

	SimObj *my = getObj(myname());
}


///@brief Movement of the robot.
double ManNiiAvatarControllerByOptiTrack::onAction(ActionEvent &evt)
{
	this->optiTrackServiceName = optiTrackServiceNameDefault;
	bool optiTrackAvailable = checkService(this->optiTrackServiceName);
	if (optiTrackAvailable && this->optiTrackService == NULL) {
		this->optiTrackService = connectToService(this->optiTrackServiceName);
	}
	else if (!optiTrackAvailable && this->optiTrackService != NULL) {
		this->optiTrackService = NULL;
	}

	return 1.0;
}

void ManNiiAvatarControllerByOptiTrack::onRecvMsg(RecvMsgEvent &evt)
{
	const std::string allMsg = evt.getMsg();

	//std::cout << allMsg << std::endl;

	std::map<std::string, std::vector<std::string> > messageMap = SensorData::convertMessage2Map(allMsg);

	std::vector<std::string> tmpValuesStringVector = messageMap["QUATERNION"];

	const double w = atof(tmpValuesStringVector[0].c_str());
	const double x = atof(tmpValuesStringVector[1].c_str());
	const double y = atof(tmpValuesStringVector[2].c_str());
	const double z = atof(tmpValuesStringVector[3].c_str());

	SimObj *obj = getObj(myname());
	obj->setJointQuaternion("LARM_JOINT4", w, x, y, z);

	//OculusRiftDK1SensorData sensorData;
	//sensorData.decodeSensorData(allMsg);

	//EulerAngleType eulerAngle;
	//eulerAngle.yaw = sensorData.yaw();
	//eulerAngle.pitch = sensorData.pitch();
	//eulerAngle.roll = sensorData.roll();

	//ManNiiPosture manNiiPosture = ManNiiPosture();
	//this->convertEulerAngle2ManNiiPosture(eulerAngle, manNiiPosture);

	//SimObj *obj = getObj(myname());
	//this->setJointQuaternionsForOculus(obj, manNiiPosture);
}

void ManNiiAvatarControllerByOptiTrack::setJointQuaternion(SimObj *obj, const ManNiiJointQuaternion &jq)
{
	obj->setJointQuaternion(manNiiJointTypeStr(jq.manNiiJointType).c_str(), jq.quaternion.w, jq.quaternion.x, jq.quaternion.y, jq.quaternion.z);
	//std::cout << manNiiJointTypeStr(jq.manNiiJointType).c_str() << jq.quaternion.w << jq.quaternion.x << jq.quaternion.y << jq.quaternion.z << std::endl;
}


void ManNiiAvatarControllerByOptiTrack::setJointQuaternionForOptiTrack(SimObj *obj, ManNiiPosture &ManNiiAvatarPosture)
{

}
//void ManNiiAvatarControllerByOptiTrack::setJointQuaternionsForOculus(SimObj *obj, ManNiiPosture &manNiiAvatarPosture)
//{
	//this->setJointQuaternion(obj, manNiiAvatarPosture.jointQuaternions[HEAD_JOINT0]);
//}

//void ManNiiAvatarControllerByOptiTrack::convertEulerAngle2ManNiiPosture(const EulerAngleType &eulerAngle, ManNiiPosture &manNiiAvatarPosture)
//{
	//dQuaternion qyaw;
	//dQuaternion qpitch;
	//dQuaternion qroll;

	//qyaw[0] = cos(-eulerAngle.yaw/2.0);
	//qyaw[1] = 0.0;
	//qyaw[2] = sin(-eulerAngle.yaw/2.0);
	//qyaw[3] = 0.0;

	//qpitch[0] = cos(-eulerAngle.pitch/2.0);
	//qpitch[1] = sin(-eulerAngle.pitch/2.0);
	//qpitch[2] = 0.0;
	//qpitch[3] = 0.0;

	//qroll[0] = cos(-eulerAngle.roll/2.0);
	//qroll[1] = 0.0;
	//qroll[2] = 0.0;
	//qroll[3] = sin(-eulerAngle.roll/2.0);
	//dQuaternion tmpQ1;
	//dQuaternion tmpQ2;

	//dQMultiply0(tmpQ1, qyaw, qpitch);

	//dQMultiply0(tmpQ2, tmpQ1, qroll);


	//dQuaternion tmpQ3;

	////dQuaternion defaultQ = this->defaultHeadJointQuaternion;
	//dQMultiply1(tmpQ3, this->defaultHeadJoint0Quaternion, tmpQ2);

	//Quaternion tmpQ4(tmpQ3[0], tmpQ3[1], tmpQ3[2], tmpQ3[3]);

	//manNiiAvatarPosture.jointQuaternions[HEAD_JOINT0].manNiiJointType = HEAD_JOINT0;
	//manNiiAvatarPosture.jointQuaternions[HEAD_JOINT0].quaternion = tmpQ4;

//}
