/*
 * ManNiiAvatarController.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */
#ifndef __MAN_NII_AVATAR_CONTROLLER_BY_KINECT_V2_H__
#define __MAN_NII_AVATAR_CONTROLLER_BY_KINECT_V2_H__

#include <sigverse/Controller.h>
#include <sigverse/comm/controller/Controller.h>
#include <sigverse/ControllerEvent.h>

#include "../ControllerCommon/AvatarController.h"
#include "../ControllerCommon/ManNiiPosture.h"
#include "../../Common/KinectV2SensorData.h"


#define PARAM_FILE_NAME_KINECTV2_INI  "KinectV2.ini"


class ManNiiAvatarControllerByKinectV2 : public AvatarController
{
private:
	///@brief Parameter file name.
	std::string parameterFileName;

	//ManNiiAvatarPosture posture;
	BaseService *kinectV2Service;
	std::string kinectV2ServiceName;
	std::string kinectV2DeviceType;
	std::string kinectV2DeviceUniqueID;

	void readIniFile();

public:
	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	///@brief Convert euler angle to avatar posture structure.
	void convertKinectV2JointOrientations2ManNiiPosture(KinectV2SensorData::KinectV2JointOrientation *kinectV2Joints, ManNiiPosture &manNiiPosture);
	//void convertKinectV2JointOrientations2ManNiiPosture(std::vector<KinectV2JointOrientation> &kinectV2Joints, ManNiiPosture &manNiiPosture);

	void setJointQuaternion(SimObj *obj, ManNiiPosture::ManNiiJoint &jq);

	void setJointQuaternionsForKinect(SimObj *obj, ManNiiPosture &manNiiPosture);
};

extern "C" Controller * createController()
{
	return new ManNiiAvatarControllerByKinectV2;
}

#endif //__MAN_NII_AVATAR_CONTROLLER_BY_KINECT_V2_H__
