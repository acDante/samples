/*
 * ManNiiAvatarController.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */
#ifndef __MAN_NII_AVATAR_CONTROLLER_BY_OCULUS_H__
#define __MAN_NII_AVATAR_CONTROLLER_BY_OCULUS_H__

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>

#include "../../Common/OculusRiftDK1SensorData.h"

#include "../ControllerCommon/AvatarController.h"
#include "../ControllerCommon/ManNiiPosture.h"


#define PARAM_FILE_NAME_OCULUS_RIFT_DK1_INI  "OculusRiftDK1.ini"

class ManNiiAvatarControllerByOculus : public AvatarController
{
private:
	///@brief Parameter file name.
	std::string parameterFileName;

	BaseService *oculusDK1Service;

	std::string oculusDK1ServiceName;
	std::string oculusDK1DeviceType;
	std::string oculusDK1DeviceUniqueID;

	dQuaternion defaultHeadJoint0Quaternion;

//	double prevYaw, prevPitch, prevRoll;

	ManNiiPosture posture;

	void readIniFile();

public:
	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	///@brief Convert euler angle to avatar posture structure.
	void convertEulerAngle2ManNiiPosture(const SensorData::EulerAngleType &eulerAngle);

	void setJointQuaternions(SimObj *obj);

	void setJointQuaternionsForOculus(SimObj *obj);
};

extern "C" Controller * createController()
{
	return new ManNiiAvatarControllerByOculus;
}

#endif //__MAN_NII_AVATAR_CONTROLLER_BY_OCULUS_H__
