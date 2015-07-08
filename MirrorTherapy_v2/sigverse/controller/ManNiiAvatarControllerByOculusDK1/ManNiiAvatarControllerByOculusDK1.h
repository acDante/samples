/*
 * ManNiiAvatarControllerByOculusDK1.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */
#ifndef __MAN_NII_AVATAR_CONTROLLER_BY_OCULUS_DK1_H__
#define __MAN_NII_AVATAR_CONTROLLER_BY_OCULUS_DK1_H__

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>

#include "../../common/device/OculusRiftDK1SensorData.h"

#include "../ControllerCommon/ManNiiAvatarController.h"
#include "../ControllerCommon/ManNiiPosture.h"


class ManNiiAvatarControllerByOculusDK1 : public ManNiiAvatarController
{
protected:
	///@brief Parameter file name.
	static const std::string parameterFileName;

	BaseService *oculusDK1Service;

	std::string oculusDK1ServiceName;
	std::string oculusDK1DeviceType;
	std::string oculusDK1DeviceUniqueID;

	dQuaternion defaultHeadJoint0Quaternion;

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
	void convertEulerAngle2ManNiiPosture(const SigCmn::EulerAngleType &eulerAngle);

	void setJointQuaternions(SimObj *obj);

	void setJointQuaternionsForOculus(SimObj *obj);
};

extern "C" Controller * createController()
{
	return new ManNiiAvatarControllerByOculusDK1;
}

#endif //__MAN_NII_AVATAR_CONTROLLER_BY_OCULUS_DK1_H__
