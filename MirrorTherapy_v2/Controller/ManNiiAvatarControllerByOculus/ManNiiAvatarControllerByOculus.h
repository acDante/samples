/*
 * ManNiiAvatarController.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>

#include "../ControllerCommon/AvatarController.h"
#include "../ControllerCommon/ManNiiPosture.h"

#define OCULUS_SERVICE_NAME "SIGOCULUS"
static const std::string oculusDK1ServiceNameDefault = OCULUS_SERVICE_NAME;

// Define DEV_TYPE.
static const std::string devTypeOculus = "OCULUSDK1";

class ManNiiAvatarControllerByOculus : public AvatarController
{
private:
	//ManNiiAvatarPosture posture;
	BaseService *kinectV2Service;
	std::string kinectV2ServiceName;
	BaseService *oculusDK1Service;
	std::string oculusDK1ServiceName;

	dQuaternion defaultHeadJoint0Quaternion;

	double prevYaw, prevPitch, prevRoll;

public:

	//ManNiiAvatarPosture posture;

	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	///@brief Convert euler angle to avatar posture structure.
	void convertEulerAngle2ManNiiPosture(const EulerAngleType &eulerAngle, ManNiiPosture &manNiiAvatarPosture);

	void setJointQuaternion(SimObj *obj, const ManNiiJointQuaternion &jq);

	void setJointQuaternions(SimObj *obj);

	void setJointQuaternionsForOculus(SimObj *obj, ManNiiPosture &manNiiAvatarPosture);
};

extern "C" Controller * createController()
{
	return new ManNiiAvatarControllerByOculus;
}
