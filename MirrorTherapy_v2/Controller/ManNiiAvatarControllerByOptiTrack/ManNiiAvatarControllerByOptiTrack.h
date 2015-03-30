/*
 * ManNiiAvatarControllerByOptiTrack.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>

#include "../ControllerCommon/AvatarController.h"
#include "../ControllerCommon/ManNiiPosture.h"

#define OPTITRACK_SERVICE_NAME "SIGMOCAP"
static const std::string optiTrackServiceNameDefault = OPTITRACK_SERVICE_NAME;

// Define DEV_TYPE.
static const std::string devTypeOptiTrack = "OPTITRACK";

class ManNiiAvatarControllerByOptiTrack : public AvatarController
{
private:
	//ManNiiAvatarPosture posture;
	BaseService *optiTrackService;
	std::string optiTrackServiceName;

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
	//void convertEulerAngle2ManNiiPosture(const EulerAngleType &eulerAngle, ManNiiPosture &manNiiAvatarPosture);

	void setJointQuaternion(SimObj *obj, const ManNiiJointQuaternion &jq);

	void setJointQuaternions(SimObj *obj);

	void setJointQuaternionForOptiTrack(SimObj *obj, ManNiiPosture &ManNiiAvatarPosture);

	//void setJointQuaternionsForOculus(SimObj *obj, ManNiiPosture &manNiiAvatarPosture);
};

extern "C" Controller * createController()
{
	return new ManNiiAvatarControllerByOptiTrack;
}
