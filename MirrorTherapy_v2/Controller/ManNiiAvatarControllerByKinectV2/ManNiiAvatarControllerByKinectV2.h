/*
 * ManNiiAvatarController.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#include <sigverse/Controller.h>
#include <sigverse/comm/controller/Controller.h>
#include <sigverse/ControllerEvent.h>

#include "../ControllerCommon/AvatarController.h"
#include "../ControllerCommon/ManNiiPosture.h"
#include "../../Common/KinectV2SensorData.h"


#define KINECT_SERVICE_NAME "SIGKINECT"
static const std::string kinectV2ServiceNameDefault = KINECT_SERVICE_NAME;

// Define DEV_TYPE.
static const std::string devTypeKinectV2 = "KINECTV2";

class ManNiiAvatarControllerByKinectV2 : public AvatarController
{
private:
	//ManNiiAvatarPosture posture;
	BaseService *kinectV2Service;
	std::string kinectV2ServiceName;


public:

	//ManNiiAvatarPosture posture;

	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	///@brief Convert euler angle to avatar posture structure.
	void convertKinectV2JointOrientations2ManNiiPosture(KinectV2JointOrientation *kinectV2Joints, ManNiiPosture &manNiiPosture);
	//void convertKinectV2JointOrientations2ManNiiPosture(std::vector<KinectV2JointOrientation> &kinectV2Joints, ManNiiPosture &manNiiPosture);

	void setJointQuaternion(SimObj *obj, ManNiiJointQuaternion &jq);

	void setJointQuaternionsForKinect(SimObj *obj, ManNiiPosture &manNiiPosture);
};

extern "C" Controller * createController()
{
	return new ManNiiAvatarControllerByKinectV2;
}
