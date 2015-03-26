/*
 * MirrorTherapyController.h
 *
 *  Created on: 2015/03/25
 *      Author: Nozaki
 */

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>
#include <sigverse/comm/controller/Controller.h>

#include "../ControllerCommon/AvatarController.h"
#include "../ControllerCommon/ManNiiPosture.h"
#include "../../Common/KinectV2SensorData.h"
#include "../../Common/OculusRiftDK1SensorData.h"

// Define service name.
#define KINECT_SERVICE_NAME "SIGKINECT"
static const std::string kinectV2ServiceNameDefault = KINECT_SERVICE_NAME;
#define OCULUS_SERVICE_NAME "SIGOCULUS"
static const std::string oculusDK1ServiceNameDefault = OCULUS_SERVICE_NAME;

// Define DEV_TYPE.
static const std::string devTypeKinectV2 = "KINECTV2";
static const std::string devTypeOculus = "OCULUSDK1";

#define TIMESERIESBUFFERSIZE 500
static const int timeSeriesBufferSize = TIMESERIESBUFFERSIZE;

#define REVERSEKEY "REVERSE"
static const std::string reverseKey = REVERSEKEY;

#define DELAYKEY "DELAY"
static const std::string delayKey = DELAYKEY;

#define DEFAULT_DELAY_TIME 1000.0
static const double defaultDelayTime = DEFAULT_DELAY_TIME;

class MirrorTherapyController : public AvatarController
{
private:
	BaseService *kinectV2Service;
	std::string kinectV2ServiceName;
	BaseService *oculusDK1Service;
	std::string oculusDK1ServiceName;

	dQuaternion defaultHeadJoint0Quaternion;

	double prevYaw, prevPitch, prevRoll;

	// For Mirror therapy variables.
	std::string reverseMode;

	///@brief Time stamp on initialize.
	struct timeval initTimeVal;

	unsigned long frameNumber;

	double targetDelayTime;

	double actualDelayTime;

	struct TimeAndPostureType {
		long timeStamp;
		ManNiiPosture posture;
	};
	std::vector<TimeAndPostureType> pastPostures;

public:

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	std::string getDeviceIDFromMessage(const std::string &message);

	///@brief Convert kinect v2 joint orientations to avatar posture structure.
	void convertKinectV2JointOrientations2ManNiiPosture(KinectV2JointOrientation *kinectV2Joints, ManNiiPosture &manNiiPosture);

	///@brief Convert euler angle to avatar posture structure.
	void convertEulerAngle2ManNiiPosture(const EulerAngleType &eulerAngle, ManNiiPosture &manNiiAvatarPosture);

	///
	void setJointQuaternion(SimObj *obj, ManNiiJointQuaternion &jq);

	void setJointQuaternionsForKinect(SimObj *obj, ManNiiPosture &manNiiPosture);

	void setJointQuaternionsForOculus(SimObj *obj, ManNiiPosture &manNiiAvatarPosture);

	const double generateCurrentTimeStamp();

	bool setReverseModeAndDelayTime(const std::string &message);
};

extern "C" Controller * createController()
{
	return new MirrorTherapyController;
}
