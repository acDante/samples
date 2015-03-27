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
	// Base services for each device.
	BaseService *kinectV2Service;
	std::string kinectV2ServiceName;
	BaseService *oculusDK1Service;
	std::string oculusDK1ServiceName;
	
	///@brief Default quaternion for head.
	dQuaternion defaultHeadJoint0Quaternion;

	///@brief Previous euler angle for head.
	double prevYaw, prevPitch, prevRoll;

	///@brief For Mirror therapy variables.
	std::string reverseMode;

	///@brief Time stamp on initialize.
	struct timeval initTimeVal;

	///@brief Frame numbers during receive message from kinect v2 plugin.
	unsigned long frameNumber;

	///@brief Target delay time.
	double targetDelayTime;

	double actualDelayTime;

	///@brief Structure of posture record (with time stamp).
	struct TimeAndPostureType {
		long timeStamp;
		ManNiiPosture posture;
	};
	
	///@brief Posture record.
	std::vector<TimeAndPostureType> pastPostures;

public:

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	///@brief Get value of DEV_ID from whole message.
	std::string getDeviceIDFromMessage(const std::string &message);

	///@brief Convert kinect v2 joint orientations to avatar posture structure.
	void convertKinectV2JointOrientations2ManNiiPosture(KinectV2JointOrientation *kinectV2Joints, ManNiiPosture &manNiiPosture);

	///@brief Convert euler angle to avatar posture structure.
	void convertEulerAngle2ManNiiPosture(const EulerAngleType &eulerAngle, ManNiiPosture &manNiiAvatarPosture);

	///@brief Set avatar's joint quaternion. 
	void setJointQuaternion(SimObj *obj, ManNiiJointQuaternion &jq);

	///@brief Set avatar's joint quaternions from kinect plugin message.
	void setJointQuaternionsForKinect(SimObj *obj, ManNiiPosture &manNiiPosture);

	///@brief Set avatar's joint quaternions from oculus plugin message.
	void setJointQuaternionsForOculus(SimObj *obj, ManNiiPosture &manNiiAvatarPosture);

	///@brief Generate time stamp for posture record.
	const double generateCurrentTimeStamp();

	///@brief Modify the mode for MirrorTherapy.
	bool setReverseModeAndDelayTime(const std::string &message);
};

extern "C" Controller * createController()
{
	return new MirrorTherapyController;
}
