/*
 * MirrorTherapyController.h
 *
 *  Created on: 2015/03/16
 *      Author: Nozaki
 */


#ifndef MIRRORTHERAPYCONTROLLER_H_
#define MIRRORTHERAPYCONTROLLER_H_

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>
#include <sys/time.h> // For time stamp.

#include "AvatarController.h"
#include "ManNiiPosture.h"

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

#define PARMETER_FILE_NAME "MirrorTherapyController.conf"
static const std::string parameterFileName = PARMETER_FILE_NAME;

#ifndef _Vector4_
#define _Vector4_
///@brief For expression quaternion.
typedef struct _Vector4
{
	float x;
	float y;
	float z;
	float w;
} 	Vector4;
#endif //_Vector4_

#ifndef _KinectV2JointType_
#define _KinectV2JointType_

enum KinectV2JointType
{
	SpineBase		= 0,
	SpineMid		= 1,
	Neck			= 2,
	Head			= 3,
	ShoulderLeft	= 4,
	ElbowLeft		= 5,
	WristLeft		= 6,
	HandLeft		= 7,
	ShoulderRight	= 8,
	ElbowRight		= 9,
	WristRight		= 10,
	HandRight		= 11,
	HipLeft			= 12,
	KneeLeft		= 13,
	AnkleLeft		= 14,
	FootLeft		= 15,
	HipRight		= 16,
	KneeRight		= 17,
	AnkleRight		= 18,
	FootRight		= 19,
	SpineShoulder	= 20,
	HandTipLeft		= 21,
	ThumbLeft		= 22,
	HandTipRight	= 23,
	ThumbRight		= 24,
	KinectV2JointType_Count = (ThumbRight + 1)
};
#endif // _KinectV2JointType_

#ifndef _KinectV2JointOrientation_
#define _KinectV2JointOrientation_
typedef struct _KinectV2JointOrientation
{
	KinectV2JointType jointType;
	Vector4 orientation;
} 	KinectV2JointOrientation;

#endif // _KinectV2JointOrientation_

#ifndef _EulerAngleType_
#define _EulerAngleType_
typedef struct _EulerAngleType
{
	float yaw;
	float pitch;
	float roll;
}	EulerAngleType;
#endif // _EulerAngleType_


enum ReverseModeType {
	NOREVERSE = 0,
	RIGHTHAND = 1,
	LEFTHAND  = 2,
	ReverseMode_Count = (LEFTHAND + 1)
};

static const std::string reverseModes[ReverseMode_Count] = {
	"NOREVERSE",
	"RIGHTHAND",
	"LEFTHAND"
};

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
	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	///@brief Convert message to avatar posture, and set joints.
	void convertMessage2Posture(const std::string &message);

	///@brief Convert message to records. Each record consisted of "KEY:VALUE;".
	void convertMessage2Records(const std::string &message, std::vector<Record> &records);

	///@brief Generate euler angle value(s) for head joint from one record.
	EulerAngleType generateEulerAngleFromRecord(const Record &record);

	///@brief Convert euler angle to avatar posture structure.
	void convertEulerAngle2ManNiiPosture(const EulerAngleType &eulerAngle, ManNiiPosture &manNiiAvatarPosture);

	///@brief Convert records to KinectV2 joint orientation structure.
	void convertRecords2KinectV2JointOrientations(std::vector<Record> &kinectRecords, KinectV2JointOrientation *kinectV2JointOrientations);

	void convertKinectV2JointOrientations2ManNiiPosture(KinectV2JointOrientation *kinectV2Joints, ManNiiPosture &manNiiAvatarPosture);

	void setJointQuaternion(SimObj *obj, const ManNiiJointQuaternion &jq);

	void setJointQuaternions(SimObj *obj);

	void setJointQuaternionsForKinect(SimObj *obj, ManNiiPosture *manNiiAvatarPosture);

	void setJointQuaternionsForOculus(SimObj *obj, ManNiiPosture *manNiiAvatarPosture);

	KinectV2JointType getJointTypeFromKey(const std::string &key);

	const bool isReverseModeOk(const std::string &mode_string);

	bool setReverseModeAndDelayTime(const std::string &message);

	const double generateCurrentTimeStamp();

	const bool isDelayTimeRangeOk(const double &delayTime);

	const bool isDelayTimeRangeOk(const std::string &delayTime);
};

extern "C" Controller * createController()
{
	return new MirrorTherapyController;
}

#endif /* MIRRORTHERAPYCONTROLLER_H_ */
