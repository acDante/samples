/*
 * ManNiiController.h
 */


#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>

#include "AvatarController.h"
#include "ManNiiAvatarPosture.h"

#define SERVICE_NAME "SIGKINECT"
static const std::string serviceNameDefault = SERVICE_NAME;

// Define DEV_TYPE.
static const std::string devTypeKinectV2 = "KINECTV2";
static const std::string devTypeOculus = "OCULUS";


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


class ManNiiAvatarController : public AvatarController
{
private:
	//ManNiiAvatarPosture posture;
	BaseService *kinectV2Service;
	std::string serviceName;

public:

	//ManNiiAvatarPosture posture;

	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	void convertMessage2AvatarPosture(const std::string &message);

	void setJointQuaternion(SimObj *obj, const ManNiiJointQuaternion &jq);

	void setJointQuaternions(SimObj *obj);

	void setJointQuaternions(SimObj *obj, ManNiiAvatarPosture *manNiiAvatarPosture);

	KinectV2JointType getJointTypeFromKey(const std::string &key);

	void convertKinectV2JointOrientations2ManNiiAvatarPosture(KinectV2JointOrientation* kinectV2Joints, ManNiiAvatarPosture &manNiiAvatarPosture);
};

