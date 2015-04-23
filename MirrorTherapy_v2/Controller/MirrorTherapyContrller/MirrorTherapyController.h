/*
 * MirrorTherapyController.h
 *
 *  Created on: 2015/03/25
 *      Author: Nozaki
 */
#ifndef __MIRROR_THERAPY_CONTROLLER_H__
#define __MIRROR_THERAPY_CONTROLLER_H__

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>
#include <sigverse/comm/controller/Controller.h>

#include "../ControllerCommon/AvatarController.h"
#include "../ControllerCommon/ManNiiPosture.h"
#include "../../Common/KinectV2SensorData.h"
#include "../../Common/OculusRiftDK1SensorData.h"


#define PARAM_FILE_NAME_MIRROR_THERAPY_INI  "MirrorTherapy.ini"

//Parameter file information
#define PARAMETER_FILE_KEY_KINECTV2_SERVICE_NAME      "KinectV2.service_name"
#define PARAMETER_FILE_KEY_KINECTV2_DEVICE_TYPE       "KinectV2.device_type"
#define PARAMETER_FILE_KEY_KINECTV2_DEVICE_UNIQUE_ID  "KinectV2.device_unique_id"

//Parameter file information
#define PARAMETER_FILE_KEY_OCULUS_SERVICE_NAME      "Oculus.service_name"
#define PARAMETER_FILE_KEY_OCULUS_DEVICE_TYPE       "Oculus.device_type"
#define PARAMETER_FILE_KEY_OCULUS_DEVICE_UNIQUE_ID  "Oculus.device_unique_id"


#define MSG_KEY_REVERSE "REVERSE"

#define MSG_KEY_DELAY "DELAY"

#define DEFAULT_DELAY_TIME 1000.0

#define TIME_SERIES_BUFFER_SIZE 500


class MirrorTherapyController : public AvatarController
{
private:
	///@brief Parameter file name.
	std::string parameterFileName;

	// Base services for each device.
	BaseService *kinectV2Service;
	std::string kinectV2ServiceName;
	std::string kinectV2DeviceType;
	std::string kinectV2DeviceUniqueID;

	BaseService *oculusDK1Service;
	std::string oculusDK1ServiceName;
	std::string oculusDK1DeviceType;
	std::string oculusDK1DeviceUniqueID;
	
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
	struct TimeAndPostureType
	{
		long timeStamp;
		ManNiiPosture posture;
	};
	
	///@brief Posture record.
	std::vector<TimeAndPostureType> pastPostures;

	void readIniFile();

public:
	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

//	///@brief Get value of DEV_ID from whole message.
//	std::string getDeviceIDFromMessage(const std::string &message);

	///@brief Convert kinect v2 joint orientations to avatar posture structure.
	void convertKinectV2JointOrientations2ManNiiPosture(KinectV2SensorData::KinectV2JointOrientation *kinectV2Joints, ManNiiPosture &manNiiPosture);

	///@brief Convert euler angle to avatar posture structure.
	void convertEulerAngle2ManNiiPosture(const SensorData::EulerAngleType &eulerAngle, ManNiiPosture &manNiiAvatarPosture);

	///@brief Set avatar's joint quaternion. 
	void setJointQuaternion(SimObj *obj, ManNiiPosture::ManNiiJoint &jq);

	///@brief Set avatar's joint quaternions from kinect plugin message.
	void setJointQuaternionsForKinect(SimObj *obj, ManNiiPosture &manNiiPosture);

	///@brief Set avatar's joint quaternions from oculus plugin message.
	void setJointQuaternionsForOculus(SimObj *obj, ManNiiPosture &manNiiAvatarPosture);

	///@brief Generate time stamp for posture record.
	const double generateCurrentTimeStamp();

	///@brief Modify the mode for MirrorTherapy.
	bool setReverseModeAndDelayTime(const std::map<std::string, std::vector<std::string> > &map);
};

extern "C" Controller * createController()
{
	return new MirrorTherapyController;
}

#endif //__MIRROR_THERAPY_CONTROLLER_H__
