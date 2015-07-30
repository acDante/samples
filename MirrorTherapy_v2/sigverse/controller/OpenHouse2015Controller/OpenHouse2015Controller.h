/*
 * OpenHouse2015Controller.h
 *
 *  Created on: 2015/06/08
 *      Author: Yamada@tome
 */
#ifndef SIGVERSE_OPEN_HOUSE_2015_CONTROLLER_H
#define SIGVERSE_OPEN_HOUSE_2015_CONTROLLER_H

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>
#include <sigverse/comm/controller/Controller.h>

#include <sigverse/common/device/KinectV2SensorData.h>
#include <sigverse/common/device/OculusRiftDK1SensorData.h>
#include <sigverse/controller/common/AvatarController.h>
#include <sigverse/controller/common/ManNiiPosture.h>
#include <sigverse/controller/MirrorTherapyController/MirrorTherapyController.h>


class OpenHouse2015Controller : public ManNiiAvatarController
{
public:
//	///@brief Structure of posture record (with time stamp).
//	struct TimeAndPostureType
//	{
//		long timeStamp;
//
//		Quaternion qLARM_JOINT2;
//		Quaternion qLARM_JOINT3;
//		Quaternion qRARM_JOINT2;
//		Quaternion qRARM_JOINT3;
////
////		ManNiiPosture posture;
//	};
//
//	///@brief Posture record.
//	std::vector<TimeAndPostureType> pastPostures;

	///@brief Parameter file name.
	static const std::string parameterFileName;

	//Parameter file key name.
	static const std::string paramFileKeyKinectV2ServiceName;
	static const std::string paramFileKeyKinectV2Devicetype;
	static const std::string paramFileKeyKinectV2DeviceUniqueID;

	static const std::string paramFileKeyKinectV2SensorDataMode;
	static const std::string paramFileKeyKinectV2ScaleRatio;

	static const std::string paramFileKeyOculusDK1ServiceName;
	static const std::string paramFileKeyOculusDK1Devicetype;
	static const std::string paramFileKeyOculusDK1DeviceUniqueID;

	static const std::string paramFileValKinectV2SensorDataModeDefault;
	static const double      paramFileValKinectV2ScaleRatioDefault;

	static const std::string paramFileKeyChangeAvatarGUIServiceName;
	static const std::string paramFileValChangeAvatarGUIServiceNameDefault;

	//static param
	static const std::string msgKeyAvatar;
	static const std::string msgKeyReverse;
	static const std::string msgKeyDelay;

	static const double defaultDelayTime;
	static const int    timeSeriesBufferSize;

	enum ReverseModeType
	{
		RIGHTHAND = 0,
		LEFTHAND  = 1,
		NOREVERSE = 2,
		ReverseMode_Count = (NOREVERSE + 1)
	};

	static const std::string reverseModes[ReverseMode_Count];

	static const std::string headName;
	static const std::string mirrorTherapyManName;
	static const std::string longArmManName;
	static const std::string shortArmManName;
	static const std::string robotArmManName;

	///@brief Structure of posture record (with time stamp).
	struct TimeAndPosture
	{
		long timeStamp;
		ManNiiPosture posture;
	};

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	void readIniFileAndInitialize();


	///@brief Generate time stamp for posture record.
	const double generateCurrentTimeStamp();

	int storeCurrentPosture(const ManNiiPosture &posture);

	ManNiiPosture getDelayedPosture(const int currentIndex);

	void invertPosture(ManNiiPosture &posture, const ManNiiPosture &delayedPosture);

	void changeAvatar(const std::map<std::string, std::vector<std::string> > &map);

	///@brief Modify the mode for MirrorTherapy.
	void setReverseModeAndDelayTime(const std::map<std::string, std::vector<std::string> > &map);


	KinectV2DeviceManager  kinectV2DeviceManager;
	OculusDK1DeviceManager oculusDK1DeviceManager;

	///@brief For Mirror therapy variables.
	std::string reverseMode;

	///@brief Time stamp on initialize.
	timeval initTimeVal;

	///@brief Frame numbers during receive message from kinect v2 plugin.
	unsigned long frameNumber;

	///@brief Target delay time.
	double targetDelayTime;

	///@brief Posture record.
	std::vector<TimeAndPosture> pastTimeAndPostures;


	bool isHead = false;
	bool usingOculus = false;
	std::string bodyAvatarName;

	BaseService *guiService;
	std::string guiServiceName;

};

#endif // SIGVERSE_OPEN_HOUSE_2015_CONTROLLER_H
