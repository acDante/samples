/*
 * ManNiiAvatarController.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */
#ifndef __MAN_NII_AVATAR_CONTROLLER_BY_KINECT_V2_H__
#define __MAN_NII_AVATAR_CONTROLLER_BY_KINECT_V2_H__

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>

#include <sigverse/common/device/KinectV2SensorData.h>
#include <sigverse/controller/common/ManNiiAvatarController.h>
#include <sigverse/controller/common/ManNiiPosture.h>
#include <sigverse/controller/common/device/KinectV2DeviceManager.h>


class ManNiiAvatarControllerByKinectV2 : public ManNiiAvatarController
{
public:
	///@brief Parameter file name.
	static const std::string parameterFileName;

	//Parameter file information
	static const std::string paramFileKeyKinectV2SensorDataMode;
	static const std::string paramFileKeyKinectV2ScaleRatio;
	static const std::string paramFileValKinectV2SensorDataModeDefault;
	static const double      paramFileValKinectV2ScaleRatioDefault;

	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	void readIniFileAndInitialize();

	KinectV2DeviceManager kinectV2DeviceManager;
};

#endif //__MAN_NII_AVATAR_CONTROLLER_BY_KINECT_V2_H__
