/*
 * ManNiiAvatarControllerByOculusDK2.h
 *
 *  Created on: 2015/07/30
 *      Author: Wada
 */
#ifndef SIGVERSE_MAN_NII_AVATAR_CONTROLLER_BY_OCULUS_DK2_H
#define SIGVERSE_MAN_NII_AVATAR_CONTROLLER_BY_OCULUS_DK2_H

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>

#include <sigverse/common/device/OculusRiftDK2SensorData.h>

#include <sigverse/controller/common/device/OculusDK2DeviceManager.h>
#include <sigverse/controller/common/ManNiiPosture.h>
#include <sigverse/controller/common/ManNiiAvatarController.h>


class ManNiiAvatarControllerByOculusDK2 : public ManNiiAvatarController
{
public:
	///@brief Parameter file name.
	static const std::string parameterFileName;

	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	void readIniFileAndInitialize();

	OculusDK2DeviceManager oculusDK2DeviceManager;

	ManNiiPosture posture;
};

#endif // SIGVERSE_MAN_NII_AVATAR_CONTROLLER_BY_OCULUS_DK2_H
