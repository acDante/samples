/*
 * ManNiiAvatarControllerByOptiTrack.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */
#ifndef SIGVERSE_MAN_NII_AVATAR_CONTROLLER_BY_OPTITRACK_H
#define SIGVERSE_MAN_NII_AVATAR_CONTROLLER_BY_OPTITRACK_H

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>

#include <sigverse/common/device/OptiTrackSensorData.h>
#include <sigverse/controller/common/device/OptiTrackDeviceManager.h>
#include <sigverse/controller/common/ManNiiAvatarController.h>
#include <sigverse/controller/common/ManNiiPosture.h>


class ManNiiAvatarControllerByOptiTrack : public ManNiiAvatarController
{
public:
	///@brief Parameter file name.
	static const std::string parameterFileName;

private:
	void setRigidBody2ManNiiJoint(ManNiiPosture::ManNiiJoint &manNiiJoint, const OptiTrackSensorData::sRigidBodyDataSgv &rigidBodySgv);

public:

	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	void readIniFileAndInitialize();

	OptiTrackDeviceManager optiTrackDeviceManager;

	ManNiiPosture posture;
};

#endif // SIGVERSE_MAN_NII_AVATAR_CONTROLLER_BY_OPTITRACK_H
