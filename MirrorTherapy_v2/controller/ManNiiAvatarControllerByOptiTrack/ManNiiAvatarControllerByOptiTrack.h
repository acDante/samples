/*
 * ManNiiAvatarControllerByOptiTrack.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */
#ifndef __MAN_NII_AVATAR_CONTROLLER_BY_OPTITRACK_H__
#define __MAN_NII_AVATAR_CONTROLLER_BY_OPTITRACK_H__

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>

#include "../../Common/OptiTrackSensorData.h"

#include "../ControllerCommon/AvatarController.h"
#include "../ControllerCommon/ManNiiPosture.h"

#define PARAM_FILE_NAME_OPTITRACK_INI  "OptiTrack.ini"


class ManNiiAvatarControllerByOptiTrack : public AvatarController
{
private:
	//Hand that are linked with the device.
	enum HandsType{ RIGHT_HAND, LEFT_HAND };

	std::string handsType2Str(HandsType handsType)
	{
		switch(handsType)
		{
			case RIGHT_HAND : { return "right"; }
			case LEFT_HAND  : { return "left"; }
			default         : { std::cout << "illegal HandsType" << std::endl; throw std::string("illegal HandsType"); }
		}
	}

	///@brief Parameter file name.
	std::string parameterFileName;

	//ManNiiAvatarPosture posture;
	BaseService *optiTrackService;

	std::string optiTrackServiceName;
	std::string optiTrackDeviceType;
	std::string optiTrackDeviceUniqueID;

//	dQuaternion defaultHeadJoint0Quaternion;

//	double prevYaw, prevPitch, prevRoll;

	HandsType handsType;

	ManNiiPosture posture;

	void readIniFile();

	void setRigidBody2ManNiiJoint(ManNiiPosture::ManNiiJoint &manNiiJoint, const OptiTrackSensorData::sRigidBodyDataSgv &rigidBodySgv);

public:

	//ManNiiAvatarPosture posture;

	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	///@brief Convert euler angle to avatar posture structure.
	//void convertEulerAngle2ManNiiPosture(const EulerAngleType &eulerAngle, ManNiiPosture &manNiiAvatarPosture);

	void setPosture(const OptiTrackSensorData &sensorData);
//	void setJointQuaternion(SimObj *obj, const ManNiiJoint &joint);
	void setJointQuaternionForOptiTrack(SimObj *obj);

//	void setJointQuaternionForOptiTrack(SimObj *obj, const ManNiiJoint &joint);

	//void setJointQuaternionsForOculus(SimObj *obj, ManNiiPosture &manNiiAvatarPosture);
};

extern "C" Controller * createController()
{
	return new ManNiiAvatarControllerByOptiTrack;
}

#endif //__MAN_NII_AVATAR_CONTROLLER_BY_OPTITRACK_H__
