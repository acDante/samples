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


class OpenHouse2015Controller : public MirrorTherapyController
{
public:
	///@brief Structure of posture record (with time stamp).
	struct TimeAndPostureType
	{
		long timeStamp;

		Quaternion qLARM_JOINT2;
		Quaternion qLARM_JOINT3;
		Quaternion qRARM_JOINT2;
		Quaternion qRARM_JOINT3;
//
//		ManNiiPosture posture;
	};

	///@brief Posture record.
	std::vector<TimeAndPostureType> pastPostures;

	bool usingOculus;

	Quaternion getQuaternion(const double w, const double x, const double y, const double z);

};

#endif // SIGVERSE_OPEN_HOUSE_2015_CONTROLLER_H
