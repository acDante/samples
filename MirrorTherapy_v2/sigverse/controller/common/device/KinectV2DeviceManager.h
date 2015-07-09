/*
 * KinectV2DeviceManager.h
 *
 *  Created on: 2015/07/09
 *      Author: tome-yamada
 */
#ifndef SIGVERSE_KINECT_V2_DEVICE_MANAGER_H
#define SIGVERSE_KINECT_V2_DEVICE_MANAGER_H

#include <sigverse/SimObj.h>
#include <sigverse/ControllerImpl.h>

#include <sigverse/common/device/SensorData.h>
#include <sigverse/common/device/KinectV2SensorData.h>
#include <sigverse/controller/common/ManNiiPosture.h>

class KinectV2DeviceManager
{
public:
	static const double normalization_range;

	KinectV2DeviceManager();
	KinectV2DeviceManager(std::string &name, std::string &deviceType, std::string &deviceUniqueID, double scaleRatio);

	void initPositionAndRotation(SimObj *myself);

	static ManNiiPosture convertSensorData2ManNiiPosture(const KinectV2SensorData &sensorData);
	///@brief Convert Kinect V2 joint orientation to avatar posture structure.
	static ManNiiPosture convertKinectV2JointOrientations2ManNiiPosture(const KinectV2SensorData::KinectV2JointOrientation* kinectV2Joints);
	///@brief Convert Kinect V2 joint position to avatar posture structure.
	static ManNiiPosture convertKinectV2JointPosition2ManNiiPosture(const KinectV2SensorData::KinectV2JointPosition* positionArray);

	static void setJointQuaternion2ManNii(SimObj *obj, const ManNiiPosture::ManNiiJoint &joint);
	static void setJointQuaternions2ManNii(SimObj *obj, const ManNiiPosture &manNiiPosture, const KinectV2SensorData &sensorData);

	void setRootPosition(SimObj *obj, const SigCmn::Vector3 &pos);


	BaseService *service;
	std::string serviceName;
	std::string deviceType;
	std::string deviceUniqueID;

	double scaleRatio;

	SigCmn::Vector3 iniPos;
	double yrot;

	SigCmn::Vector3 startpos;
	bool started;
};

#endif //SIGVERSE_KINECT_V2_DEVICE_MANAGER_H
