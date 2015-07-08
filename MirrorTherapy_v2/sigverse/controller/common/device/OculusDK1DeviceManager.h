/*
 * OculusDK1Service.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */
#ifndef __OCULUS_DK1_SERVICE_H__
#define __OCULUS_DK1_SERVICE_H__

#include <sigverse/SimObj.h>
#include <sigverse/ControllerImpl.h>

#include <sigverse/common/device/SensorData.h>
#include <sigverse/controller/common/ManNiiPosture.h>

class OculusDK1DeviceManager
{
public:
	static const dQuaternion defaultHeadJoint0Quaternion;

	BaseService *service;

	std::string serviceName;
	std::string deviceType;
	std::string deviceUniqueID;

	OculusDK1DeviceManager();
	OculusDK1DeviceManager(std::string &name, std::string &deviceType, std::string &deviceUniqueID);

	///@brief Convert euler angle to avatar posture structure.
	static ManNiiPosture convertEulerAngle2ManNiiPosture(const SigCmn::EulerAngleType &eulerAngle);

	void setJointQuaternions2ManNii(SimObj *obj, const ManNiiPosture &posture);
};

#endif //__OCULUS_DK1_SERVICE_H__
