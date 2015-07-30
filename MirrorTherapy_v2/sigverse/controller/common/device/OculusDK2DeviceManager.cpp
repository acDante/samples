/*
 * OculusDK2DeviceManager.cpp
 *
 *  Created on: 2015/07/30
 *      Author: Wada
 */

#include <sigverse/controller/common/device/OculusDK2DeviceManager.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <cmath>


const dQuaternion OculusDK2DeviceManager::defaultHeadJoint0Quaternion = { 1.0, 0.0, 0.0, 0.0 };


OculusDK2DeviceManager::OculusDK2DeviceManager()
{
	this->service        = NULL;
	this->serviceName    = "";
	this->deviceType     = "";
	this->deviceUniqueID = "";
}

OculusDK2DeviceManager::OculusDK2DeviceManager(std::string &name, std::string &deviceType, std::string &deviceUniqueID)
{
	this->service        = NULL;
	this->serviceName    = name;
	this->deviceType     = deviceType;
	this->deviceUniqueID = deviceUniqueID;
}

void OculusDK2DeviceManager::setJointQuaternions2ManNii(SimObj *obj, const ManNiiPosture &posture)
{
	ManNiiPosture::ManNiiJoint joint = posture.joint[ManNiiPosture::HEAD_JOINT0];

	obj->setJointQuaternion(ManNiiPosture::manNiiJointTypeStr(joint.jointType).c_str(), joint.quaternion.w, joint.quaternion.x, joint.quaternion.y, joint.quaternion.z);
}


ManNiiPosture OculusDK2DeviceManager::convertQuaternion2ManNiiPosture(const SigCmn::Vector4 &quaternion)
{
	ManNiiPosture posture;

	dQuaternion tmpQ1;
	tmpQ1[0] = quaternion.w;
	tmpQ1[1] = -quaternion.x;
	tmpQ1[2] = quaternion.y;
	tmpQ1[3] = -quaternion.z;

	dQuaternion tmpQ2;

	//dQuaternion defaultQ = this->defaultHeadJointQuaternion;
	dQMultiply1(tmpQ2, defaultHeadJoint0Quaternion, tmpQ1);

	Quaternion tmpQ3(tmpQ2[0], tmpQ2[1], tmpQ2[2], tmpQ2[3]);

	posture.joint[ManNiiPosture::HEAD_JOINT0].jointType  = ManNiiPosture::HEAD_JOINT0;
	posture.joint[ManNiiPosture::HEAD_JOINT0].quaternion = tmpQ3;

	return posture;
}


