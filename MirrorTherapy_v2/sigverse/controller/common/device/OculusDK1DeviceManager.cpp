/*
 * OculusDK1Service.cpp
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#include <sigverse/controller/common/device/OculusDK1DeviceManager.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <cmath>


const dQuaternion OculusDK1DeviceManager::defaultHeadJoint0Quaternion = { 1.0, 0.0, 0.0, 0.0 };


OculusDK1DeviceManager::OculusDK1DeviceManager()
{
	this->service        = NULL;
	this->serviceName    = "";
	this->deviceType     = "";
	this->deviceUniqueID = "";
}

void OculusDK1DeviceManager::setJointQuaternions2ManNii(SimObj *obj, const ManNiiPosture &posture)
{
	ManNiiPosture::ManNiiJoint joint = posture.joint[ManNiiPosture::HEAD_JOINT0];

	obj->setJointQuaternion(ManNiiPosture::manNiiJointTypeStr(joint.jointType).c_str(), joint.quaternion.w, joint.quaternion.x, joint.quaternion.y, joint.quaternion.z);
}


ManNiiPosture OculusDK1DeviceManager::convertEulerAngle2ManNiiPosture(const SigCmn::EulerAngleType &eulerAngle)
{
	ManNiiPosture posture;

	dQuaternion qyaw;
	dQuaternion qpitch;
	dQuaternion qroll;

	qyaw[0] = std::cos(eulerAngle.yaw/2.0);
	qyaw[1] = 0.0;
	qyaw[2] = std::sin(eulerAngle.yaw/2.0);
	qyaw[3] = 0.0;

	qpitch[0] = std::cos(-eulerAngle.pitch/2.0);
	qpitch[1] = std::sin(-eulerAngle.pitch/2.0);
	qpitch[2] = 0.0;
	qpitch[3] = 0.0;

	qroll[0] = std::cos(-eulerAngle.roll/2.0);
	qroll[1] = 0.0;
	qroll[2] = 0.0;
	qroll[3] = std::sin(-eulerAngle.roll/2.0);
	dQuaternion tmpQ1;
	dQuaternion tmpQ2;

	dQMultiply0(tmpQ1, qyaw, qpitch);

	dQMultiply0(tmpQ2, tmpQ1, qroll);


	dQuaternion tmpQ3;

	//dQuaternion defaultQ = this->defaultHeadJointQuaternion;
	dQMultiply1(tmpQ3, defaultHeadJoint0Quaternion, tmpQ2);

	Quaternion tmpQ4(tmpQ3[0], tmpQ3[1], tmpQ3[2], tmpQ3[3]);

	posture.joint[ManNiiPosture::HEAD_JOINT0].jointType  = ManNiiPosture::HEAD_JOINT0;
	posture.joint[ManNiiPosture::HEAD_JOINT0].quaternion = tmpQ4;

	return posture;
}


