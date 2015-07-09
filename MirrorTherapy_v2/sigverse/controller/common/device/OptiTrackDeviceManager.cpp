/*
 * OptiTrackDeviceManager.cpp
 *
 *  Created on: 2015/07/10
 *      Author: tome-yamada
 */

#include <sigverse/controller/common/device/OptiTrackDeviceManager.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <cmath>


OptiTrackDeviceManager::OptiTrackDeviceManager()
{
	this->service        = NULL;
	this->serviceName    = "";
	this->deviceType     = "";
	this->deviceUniqueID = "";

	this->handsType = HandsType::RIGHT_HAND;
}

OptiTrackDeviceManager::OptiTrackDeviceManager(std::string &name, std::string &deviceType, std::string &deviceUniqueID)
{
	this->service        = NULL;
	this->serviceName    = name;
	this->deviceType     = deviceType;
	this->deviceUniqueID = deviceUniqueID;

	this->handsType = HandsType::RIGHT_HAND;
}

const std::string OptiTrackDeviceManager::handsType2Str(HandsType handsType)
{
	switch(handsType)
	{
		case RIGHT_HAND : { return "right"; }
		case LEFT_HAND  : { return "left"; }
		default         : { std::cout << "illegal HandsType" << std::endl; throw std::exception(); }
	}
}

///@brief Convert sensor data to avatar posture structure.
void OptiTrackDeviceManager::setSensorData2ManNiiPosture(ManNiiPosture &posture, const OptiTrackSensorData &sensorData)
{
	posture.clearJointValidFlag();

	for(int i=0; i<sensorData.getNRigidBodies(); i++)
	{
		short params = sensorData.getSRigidBodyDataSgv(i).params;

		if(this->handsType==HandsType::RIGHT_HAND)
		{
			switch(sensorData.getSRigidBodyDataSgv(i).ID)
			{
				//RIGID1->RARM_JOINT1
				case 1:{ setRigidBody2ManNiiJoint(posture.joint[ManNiiPosture::RARM_JOINT1], sensorData.getSRigidBodyDataSgv(i)); break; }
				//RIGID2->RARM_JOINT4
				case 2:{ setRigidBody2ManNiiJoint(posture.joint[ManNiiPosture::RARM_JOINT4], sensorData.getSRigidBodyDataSgv(i)); break; }
				//RIGID3->RARM_JOINT6
				case 3:{ setRigidBody2ManNiiJoint(posture.joint[ManNiiPosture::RARM_JOINT6], sensorData.getSRigidBodyDataSgv(i)); break; }
				default:{ break; }
			}
		}
		if(this->handsType==HandsType::LEFT_HAND)
		{
			switch(sensorData.getSRigidBodyDataSgv(i).ID)
			{
				//RIGID1->LARM_JOINT1
				case 1:{ setRigidBody2ManNiiJoint(posture.joint[ManNiiPosture::LARM_JOINT1], sensorData.getSRigidBodyDataSgv(i)); break; }
				//RIGID2->LARM_JOINT4
				case 2:{ setRigidBody2ManNiiJoint(posture.joint[ManNiiPosture::LARM_JOINT4], sensorData.getSRigidBodyDataSgv(i)); break; }
				//RIGID3->LARM_JOINT6
				case 3:{ setRigidBody2ManNiiJoint(posture.joint[ManNiiPosture::LARM_JOINT6], sensorData.getSRigidBodyDataSgv(i)); break; }
				default:{ break; }
			}
		}
	}
}


void OptiTrackDeviceManager::setJointQuaternions2ManNii(SimObj *obj, const ManNiiPosture &posture)
{
	for(int i=0; i<ManNiiPosture::ManNiiJointType_Count; i++)
	{
		ManNiiPosture::ManNiiJoint joint = posture.joint[i];

		if(joint.isValid)
		{
			obj->setJointQuaternion(ManNiiPosture::manNiiJointTypeStr(joint.jointType).c_str(), joint.quaternion.w, joint.quaternion.x, joint.quaternion.y, joint.quaternion.z);
		}
	}
}


void OptiTrackDeviceManager::setRigidBody2ManNiiJoint(ManNiiPosture::ManNiiJoint &manNiiJoint, const OptiTrackSensorData::sRigidBodyDataSgv &rigidBodySgv)
{
	manNiiJoint.quaternion.w = rigidBodySgv.qw;
	manNiiJoint.quaternion.x = rigidBodySgv.qx;
	manNiiJoint.quaternion.y = rigidBodySgv.qy;
	manNiiJoint.quaternion.z = rigidBodySgv.qz;
	manNiiJoint.isValid      = rigidBodySgv.params;
}


bool OptiTrackDeviceManager::changeHandsType(std::string msg)
{
	if(msg==handsType2Str(HandsType::RIGHT_HAND))
	{
		this->handsType = HandsType::RIGHT_HAND;

		std::cout << "Change HandsType to [" << msg << "]." << std::endl;

		return true;
	}
	else if(msg==handsType2Str(HandsType::LEFT_HAND))
	{
		this->handsType = HandsType::LEFT_HAND;

		std::cout << "Change HandsType to [" << msg << "]." << std::endl;

		return true;
	}

	return false;
}
