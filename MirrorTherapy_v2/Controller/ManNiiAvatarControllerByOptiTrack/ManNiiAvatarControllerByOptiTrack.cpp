/*
 * ManNiiAvatarController.cpp
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#include "ManNiiAvatarControllerByOptiTrack.h"
#include <math.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>


///@brief Initialize this controller.
void ManNiiAvatarControllerByOptiTrack::onInit(InitEvent &evt)
{
	this->parameterFileName = PARAM_FILE_NAME_OPTITRACK_INI;
	readIniFile();

	this->optiTrackService = NULL;

	SimObj *myself = getObj(myname());

	/* Adjustment of knee angles to sit on the chair */
	myself->setJointAngle(ManNiiPosture::manNiiJointTypeStr(ManNiiPosture::RARM_JOINT0).c_str(), DEG2RAD(+90));
	myself->setJointAngle(ManNiiPosture::manNiiJointTypeStr(ManNiiPosture::LARM_JOINT0).c_str(), DEG2RAD(-90));

	/* Root of both legs */
	myself->setJointAngle(ManNiiPosture::manNiiJointTypeStr(ManNiiPosture::RLEG_JOINT2).c_str(), DEG2RAD(-90));
	myself->setJointAngle(ManNiiPosture::manNiiJointTypeStr(ManNiiPosture::LLEG_JOINT2).c_str(), DEG2RAD(-90));
	/* Both knee */
	myself->setJointAngle(ManNiiPosture::manNiiJointTypeStr(ManNiiPosture::RLEG_JOINT4).c_str(), DEG2RAD(+90));
	myself->setJointAngle(ManNiiPosture::manNiiJointTypeStr(ManNiiPosture::LLEG_JOINT4).c_str(), DEG2RAD(+90));

	this->handsType = RIGHT_HAND;

	this->posture = ManNiiPosture();
}


///@brief Movement of the robot.
double ManNiiAvatarControllerByOptiTrack::onAction(ActionEvent &evt)
{
	bool optiTrackAvailable = checkService(this->optiTrackServiceName);

	if (optiTrackAvailable && this->optiTrackService == NULL)
	{
		this->optiTrackService = connectToService(this->optiTrackServiceName);
	}
	else if (!optiTrackAvailable && this->optiTrackService != NULL)
	{
		this->optiTrackService = NULL;
	}

	return 1.0;
}

void ManNiiAvatarControllerByOptiTrack::onRecvMsg(RecvMsgEvent &evt)
{
	try
	{
		const std::string msg = evt.getMsg();

		if(msg==handsType2Str(RIGHT_HAND))
		{
			this->handsType = RIGHT_HAND;
			std::cout << "change HandsType to [" << handsType2Str(RIGHT_HAND) << "]" << std::endl;
		}
		else if(msg==handsType2Str(LEFT_HAND))
		{
			this->handsType = LEFT_HAND;
			std::cout << "change HandsType to [" << handsType2Str(LEFT_HAND) << "]" << std::endl;
		}
		else
		{
			OptiTrackSensorData sensorData;
			std::map<std::string, std::vector<std::string> > sensorDataMap = sensorData.decodeSensorData(msg);

			if (sensorDataMap.find(MSG_KEY_DEV_TYPE) == sensorDataMap.end()){ return; }

			if(sensorDataMap[MSG_KEY_DEV_TYPE][0]     !=this->optiTrackDeviceType    ){ return; }
			if(sensorDataMap[MSG_KEY_DEV_UNIQUE_ID][0]!=this->optiTrackDeviceUniqueID){ return; }

			sensorData.setSensorData(sensorDataMap);

			this->setPosture(sensorData);

			SimObj *obj = getObj(myname());
			this->setJointQuaternionForOptiTrack(obj);
		}
	}
	catch (...)
	{
		std::cout << "catch (...) in ManNiiAvatarControllerByOptiTrack::onRecvMsg" << std::endl;
	}
}


void ManNiiAvatarControllerByOptiTrack::setJointQuaternionForOptiTrack(SimObj *obj)
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

void ManNiiAvatarControllerByOptiTrack::setRigidBody2ManNiiJoint(ManNiiPosture::ManNiiJoint &manNiiJoint, const OptiTrackSensorData::sRigidBodyDataSgv &rigidBodySgv)
{
	manNiiJoint.quaternion.w = rigidBodySgv.qw;
	manNiiJoint.quaternion.x = rigidBodySgv.qx;
	manNiiJoint.quaternion.y = rigidBodySgv.qy;
	manNiiJoint.quaternion.z = rigidBodySgv.qz;
	manNiiJoint.isValid      = rigidBodySgv.params;
}


void ManNiiAvatarControllerByOptiTrack::setPosture(const OptiTrackSensorData &sensorData)
{
	this->posture.clearJointValidFlag();

	for(int i=0; i<sensorData.getNRigidBodies(); i++)
	{
		short params = sensorData.getSRigidBodyDataSgv(i).params;

		if(this->handsType==RIGHT_HAND)
		{
			switch(sensorData.getSRigidBodyDataSgv(i).ID)
			{
				//RIGID1->RARM_JOINT1
				case 1:{ setRigidBody2ManNiiJoint(this->posture.joint[ManNiiPosture::RARM_JOINT1], sensorData.getSRigidBodyDataSgv(i)); break; }
				//RIGID2->RARM_JOINT4
				case 2:{ setRigidBody2ManNiiJoint(this->posture.joint[ManNiiPosture::RARM_JOINT4], sensorData.getSRigidBodyDataSgv(i)); break; }
				//RIGID3->RARM_JOINT6
				case 3:{ setRigidBody2ManNiiJoint(this->posture.joint[ManNiiPosture::RARM_JOINT6], sensorData.getSRigidBodyDataSgv(i)); break; }
				default:{ break; }
			}
		}
		if(this->handsType==LEFT_HAND)
		{
			switch(sensorData.getSRigidBodyDataSgv(i).ID)
			{
				//RIGID1->LARM_JOINT1
				case 1:{ setRigidBody2ManNiiJoint(this->posture.joint[ManNiiPosture::LARM_JOINT1], sensorData.getSRigidBodyDataSgv(i)); break; }
				//RIGID2->LARM_JOINT4
				case 2:{ setRigidBody2ManNiiJoint(this->posture.joint[ManNiiPosture::LARM_JOINT4], sensorData.getSRigidBodyDataSgv(i)); break; }
				//RIGID3->LARM_JOINT6
				case 3:{ setRigidBody2ManNiiJoint(this->posture.joint[ManNiiPosture::LARM_JOINT6], sensorData.getSRigidBodyDataSgv(i)); break; }
				default:{ break; }
			}
		}
	}
}


///@brief Read parameter file.
///@return When couldn't read parameter file, return false;
void ManNiiAvatarControllerByOptiTrack::readIniFile()
{
	std::ifstream ifs(this->parameterFileName.c_str());

	if (ifs.fail())
	{
		std::cout << "Not exist : " << this->parameterFileName << std::endl;
		std::cout << "Use default parameter." << std::endl;

		this->optiTrackServiceName    = SERVICE_NAME_OPTITRACK;
		this->optiTrackDeviceType     = DEV_TYPE_OPTITRACK;
		this->optiTrackDeviceUniqueID = DEV_UNIQUE_ID_0;
	}
	else
	{
		try
		{
			// パラメータファイルが見つかった時は，書いてある内容を取得してセットする．
			std::cout << "Read " << this->parameterFileName << std::endl;
			boost::property_tree::ptree pt;
			boost::property_tree::read_ini(this->parameterFileName, pt);

			this->optiTrackServiceName    = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME);
			this->optiTrackDeviceType     = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE);
			this->optiTrackDeviceUniqueID = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID);
		}
		catch (boost::exception &ex)
		{
			std::cout << this->parameterFileName << " ERR :" << *boost::diagnostic_information_what(ex) << std::endl;
		}
	}

	std::cout << PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME     << ":" << this->optiTrackServiceName    << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE      << ":" << this->optiTrackDeviceType     << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID << ":" << this->optiTrackDeviceUniqueID << std::endl;
}

