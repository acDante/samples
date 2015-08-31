/*
 * ManNiiAvatarController.cpp
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#include <sigverse/common/SigCmn.h>
#include <sigverse/controller/ManNiiAvatarControllerByOptiTrack/ManNiiAvatarControllerByOptiTrack.h>
#include <math.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>


///@brief Parameter file name.
const std::string ManNiiAvatarControllerByOptiTrack::parameterFileName = "OptiTrack.ini";


///@brief Initialize this controller.
void ManNiiAvatarControllerByOptiTrack::onInit(InitEvent &evt)
{
	this->readIniFileAndInitialize();

	SimObj *myself = getObj(myname());

	/* Adjustment of knee angles to sit on the chair */
	myself->setJointAngle(ManNiiPosture::manNiiJointTypeStr(ManNiiPosture::RARM_JOINT0).c_str(), SigCmn::deg2rad(+90));
	myself->setJointAngle(ManNiiPosture::manNiiJointTypeStr(ManNiiPosture::LARM_JOINT0).c_str(), SigCmn::deg2rad(+90));

	/* Root of both legs */
	myself->setJointAngle(ManNiiPosture::manNiiJointTypeStr(ManNiiPosture::RLEG_JOINT2).c_str(), SigCmn::deg2rad(-90));
	myself->setJointAngle(ManNiiPosture::manNiiJointTypeStr(ManNiiPosture::LLEG_JOINT2).c_str(), SigCmn::deg2rad(-90));
	/* Both knee */
	myself->setJointAngle(ManNiiPosture::manNiiJointTypeStr(ManNiiPosture::RLEG_JOINT4).c_str(), SigCmn::deg2rad(+90));
	myself->setJointAngle(ManNiiPosture::manNiiJointTypeStr(ManNiiPosture::LLEG_JOINT4).c_str(), SigCmn::deg2rad(+90));

	this->posture = ManNiiPosture();
}


///@brief Movement of the robot.
double ManNiiAvatarControllerByOptiTrack::onAction(ActionEvent &evt)
{
	bool optiTrackAvailable = checkService(this->optiTrackDeviceManager.serviceName);

	if (optiTrackAvailable && this->optiTrackDeviceManager.service == NULL)
	{
		this->optiTrackDeviceManager.service = connectToService(this->optiTrackDeviceManager.serviceName);
	}
	else if (!optiTrackAvailable && this->optiTrackDeviceManager.service != NULL)
	{
		this->optiTrackDeviceManager.service = NULL;
	}

	return 1.0;
}

void ManNiiAvatarControllerByOptiTrack::onRecvMsg(RecvMsgEvent &evt)
{
	try
	{
		const std::string msg = evt.getMsg();

//		std::cout << "msg:" << msg << std::endl;

		bool handsTypeChanged = this->optiTrackDeviceManager.changeHandsType(msg);

		if(handsTypeChanged){ return; }

		OptiTrackSensorData sensorData;
		std::map<std::string, std::vector<std::string> > sensorDataMap = sensorData.decodeSensorData(msg);

		if (sensorDataMap.find(MSG_KEY_DEV_TYPE) == sensorDataMap.end()){ return; }

		if(sensorDataMap[MSG_KEY_DEV_TYPE][0]     !=this->optiTrackDeviceManager.deviceType    ){ return; }
		if(sensorDataMap[MSG_KEY_DEV_UNIQUE_ID][0]!=this->optiTrackDeviceManager.deviceUniqueID){ return; }

		sensorData.setSensorData(sensorDataMap);

		this->optiTrackDeviceManager.setSensorData2ManNiiPosture(this->posture, sensorData);

		SimObj *obj = getObj(myname());
		OptiTrackDeviceManager::setJointQuaternions2ManNii(obj, this->posture);
	}
	catch (...)
	{
		std::cout << "catch (...) in ManNiiAvatarControllerByOptiTrack::onRecvMsg" << std::endl;
	}
}


///@brief Read parameter file.
///@return When couldn't read parameter file, return false;
void ManNiiAvatarControllerByOptiTrack::readIniFileAndInitialize()
{
	std::ifstream ifs(parameterFileName.c_str());

	std::string serviceName;
	std::string deviceType;
	std::string deviceUniqueID;

	// Parameter file is "not" exists.
	if (ifs.fail())
	{
		std::cout << "Not exist : " << parameterFileName << std::endl;
		std::cout << "Use default parameter." << std::endl;

		serviceName    = SERVICE_NAME_OPTITRACK;
		deviceType     = DEV_TYPE_OPTITRACK;
		deviceUniqueID = DEV_UNIQUE_ID_0;
	}
	// Parameter file is exists.
	else
	{
		try
		{
			std::cout << "Read " << parameterFileName << std::endl;
			boost::property_tree::ptree pt;
			boost::property_tree::read_ini(parameterFileName, pt);

			serviceName    = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME);
			deviceType     = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE);
			deviceUniqueID = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID);
		}
		catch (boost::exception &ex)
		{
			std::cout << parameterFileName << " ERR :" << *boost::diagnostic_information_what(ex) << std::endl;
		}
	}

	std::cout << PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME     << ":" << serviceName    << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE      << ":" << deviceType     << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID << ":" << deviceUniqueID << std::endl;

	this->optiTrackDeviceManager = OptiTrackDeviceManager(serviceName, deviceType, deviceUniqueID);
}


extern "C" Controller * createController()
{
	return new ManNiiAvatarControllerByOptiTrack;
}
