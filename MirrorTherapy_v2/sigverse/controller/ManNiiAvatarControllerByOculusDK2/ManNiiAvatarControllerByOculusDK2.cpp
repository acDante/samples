/*
 * ManNiiAvatarControllerByOculusDK2.cpp
 *
 *  Created on: 2015/07/30
 *      Author: Wada
 */

#include <sigverse/common/device/OculusRiftDK2SensorData.h>
#include <sigverse/controller/ManNiiAvatarControllerByOculusDK2/ManNiiAvatarControllerByOculusDK2.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <cmath>

///@brief Parameter file name.
const std::string ManNiiAvatarControllerByOculusDK2::parameterFileName = "OculusRiftDK2.ini";


///@brief Initialize this controller.
void ManNiiAvatarControllerByOculusDK2::onInit(InitEvent &evt)
{
	readIniFileAndInitialize();
//
//	SimObj *my = getObj(myname());
}


///@brief Movement of the robot.
double ManNiiAvatarControllerByOculusDK2::onAction(ActionEvent &evt)
{
	bool oculusDK2Available = checkService(this->oculusDK2DeviceManager.serviceName);

	if (oculusDK2Available && this->oculusDK2DeviceManager.service == NULL)
	{
		this->oculusDK2DeviceManager.service = connectToService(this->oculusDK2DeviceManager.serviceName);
	}
	else if (!oculusDK2Available && this->oculusDK2DeviceManager.service != NULL)
	{
		this->oculusDK2DeviceManager.service = NULL;
	}

	return 1.0;
}

void ManNiiAvatarControllerByOculusDK2::onRecvMsg(RecvMsgEvent &evt)
{
	const std::string allMsg = evt.getMsg();

	//std::cout << allMsg << std::endl;

	std::map<std::string, std::vector<std::string> > sensorDataMap = OculusRiftDK2SensorData::decodeSensorData(allMsg);

	if (sensorDataMap.find(MSG_KEY_DEV_TYPE) == sensorDataMap.end()){ return; }

	if(sensorDataMap[MSG_KEY_DEV_TYPE][0]     !=this->oculusDK2DeviceManager.deviceType    ){ return; }
	if(sensorDataMap[MSG_KEY_DEV_UNIQUE_ID][0]!=this->oculusDK2DeviceManager.deviceUniqueID){ return; }

	OculusRiftDK2SensorData sensorData;
	sensorData.setSensorData(sensorDataMap);

	ManNiiPosture posture = OculusDK2DeviceManager::convertQuaternion2ManNiiPosture(sensorData.getQuaternion());

	SimObj *obj = getObj(myname());
	OculusDK2DeviceManager::setJointQuaternions2ManNii(obj, posture);
}



///@brief Read parameter file.
///@return When couldn't read parameter file, return false;
void ManNiiAvatarControllerByOculusDK2::readIniFileAndInitialize()
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

		serviceName    = SERVICE_NAME_OCULUS_DK2;
		deviceType     = DEV_TYPE_OCULUS_DK2;
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

	this->oculusDK2DeviceManager = OculusDK2DeviceManager(serviceName, deviceType, deviceUniqueID);
}


extern "C" Controller * createController()
{
	return new ManNiiAvatarControllerByOculusDK2;
}


