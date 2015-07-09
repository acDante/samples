/*
 * ManNiiAvatarController.cpp
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#include <sigverse/common/device/SensorData.h>
#include <sigverse/controller/ManNiiAvatarControllerByKinectV2/ManNiiAvatarControllerByKinectV2.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <cmath>

///@brief Parameter file name.
const std::string ManNiiAvatarControllerByKinectV2::parameterFileName = "KinectV2.ini";

//Parameter file information
const std::string ManNiiAvatarControllerByKinectV2::paramFileKeyKinectV2SensorDataMode = "KinectV2.sensor_data_mode";
const std::string ManNiiAvatarControllerByKinectV2::paramFileKeyKinectV2ScaleRatio     = "KinectV2.scale_ratio";

const std::string ManNiiAvatarControllerByKinectV2::paramFileValKinectV2SensorDataModeDefault = "QUATERNION";
const double      ManNiiAvatarControllerByKinectV2::paramFileValKinectV2ScaleRatioDefault     = 10000.0;

///@brief Initialize this controller.
void ManNiiAvatarControllerByKinectV2::onInit(InitEvent &evt)
{
	this->readIniFileAndInitialize();

	SimObj *myself = getObj(myname());

	this->kinectV2DeviceManager.initPositionAndRotation(myself);
}


///@brief Movement of the robot.
double ManNiiAvatarControllerByKinectV2::onAction(ActionEvent &evt)
{
	bool kinectV2Available = checkService(this->kinectV2DeviceManager.serviceName);

	if (kinectV2Available && this->kinectV2DeviceManager.service == NULL)
	{
		this->kinectV2DeviceManager.service = connectToService(this->kinectV2DeviceManager.serviceName);
	}
	else if (!kinectV2Available && this->kinectV2DeviceManager.service != NULL)
	{
		this->kinectV2DeviceManager.service = NULL;
	}

	return 1.0;
}

void ManNiiAvatarControllerByKinectV2::onRecvMsg(RecvMsgEvent &evt)
{
	try
	{
		std::string allMsg = evt.getMsg();

//		std::cout << "msg:" << allMsg << std::endl;

		// Decode message to sensor data of kinect v2.
		std::map<std::string, std::vector<std::string> > sensorDataMap = KinectV2SensorData::decodeSensorData(allMsg);

		if (sensorDataMap.find(MSG_KEY_DEV_TYPE) == sensorDataMap.end()){ return; }
		if(sensorDataMap[MSG_KEY_DEV_TYPE][0]     !=this->kinectV2DeviceManager.deviceType    ){ return; }
		if(sensorDataMap[MSG_KEY_DEV_UNIQUE_ID][0]!=this->kinectV2DeviceManager.deviceUniqueID){ return; }

		KinectV2SensorData sensorData;
		sensorData.setSensorData(sensorDataMap);

		ManNiiPosture posture = KinectV2DeviceManager::convertSensorData2ManNiiPosture(sensorData);

		// Set SIGVerse quaternions and positions
		SimObj *obj = getObj(myname());
		this->kinectV2DeviceManager.setRootPosition(obj, sensorData.rootPosition);
		KinectV2DeviceManager::setJointQuaternions2ManNii(obj, posture, sensorData);
	}
	catch(SimObj::NoAttributeException &err)
	{
		LOG_MSG(("NoAttributeException: %s", err.msg()));
	}
	catch(SimObj::AttributeReadOnlyException &err)
	{
		LOG_MSG(("AttributeReadOnlyException: %s", err.msg()));
	}
	catch(SimObj::Exception &err)
	{
		LOG_MSG(("Exception: %s", err.msg()));
	}
	catch(...)
	{
		std::cout << "some error occurred." << std::endl;
	}
}


///@brief Read parameter file.
///@return When couldn't read parameter file, return false;
void ManNiiAvatarControllerByKinectV2::readIniFileAndInitialize()
{
	std::ifstream ifs(parameterFileName.c_str());

	std::string serviceName;
	std::string deviceType;
	std::string deviceUniqueID;
	double      scaleRatio;
	std::string sensorDataModeStr;

	// Parameter file is "not" exists.
	if (ifs.fail())
	{
		std::cout << "Not exist : " << parameterFileName << std::endl;
		std::cout << "Use default parameter." << std::endl;

		serviceName    = SERVICE_NAME_KINECT_V2;
		deviceType     = DEV_TYPE_KINECT_V2;
		deviceUniqueID = DEV_UNIQUE_ID_0;

		scaleRatio        = paramFileValKinectV2ScaleRatioDefault;
		sensorDataModeStr = paramFileValKinectV2SensorDataModeDefault;
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

			scaleRatio        = pt.get<double>(paramFileKeyKinectV2ScaleRatio);
			sensorDataModeStr = pt.get<std::string>(paramFileKeyKinectV2SensorDataMode);
		}
		catch (boost::exception &ex)
		{
			std::cout << parameterFileName << " ERR :" << *boost::diagnostic_information_what(ex) << std::endl;
		}
	}

	std::cout << PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME     << ":" << serviceName    << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE      << ":" << deviceType     << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID << ":" << deviceUniqueID << std::endl;
	std::cout << paramFileKeyKinectV2SensorDataMode << ":" << sensorDataModeStr << std::endl;
	std::cout << paramFileKeyKinectV2ScaleRatio     << ":" << scaleRatio << std::endl;

	this->kinectV2DeviceManager = KinectV2DeviceManager(serviceName, deviceType, deviceUniqueID, scaleRatio);

	// Set sensor data mode.
	KinectV2SensorData::setSensorDataMode(sensorDataModeStr);
}


extern "C" Controller * createController()
{
	return new ManNiiAvatarControllerByKinectV2;
}


