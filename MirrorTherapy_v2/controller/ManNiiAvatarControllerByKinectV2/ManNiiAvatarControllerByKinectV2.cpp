/*
 * ManNiiAvatarController.cpp
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#include "../../Common/SensorData.h"
#include "ManNiiAvatarControllerByKinectV2.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>

///@brief Parameter file name.
const std::string ManNiiAvatarControllerByKinectV2::parameterFileName = "KinectV2.ini";

//Parameter file information
const std::string ManNiiAvatarControllerByKinectV2::paramFileKeyKinectV2SensorDataMode = "KinectV2.sensor_data_mode";
const KinectV2SensorData::SensorDataMode ManNiiAvatarControllerByKinectV2::paramFileValKinectV2SensorDataModeDefault = KinectV2SensorData::SensorDataMode::Quaternion;


///@brief Initialize this controller.
void ManNiiAvatarControllerByKinectV2::onInit(InitEvent &evt)
{
	readIniFile();

	this->kinectV2Service = NULL;
	SimObj *myself = getObj(myname());
}


///@brief Movement of the robot.
double ManNiiAvatarControllerByKinectV2::onAction(ActionEvent &evt)
{
	bool kinectV2Available = checkService(this->kinectV2ServiceName);

	if (kinectV2Available && this->kinectV2Service == NULL)
	{
		this->kinectV2Service = connectToService(this->kinectV2ServiceName);
	}
	else if (!kinectV2Available && this->kinectV2Service != NULL)
	{
		this->kinectV2Service = NULL;
	}

	return 1.0;
}

void ManNiiAvatarControllerByKinectV2::onRecvMsg(RecvMsgEvent &evt)
{
	try
	{
		// Receive message.
		const std::string allMsg = evt.getMsg();

//		std::cout << "msg:" << allMsg << std::endl;

		// Decode message to sensor data of kinect v2.
		KinectV2SensorData sensorData(this->sensorDataModeStr);

		std::map<std::string, std::vector<std::string> > sensorDataMap = sensorData.decodeSensorData(allMsg);

		if (sensorDataMap.find(MSG_KEY_DEV_TYPE) == sensorDataMap.end()){ return; }

		if(sensorDataMap[MSG_KEY_DEV_TYPE][0]     !=this->kinectV2DeviceType    ){ return; }
		if(sensorDataMap[MSG_KEY_DEV_UNIQUE_ID][0]!=this->kinectV2DeviceUniqueID){ return; }

		sensorData.setSensorData(sensorDataMap);

		// Get quaternions(orientations) of kinect v2 format, from sensor data of kinect v2.
		KinectV2SensorData::KinectV2JointOrientation tmpKinectV2JointOrientations[KinectV2SensorData::KinectV2JointType_Count];
		sensorData.getKinectV2JointOrientation(tmpKinectV2JointOrientations);

		// Convert kinect v2 quaternions(orientations) to man-nii posture(sigverse quaternion format).
		ManNiiPosture manNiiPosture = ManNiiPosture();
		this->convertKinectV2JointOrientations2ManNiiPosture(tmpKinectV2JointOrientations, manNiiPosture);

		// Set SIGVerse quaternions
		SimObj *obj = getObj(myname());
		this->setJointQuaternionsForKinect(obj, manNiiPosture);
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

void ManNiiAvatarControllerByKinectV2::setJointQuaternion(SimObj *obj, ManNiiPosture::ManNiiJoint &joint)
{
	obj->setJointQuaternion(ManNiiPosture::manNiiJointTypeStr(joint.jointType).c_str(), joint.quaternion.w, joint.quaternion.x, joint.quaternion.y, joint.quaternion.z);
}

void ManNiiAvatarControllerByKinectV2::setJointQuaternionsForKinect(SimObj *obj, ManNiiPosture &manNiiPosture)
{
	this->setJointQuaternion(obj, manNiiPosture.joint[ManNiiPosture::WAIST_JOINT1]);
	this->setJointQuaternion(obj, manNiiPosture.joint[ManNiiPosture::RARM_JOINT2]);
	this->setJointQuaternion(obj, manNiiPosture.joint[ManNiiPosture::LARM_JOINT2]);
	this->setJointQuaternion(obj, manNiiPosture.joint[ManNiiPosture::RLEG_JOINT2]);
	this->setJointQuaternion(obj, manNiiPosture.joint[ManNiiPosture::LLEG_JOINT2]);
	this->setJointQuaternion(obj, manNiiPosture.joint[ManNiiPosture::RLEG_JOINT4]);
	this->setJointQuaternion(obj, manNiiPosture.joint[ManNiiPosture::LLEG_JOINT4]);
	this->setJointQuaternion(obj, manNiiPosture.joint[ManNiiPosture::RARM_JOINT3]);
	this->setJointQuaternion(obj, manNiiPosture.joint[ManNiiPosture::LARM_JOINT3]);
}



///@brief Read parameter file.
///@return When couldn't read parameter file, return false;
void ManNiiAvatarControllerByKinectV2::readIniFile()
{
	std::ifstream ifs(this->parameterFileName.c_str());

	// Parameter file is "not" exists.
	if (ifs.fail())
	{
		std::cout << "Not exist : " << this->parameterFileName << std::endl;
		std::cout << "Use default parameter." << std::endl;

		this->kinectV2ServiceName    = SERVICE_NAME_KINECT_V2;
		this->kinectV2DeviceType     = DEV_TYPE_KINECT_V2;
		this->kinectV2DeviceUniqueID = DEV_UNIQUE_ID_0;
	}
	// Parameter file is exists.
	else
	{
		try
		{
			std::cout << "Read " << this->parameterFileName << std::endl;
			boost::property_tree::ptree pt;
			boost::property_tree::read_ini(this->parameterFileName, pt);

			this->kinectV2ServiceName    = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME);
			this->kinectV2DeviceType     = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE);
			this->kinectV2DeviceUniqueID = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID);

			this->sensorDataModeStr      = pt.get<std::string>(paramFileKeyKinectV2SensorDataMode);
		}
		catch (boost::exception &ex)
		{
			std::cout << this->parameterFileName << " ERR :" << *boost::diagnostic_information_what(ex) << std::endl;
		}
	}

	std::cout << PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME     << ":" << this->kinectV2ServiceName    << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE      << ":" << this->kinectV2DeviceType     << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID << ":" << this->kinectV2DeviceUniqueID << std::endl;
	std::cout << paramFileKeyKinectV2SensorDataMode          << ":" << this->sensorDataModeStr << std::endl;
}


void ManNiiAvatarControllerByKinectV2::testPrint()
{
	// ファイル出力ストリームの初期化
	std::ofstream ofs("./controller.log", std::ios::app);

	std::vector<std::string>::iterator it;

	it = timeInfoList.begin();

	//printf("%d/%d/%d %d:%d:%d'%d ElbowRight.w,%f\n", systime.wYear, systime.wMonth, systime.wDay, systime.wHour, systime.wMinute, systime.wSecond, systime.wMilliseconds, tmpOrientations[KinectV2SensorData::KinectV2JointType::ElbowRight].Orientation.w);

	while( it != timeInfoList.end() )
	{
		ofs << (*it) << std::endl;
//		std::cout << "testprint:" << (*it) << std::endl;
		it++;
	}
	ofs.flush();
}

extern "C" Controller * createController()
{
	return new ManNiiAvatarControllerByKinectV2;
}


