/*
 * OpenHouse2015Controller.cpp
 *
 *  Created on: 2015/06/08
 *      Author: Yamada@tome
 */

#include <sigverse/common/device/SensorData.h>
#include <sigverse/controller/OpenHouse2015Controller/OpenHouse2015Controller.h>


#include <sys/time.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <math.h>


const std::string OpenHouse2015Controller::parameterFileName = "OpenHouse2015.ini";

const std::string OpenHouse2015Controller::paramFileKeyKinectV2ServiceName    = "KinectV2.service_name";
const std::string OpenHouse2015Controller::paramFileKeyKinectV2Devicetype     = "KinectV2.device_type";
const std::string OpenHouse2015Controller::paramFileKeyKinectV2DeviceUniqueID = "KinectV2.device_unique_id";

const std::string OpenHouse2015Controller::paramFileKeyKinectV2SensorDataMode = "KinectV2.sensor_data_mode";
const std::string OpenHouse2015Controller::paramFileKeyKinectV2ScaleRatio     = "KinectV2.scale_ratio";

const std::string OpenHouse2015Controller::paramFileKeyOculusDK1ServiceName   = "OculusDK1.service_name";
const std::string OpenHouse2015Controller::paramFileKeyOculusDK1Devicetype    = "OculusDK1.device_type";
const std::string OpenHouse2015Controller::paramFileKeyOculusDK1DeviceUniqueID= "OculusDK1.device_unique_id";

const std::string OpenHouse2015Controller::paramFileValKinectV2SensorDataModeDefault = "QUATERNION";
const double      OpenHouse2015Controller::paramFileValKinectV2ScaleRatioDefault     = 10000.0;

const std::string OpenHouse2015Controller::paramFileKeyChangeAvatarGUIServiceName        = "ChangeAvatarGUI.service_name";
const std::string OpenHouse2015Controller::paramFileValChangeAvatarGUIServiceNameDefault = "SVC_CHANGE_AVATAR_GUI";

const std::string OpenHouse2015Controller::msgKeyAvatar  = "AVATAR";
const std::string OpenHouse2015Controller::msgKeyReverse = "REVERSE";
const std::string OpenHouse2015Controller::msgKeyDelay   = "DELAY";

const double OpenHouse2015Controller::defaultDelayTime      = 1000.0;
const int    OpenHouse2015Controller::timeSeriesBufferSize  = 500;

const std::string OpenHouse2015Controller::reverseModes[ReverseMode_Count] = { "RIGHTHAND", "LEFTHAND", "NOREVERSE" };

const std::string OpenHouse2015Controller::headName             = "man_nii_head";
const std::string OpenHouse2015Controller::mirrorTherapyManName = "mirror_therapy_man";
const std::string OpenHouse2015Controller::longArmManName       = "right_arm_long_man";
const std::string OpenHouse2015Controller::shortArmManName      = "right_arm_short_man";
const std::string OpenHouse2015Controller::robotArmManName      = "right_arm_robot_man";


///@brief Initialize this controller.
void OpenHouse2015Controller::onInit(InitEvent &evt)
{
	readIniFileAndInitialize();

	SimObj *myself = getObj(myname());

	this->kinectV2DeviceManager.initPositionAndRotation(myself);


	// For Mirror therapy initialize.
	this->reverseMode = reverseModes[RIGHTHAND]; // Set reverse mode.

	// Set delay variables.
	gettimeofday(&this->initTimeVal, NULL);
	this->frameNumber = 0;
	this->targetDelayTime = defaultDelayTime;
	this->pastTimeAndPostures = std::vector<TimeAndPosture>(timeSeriesBufferSize);

	if(std::string(myname())==headName)
	{
		this->isHead = true;
	}
	//Set default avatar
	this->bodyAvatarName = this->mirrorTherapyManName;
}

///@brief Movement of the robot.
double OpenHouse2015Controller::onAction(ActionEvent &evt)
{
	if(this->isHead)
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

		bool oculusDK1Available = checkService(this->oculusDK1DeviceManager.serviceName);

		if (oculusDK1Available && this->oculusDK1DeviceManager.service == NULL)
		{
			this->oculusDK1DeviceManager.service = connectToService(this->oculusDK1DeviceManager.serviceName);
		}
		else if (!oculusDK1Available && this->oculusDK1DeviceManager.service != NULL)
		{
			this->oculusDK1DeviceManager.service = NULL;
		}

		bool guiAvailable = checkService(this->guiServiceName);

		if (guiAvailable && this->guiService == NULL)
		{
			this->guiService = connectToService(this->guiServiceName);
		}
		else if (!guiAvailable && this->guiService != NULL)
		{
			this->guiService = NULL;
		}
	}

	return 1.0;
}


///@brief Message from device.
void OpenHouse2015Controller::onRecvMsg(RecvMsgEvent &evt)
{
	try
	{
		if(!this->isHead && std::string(evt.getSender())!=headName){ return; }

		// Receive message.
		const std::string allMsg = evt.getMsg();

//		std::cout << "msg=" << allMsg << std::endl;

		// Get device type from message.
		std::map<std::string, std::vector<std::string> > sensorDataMap = SensorData::decodeSensorData(allMsg);

		/*
		 * Device
		 */
		if (sensorDataMap.find(MSG_KEY_DEV_TYPE) != sensorDataMap.end())
		{
			if(this->isHead)
			{
				sendMsg(this->mirrorTherapyManName, allMsg);
				sendMsg(this->longArmManName,       allMsg);
				sendMsg(this->shortArmManName,      allMsg);
				sendMsg(this->robotArmManName,      allMsg);
//				sendMsg(this->bodyAvatarName, allMsg);
			}

			std::string deviceTypeValue = sensorDataMap[MSG_KEY_DEV_TYPE][0];
			std::string deviceUniqueId  = sensorDataMap[MSG_KEY_DEV_UNIQUE_ID][0];

			/*
			 * Kinect V2
			 */
			if (deviceTypeValue == this->kinectV2DeviceManager.deviceType && deviceUniqueId == this->kinectV2DeviceManager.deviceUniqueID)
			{
				if(this->isHead && !this->kinectV2DeviceManager.started)
				{
					sendMsg(this->mirrorTherapyManName, allMsg);
					sendMsg(this->longArmManName,       allMsg);
					sendMsg(this->shortArmManName,      allMsg);
					sendMsg(this->robotArmManName,      allMsg);
				}

				// Decode message to sensor data of kinect v2.
				KinectV2SensorData sensorData;
				sensorData.setSensorData(sensorDataMap);

				// Convert kinect v2 quaternions(orientations) to man-nii posture(sigverse quaternion format).
				ManNiiPosture posture = KinectV2DeviceManager::convertSensorData2ManNiiPosture(sensorData);

				if (this->usingOculus)
				{
					posture.joint[ManNiiPosture::HEAD_JOINT0].quaternion.setQuaternion(0.0, 0.0, 0.0, 0.0);
					posture.joint[ManNiiPosture::HEAD_JOINT1].quaternion.setQuaternion(0.0, 0.0, 0.0, 0.0);
				}

				if(std::string(myname())==mirrorTherapyManName)
				{
					// Store current posture and time stamp.
					int currentIndex = this->storeCurrentPosture(posture);

					// Search nearest neighbor of target delay time in stored time stamps.
					//   last 500 postures is in this->pastPostures.
					//   find nearest index.
					ManNiiPosture delayedPosture = this->getDelayedPosture(currentIndex);

					this->invertPosture(posture, delayedPosture);
				}

				// Set SIGVerse quaternions and positions
				SimObj *obj = getObj(myname());
				this->kinectV2DeviceManager.setRootPosition(obj, sensorData.rootPosition);
				KinectV2DeviceManager::setJointQuaternions2ManNii(obj, posture, sensorData);
			}
			/*
			 * Oculus DK1
			 */
			else if (deviceTypeValue == this->oculusDK1DeviceManager.deviceType && deviceUniqueId ==  this->oculusDK1DeviceManager.deviceUniqueID)
			{
				if(!this->usingOculus){ this->usingOculus = true; }

				OculusRiftDK1SensorData sensorData;
				sensorData.setSensorData(sensorDataMap);

				ManNiiPosture posture = OculusDK1DeviceManager::convertEulerAngle2ManNiiPosture(sensorData.getEulerAngle());

				SimObj *obj = getObj(myname());
				OculusDK1DeviceManager::setJointQuaternions2ManNii(obj, posture);
			}
		}
		// Not a device.(Message from Change Avatar GUI)
		else
		{
			if(this->isHead)
			{
				sendMsg(this->mirrorTherapyManName, allMsg);
				sendMsg(this->longArmManName,       allMsg);
				sendMsg(this->shortArmManName,      allMsg);
				sendMsg(this->robotArmManName,      allMsg);
			}

			this->changeAvatar(sensorDataMap);

			if(std::string(myname())==mirrorTherapyManName)
			{
				this->setReverseModeAndDelayTime(sensorDataMap);
			}
		}
	}
	catch(...)
	{
		LOG_MSG(("SIGViewer message is invalid."));
	}
}


const double OpenHouse2015Controller::generateCurrentTimeStamp()
{
	// Get current timestamp.
	struct timeval currentTimeVal;
	gettimeofday(&currentTimeVal, NULL);
	const double currentTimeStamp = (currentTimeVal.tv_sec - this->initTimeVal.tv_sec) * 1000.0 + (currentTimeVal.tv_usec - this->initTimeVal.tv_usec) / 1000.0;
	return currentTimeStamp;
}


int OpenHouse2015Controller::storeCurrentPosture(const ManNiiPosture &posture)
{
	TimeAndPosture current;
	current.timeStamp = this->generateCurrentTimeStamp();

	for (int i = 0; i < ManNiiPosture::ManNiiJointType_Count; i++)
	{
		current.posture.joint[i] = posture.joint[i];
	}

	int index = this->frameNumber % timeSeriesBufferSize;
	this->pastTimeAndPostures[index] = current;

	this->frameNumber++;

	return index;
}


ManNiiPosture OpenHouse2015Controller::getDelayedPosture(const int currentIndex)
{
	// If targetDelayTime <= 0, don't do anything.
	if (this->targetDelayTime <= 0.0){ return this->pastTimeAndPostures[currentIndex].posture; }

	// If elapsed time < targetDelayTime, don't do anything.
	if(this->pastTimeAndPostures[currentIndex].timeStamp < this->targetDelayTime){ return this->pastTimeAndPostures[currentIndex].posture; }


	const double idealPastTime = this->pastTimeAndPostures[currentIndex].timeStamp - this->targetDelayTime;

	int delayedPostureIndex = 0;
	double nearestTimeDistance = DBL_MAX;

	// Search nearest index to the delay time.
	for (int i = timeSeriesBufferSize-1; i >= 0; i--)
	{
		double tmpTimeDistance = abs(idealPastTime - this->pastTimeAndPostures[i].timeStamp);

		if (tmpTimeDistance < nearestTimeDistance)
		{
			nearestTimeDistance = tmpTimeDistance;
			delayedPostureIndex = i;
		}
	}

	return this->pastTimeAndPostures[delayedPostureIndex].posture;
}


void OpenHouse2015Controller::invertPosture(ManNiiPosture &posture, const ManNiiPosture &delayedPosture)
{
	// left hand motion affects right hand.
	if (this->reverseMode == reverseModes[RIGHTHAND])
	{
		// Get original quaternions.
		double w2, x2, y2, z2, w3, x3, y3, z3;
		delayedPosture.joint[ManNiiPosture::LARM_JOINT2].quaternion.getQuaternion(w2, x2, y2, z2);
		delayedPosture.joint[ManNiiPosture::LARM_JOINT3].quaternion.getQuaternion(w3, x3, y3, z3);

		// Set reverse quaternions.
		posture.joint[ManNiiPosture::RARM_JOINT2].quaternion.setQuaternion(w2, x2, -y2, -z2);
		posture.joint[ManNiiPosture::RARM_JOINT3].quaternion.setQuaternion(w3, x3, -y3, -z3);
	}
	// right hand motion affects left hand.
	else if (this->reverseMode == reverseModes[LEFTHAND])
	{
		// Get original quaternions.
		double w2, x2, y2, z2, w3, x3, y3, z3;
		delayedPosture.joint[ManNiiPosture::RARM_JOINT2].quaternion.getQuaternion(w2, x2, y2, z2);
		delayedPosture.joint[ManNiiPosture::RARM_JOINT3].quaternion.getQuaternion(w3, x3, y3, z3);

		// Set reverse quaternions.
		posture.joint[ManNiiPosture::LARM_JOINT2].quaternion.setQuaternion(w2, x2, -y2, -z2);
		posture.joint[ManNiiPosture::LARM_JOINT3].quaternion.setQuaternion(w3, x3, -y3, -z3);
	}
}


void OpenHouse2015Controller::changeAvatar(const std::map<std::string, std::vector<std::string> > &map)
{
	if (map.find(msgKeyAvatar) != map.end())
	{
		std::map<std::string, std::vector<std::string> >::const_iterator it = map.find(msgKeyAvatar);
		this->bodyAvatarName = it->second[0];

		if(this->isHead)
		{
			std::cout << "Change Avatar:" << this->bodyAvatarName << std::endl;
		}
		else
		{
			if(std::string(myname())==this->bodyAvatarName){ this->kinectV2DeviceManager.iniPos.z = 0.0; }
			else                                           { this->kinectV2DeviceManager.iniPos.z = 1000.0; }

			SigCmn::Vector3 zeroPos;
			zeroPos.x = 0.0;
			zeroPos.y = 0.0;
			zeroPos.z = 0.0;

			SimObj *my = getObj(myname());
//			this->kinectV2DeviceManager.setRootPosition(my, this->kinectV2DeviceManager.rootPos);
			this->kinectV2DeviceManager.setRootPosition(my, zeroPos);
		}
	}
}

void OpenHouse2015Controller::setReverseModeAndDelayTime(const std::map<std::string, std::vector<std::string> > &map)
{
	if (map.find(msgKeyReverse) != map.end())
	{
		std::map<std::string, std::vector<std::string> >::const_iterator it = map.find(msgKeyReverse);
		this->reverseMode = it->second[0];

		std::cout << "Set reverse mode:" << this->reverseMode << std::endl;
	}

	if (map.find(msgKeyDelay) != map.end())
	{
		std::map<std::string, std::vector<std::string> >::const_iterator it = map.find(msgKeyDelay);
		this->targetDelayTime = atof(it->second[0].c_str());

		std::cout << "Set target delay time:" << this->targetDelayTime << std::endl;
	}
}



///@brief Read parameter file.
///@return When couldn't read parameter file, return false;
void OpenHouse2015Controller::readIniFileAndInitialize()
{
	std::ifstream ifs(parameterFileName.c_str());

	std::string kinectV2ServiceName;
	std::string kinectV2DeviceType;
	std::string kinectV2DeviceUniqueID;
	double      scaleRatio;
	std::string sensorDataModeStr;

	std::string oculusDK1ServiceName;
	std::string oculusDK1DeviceType;
	std::string oculusDK1DeviceUniqueID;

	// Parameter file is "not" exists.
	if (ifs.fail())
	{
		std::cout << "Not exist : " << parameterFileName << std::endl;
		std::cout << "Use default parameter." << std::endl;

		kinectV2ServiceName    = SERVICE_NAME_KINECT_V2;
		kinectV2DeviceType     = DEV_TYPE_KINECT_V2;
		kinectV2DeviceUniqueID = DEV_UNIQUE_ID_0;

		sensorDataModeStr = paramFileValKinectV2SensorDataModeDefault;
		scaleRatio        = paramFileValKinectV2ScaleRatioDefault;

		oculusDK1ServiceName    = SERVICE_NAME_OCULUS_DK1;
		oculusDK1DeviceType     = DEV_TYPE_OCULUS_DK1;
		oculusDK1DeviceUniqueID = DEV_UNIQUE_ID_0;

		this->guiServiceName = paramFileValChangeAvatarGUIServiceNameDefault;
	}
	// Parameter file is exists.
	else
	{
		try
		{
			std::cout << "Read " << parameterFileName << std::endl;
			boost::property_tree::ptree pt;
			boost::property_tree::read_ini(parameterFileName, pt);

			kinectV2ServiceName    = pt.get<std::string>(paramFileKeyKinectV2ServiceName);
			kinectV2DeviceType     = pt.get<std::string>(paramFileKeyKinectV2Devicetype);
			kinectV2DeviceUniqueID = pt.get<std::string>(paramFileKeyKinectV2DeviceUniqueID);

			sensorDataModeStr = pt.get<std::string>(paramFileKeyKinectV2SensorDataMode);
			scaleRatio        = pt.get<double>     (paramFileKeyKinectV2ScaleRatio);

			oculusDK1ServiceName    = pt.get<std::string>(paramFileKeyOculusDK1ServiceName);
			oculusDK1DeviceType     = pt.get<std::string>(paramFileKeyOculusDK1Devicetype);
			oculusDK1DeviceUniqueID = pt.get<std::string>(paramFileKeyOculusDK1DeviceUniqueID);

			this->guiServiceName = pt.get<std::string>(paramFileKeyChangeAvatarGUIServiceName);
		}
		catch (boost::exception &ex)
		{
			std::cout << parameterFileName << " ERR :" << *boost::diagnostic_information_what(ex) << std::endl;
		}
	}

	std::cout << paramFileKeyKinectV2ServiceName    << ":" << kinectV2ServiceName    << std::endl;
	std::cout << paramFileKeyKinectV2Devicetype     << ":" << kinectV2DeviceType     << std::endl;
	std::cout << paramFileKeyKinectV2DeviceUniqueID << ":" << kinectV2DeviceUniqueID << std::endl;

	std::cout << paramFileKeyKinectV2SensorDataMode << ":" << sensorDataModeStr << std::endl;
	std::cout << paramFileKeyKinectV2ScaleRatio     << ":" << scaleRatio << std::endl;

	std::cout << paramFileKeyOculusDK1ServiceName    << ":" << oculusDK1ServiceName    << std::endl;
	std::cout << paramFileKeyOculusDK1Devicetype     << ":" << oculusDK1DeviceType     << std::endl;
	std::cout << paramFileKeyOculusDK1DeviceUniqueID << ":" << oculusDK1DeviceUniqueID << std::endl;

	std::cout << paramFileKeyChangeAvatarGUIServiceName << ":" << this->guiServiceName << std::endl;


	this->kinectV2DeviceManager = KinectV2DeviceManager(kinectV2ServiceName, kinectV2DeviceType, kinectV2DeviceUniqueID, scaleRatio);

	// Set sensor data mode.
	KinectV2SensorData::setSensorDataMode(sensorDataModeStr);

	this->oculusDK1DeviceManager = OculusDK1DeviceManager(oculusDK1ServiceName, oculusDK1DeviceType, oculusDK1DeviceUniqueID);
}



extern "C" Controller * createController()
{
	return new OpenHouse2015Controller;
}

