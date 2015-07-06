/*
 * ManNiiAvatarControllerByOculus.cpp
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#include "ManNiiAvatarControllerByOculus.h"
#include "../../common/device/OculusRiftDK1SensorData.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <cmath>

///@brief Parameter file name.
const std::string ManNiiAvatarControllerByOculus::parameterFileName = "OculusRiftDK1.ini";


///@brief Initialize this controller.
void ManNiiAvatarControllerByOculus::onInit(InitEvent &evt)
{
	readIniFile();

	this->oculusDK1Service = NULL;

	this->defaultHeadJoint0Quaternion[0] = 1.0;
	this->defaultHeadJoint0Quaternion[1] = 0.0;
	this->defaultHeadJoint0Quaternion[2] = 0.0;
	this->defaultHeadJoint0Quaternion[3] = 0.0;

//	this->prevYaw = this->prevPitch = this->prevRoll = 0.0;

	SimObj *my = getObj(myname());
}


///@brief Movement of the robot.
double ManNiiAvatarControllerByOculus::onAction(ActionEvent &evt)
{
	bool oculusDK1Available = checkService(this->oculusDK1ServiceName);

	if (oculusDK1Available && this->oculusDK1Service == NULL)
	{
		this->oculusDK1Service = connectToService(this->oculusDK1ServiceName);
	}
	else if (!oculusDK1Available && this->oculusDK1Service != NULL)
	{
		this->oculusDK1Service = NULL;
	}

	return 1.0;
}

void ManNiiAvatarControllerByOculus::onRecvMsg(RecvMsgEvent &evt)
{
	const std::string allMsg = evt.getMsg();

	//std::cout << allMsg << std::endl;

	OculusRiftDK1SensorData sensorData;
	std::map<std::string, std::vector<std::string> > sensorDataMap = sensorData.decodeSensorData(allMsg);

	if (sensorDataMap.find(MSG_KEY_DEV_TYPE) == sensorDataMap.end()){ return; }

	if(sensorDataMap[MSG_KEY_DEV_TYPE][0]     !=this->oculusDK1DeviceType    ){ return; }
	if(sensorDataMap[MSG_KEY_DEV_UNIQUE_ID][0]!=this->oculusDK1DeviceUniqueID){ return; }

	sensorData.setSensorData(sensorDataMap);

	SigCmn::EulerAngleType eulerAngle;
	eulerAngle.yaw   = sensorData.getYaw();
	eulerAngle.pitch = sensorData.getPitch();
	eulerAngle.roll  = sensorData.getRoll();

	this->convertEulerAngle2ManNiiPosture(eulerAngle);

	SimObj *obj = getObj(myname());
	this->setJointQuaternionsForOculus(obj);
}


void ManNiiAvatarControllerByOculus::setJointQuaternionsForOculus(SimObj *obj)
{
	ManNiiPosture::ManNiiJoint joint = this->posture.joint[ManNiiPosture::HEAD_JOINT0];

	obj->setJointQuaternion(ManNiiPosture::manNiiJointTypeStr(joint.jointType).c_str(), joint.quaternion.w, joint.quaternion.x, joint.quaternion.y, joint.quaternion.z);
}


void ManNiiAvatarControllerByOculus::convertEulerAngle2ManNiiPosture(const SigCmn::EulerAngleType &eulerAngle)
{
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
	dQMultiply1(tmpQ3, this->defaultHeadJoint0Quaternion, tmpQ2);

	Quaternion tmpQ4(tmpQ3[0], tmpQ3[1], tmpQ3[2], tmpQ3[3]);

	this->posture.joint[ManNiiPosture::HEAD_JOINT0].jointType  = ManNiiPosture::HEAD_JOINT0;
	this->posture.joint[ManNiiPosture::HEAD_JOINT0].quaternion = tmpQ4;
}


///@brief Read parameter file.
///@return When couldn't read parameter file, return false;
void ManNiiAvatarControllerByOculus::readIniFile()
{
	std::ifstream ifs(this->parameterFileName.c_str());

	// Parameter file is "not" exists.
	if (ifs.fail())
	{
		std::cout << "Not exist : " << this->parameterFileName << std::endl;
		std::cout << "Use default parameter." << std::endl;

		this->oculusDK1ServiceName    = SERVICE_NAME_OCULUS_DK1;
		this->oculusDK1DeviceType     = DEV_TYPE_OCULUS_DK1;
		this->oculusDK1DeviceUniqueID = DEV_UNIQUE_ID_0;
	}
	// Parameter file is exists.
	else
	{
		try
		{
			std::cout << "Read " << this->parameterFileName << std::endl;
			boost::property_tree::ptree pt;
			boost::property_tree::read_ini(this->parameterFileName, pt);

			this->oculusDK1ServiceName    = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME);
			this->oculusDK1DeviceType     = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE);
			this->oculusDK1DeviceUniqueID = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID);
		}
		catch (boost::exception &ex)
		{
			std::cout << this->parameterFileName << " ERR :" << *boost::diagnostic_information_what(ex) << std::endl;
		}
	}

	std::cout << PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME     << ":" << this->oculusDK1ServiceName    << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE      << ":" << this->oculusDK1DeviceType     << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID << ":" << this->oculusDK1DeviceUniqueID << std::endl;
}

