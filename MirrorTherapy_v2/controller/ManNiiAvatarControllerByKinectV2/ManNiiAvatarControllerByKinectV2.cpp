/*
 * ManNiiAvatarController.cpp
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#include "../../common/device/SensorData.h"
#include "ManNiiAvatarControllerByKinectV2.h"
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

const double ManNiiAvatarControllerByKinectV2::normalization_range = 0.1;


///@brief Initialize this controller.
void ManNiiAvatarControllerByKinectV2::onInit(InitEvent &evt)
{
	readIniFile();

	this->kinectV2Service = NULL;
	SimObj *myself = getObj(myname());

	Vector3d tmpPos;
	myself->getPosition(tmpPos);
	this->iniPos.x = tmpPos.x();
	this->iniPos.y = tmpPos.y();
	this->iniPos.z = tmpPos.z();

	Rotation rot;
	myself->getRotation(rot);
	double qw = rot.qw();
	double qy = rot.qy();
	this->yrot = acos(fabs(qw))*2.0;
	if (qw*qy > 0){ this->yrot = -1.0*this->yrot; }
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

		//開始時の位置情報を保持
		if (this->started == false)
		{
			this->startpos.x = sensorData.rootPosition.x;
			this->startpos.y = sensorData.rootPosition.y;
			this->startpos.z = sensorData.rootPosition.z;
			printf("Start position X : %f ------ Y : %f   ---- Z : %f   ---- end \n", startpos.x, startpos.y, startpos.z);

			this->started = true;
		}

		//開始時の位置からの移動距離を算出
		SigCmn::Vector3 pos;
		pos.x = -(sensorData.rootPosition.x - this->startpos.x);
		pos.y = +(sensorData.rootPosition.y - this->startpos.y);
		pos.z = -(sensorData.rootPosition.z - this->startpos.z);


		// Convert kinect v2 quaternions(orientations) to man-nii posture(sigverse quaternion format).
		ManNiiPosture manNiiPosture;

		switch(sensorData.sensorDataMode)
		{
			case KinectV2SensorData::SensorDataMode::POSITION:
			{
				manNiiPosture = this->convertKinectV2JointPosition2ManNiiPosture(sensorData.jointPositions);
				break;
			}
			case KinectV2SensorData::SensorDataMode::QUATERNION:
			{
				manNiiPosture = this->convertKinectV2JointOrientations2ManNiiPosture(sensorData.jointOrientations);
				break;
			}
		}

		// Set SIGVerse quaternions and positions
		SimObj *obj = getObj(myname());
		this->setPosition(obj, pos);
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

/////@brief control avatar by Positions (sensor data mode = Position)
//void ManNiiAvatarControllerByKinectV2::controlAvatarByPositions(const KinectV2SensorData &sensorData)
//{
////	// Get positions of kinect v2 format, from sensor data of kinect v2.
////	KinectV2SensorData::KinectV2JointPosition tmpKinectV2JointPositions[KinectV2SensorData::KinectV2JointType_Count];
////	sensorData.getKinectV2JointPosition(tmpKinectV2JointPositions);
//
//	// Convert kinect v2 quaternions(orientations) to man-nii posture(sigverse quaternion format).
//	ManNiiPosture manNiiPosture = ManNiiPosture();
//	manNiiPosture = this->convertKinectV2JointPosition2ManNiiPosture(sensorData.jointPositions);
//
//	// Set SIGVerse quaternions
//	SimObj *obj = getObj(myname());
//	this->setPosition(obj, sensorData.rootPosition);
//	this->setJointQuaternionsForKinect(obj, manNiiPosture);
//}
//
/////@brief control avatar by Quaternions (sensor data mode = Quaternion)
//void ManNiiAvatarControllerByKinectV2::controlAvatarByQuaternions(const KinectV2SensorData &sensorData)
//{
////	// Get quaternions(orientations) of kinect v2 format, from sensor data of kinect v2.
////	KinectV2SensorData::KinectV2JointOrientation tmpKinectV2JointOrientations[KinectV2SensorData::KinectV2JointType_Count];
////	sensorData.getKinectV2JointOrientation(tmpKinectV2JointOrientations);
////
//	// Convert kinect v2 quaternions(orientations) to man-nii posture(sigverse quaternion format).
//	ManNiiPosture manNiiPosture = ManNiiPosture();
//	manNiiPosture = this->convertKinectV2JointOrientations2ManNiiPosture(sensorData.jointOrientations);
//
//	// Set SIGVerse quaternions
//	SimObj *obj = getObj(myname());
//	this->setPosition(obj, sensorData.rootPosition);
//	this->setJointQuaternionsForKinect(obj, manNiiPosture);
//}

void ManNiiAvatarControllerByKinectV2::setPosition(SimObj *obj, const SigCmn::Vector3 &pos)
{
	double x = this->scaleRatio/100.0 * pos.x;
	double y = this->scaleRatio/100.0 * pos.y;
	double z = this->scaleRatio/100.0 * pos.z;

	double gx = std::cos(this->yrot)*x - std::sin(this->yrot)*z;
	double gz = std::sin(this->yrot)*x + std::cos(this->yrot)*z;

	obj->setPosition(this->iniPos.x+gx,this->iniPos.y+y,this->iniPos.z+gz);
}

void ManNiiAvatarControllerByKinectV2::setJointQuaternion(SimObj *obj, const ManNiiPosture::ManNiiJoint &joint)
{
	if(joint.jointType != ManNiiPosture::ManNiiJointType::ROOT_JOINT0)
	{
		double angle = acos(joint.quaternion.w)*2.0;
		double tmp = sin(angle/2.0);
		double vx = joint.quaternion.x/tmp;
		double vy = joint.quaternion.y/tmp;
		double vz = joint.quaternion.z/tmp;
		double len = sqrt(vx*vx + vy*vy + vz*vz);
		if (len < (1.0-this->normalization_range) || (1+this->normalization_range) < len){ return; }
	}

	obj->setJointQuaternion(ManNiiPosture::manNiiJointTypeStr(joint.jointType).c_str(), joint.quaternion.w, joint.quaternion.x, joint.quaternion.y, joint.quaternion.z);
}

void ManNiiAvatarControllerByKinectV2::setJointQuaternionsForKinect(SimObj *obj, const ManNiiPosture &manNiiPosture)
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
	std::ifstream ifs(ManNiiAvatarControllerByKinectV2::parameterFileName.c_str());

	// Parameter file is "not" exists.
	if (ifs.fail())
	{
		std::cout << "Not exist : " << ManNiiAvatarControllerByKinectV2::parameterFileName << std::endl;
		std::cout << "Use default parameter." << std::endl;

		this->kinectV2ServiceName    = SERVICE_NAME_KINECT_V2;
		this->kinectV2DeviceType     = DEV_TYPE_KINECT_V2;
		this->kinectV2DeviceUniqueID = DEV_UNIQUE_ID_0;

		this->sensorDataModeStr      = ManNiiAvatarControllerByKinectV2::paramFileValKinectV2SensorDataModeDefault;
		this->scaleRatio             = ManNiiAvatarControllerByKinectV2::paramFileValKinectV2ScaleRatioDefault;
	}
	// Parameter file is exists.
	else
	{
		try
		{
			std::cout << "Read " << ManNiiAvatarControllerByKinectV2::parameterFileName << std::endl;
			boost::property_tree::ptree pt;
			boost::property_tree::read_ini(ManNiiAvatarControllerByKinectV2::parameterFileName, pt);

			this->kinectV2ServiceName    = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME);
			this->kinectV2DeviceType     = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE);
			this->kinectV2DeviceUniqueID = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID);

			this->sensorDataModeStr      = pt.get<std::string>(ManNiiAvatarControllerByKinectV2::paramFileKeyKinectV2SensorDataMode);
			this->scaleRatio             = pt.get<double>     (ManNiiAvatarControllerByKinectV2::paramFileKeyKinectV2ScaleRatio);
		}
		catch (boost::exception &ex)
		{
			std::cout << ManNiiAvatarControllerByKinectV2::parameterFileName << " ERR :" << *boost::diagnostic_information_what(ex) << std::endl;
		}
	}

	std::cout << PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME     << ":" << this->kinectV2ServiceName    << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE      << ":" << this->kinectV2DeviceType     << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID << ":" << this->kinectV2DeviceUniqueID << std::endl;
	std::cout << ManNiiAvatarControllerByKinectV2::paramFileKeyKinectV2SensorDataMode << ":" << this->sensorDataModeStr << std::endl;
}


extern "C" Controller * createController()
{
	return new ManNiiAvatarControllerByKinectV2;
}


