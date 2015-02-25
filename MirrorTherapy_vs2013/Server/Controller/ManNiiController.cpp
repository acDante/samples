/*
 * ManNiiController.cpp
 *
 *  Created on: 2015/02/12
 *      Author: sigverse
 */

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>
#include <sigverse/SimObj.h>
#include <sigverse/modelerr.h>
#include <sigverse/Logger.h>
#include <sigverse/Vector3d.h>
#include <sigverse/ViewImage.h>
#include <math.h>
#include <map> // For MapQuaternion class.
#include <sys/time.h> // For timestamp.
#include <limits.h> // For DBL_MAX;
#include "AvatarController.h"

class Quaternion
{
private:
	double mW;
	double mX;
	double mY;
	double mZ;

public:
	Quaternion();
	Quaternion(const double w, const double x, const double y, const double z);
	void getQuaternion(double& w, double &x, double &y, double &z);
	void setQuaternion(const double w, const double x, const double y, const double z);
	double w() const { return mW; };
	double x() const { return mX; };
	double y() const { return mY; };
	double z() const { return mZ; };
};

Quaternion::Quaternion()
{
	mW = 0;
	mX = 0;
	mY = 0;
	mZ = 0;
}
Quaternion::Quaternion(const double w, const double x, const double y, const double z)
{
	mW = w;
	mX = x;
	mY = y;
	mZ = z;
}
void Quaternion::getQuaternion(double &w, double &x, double &y, double &z)
{
	w = mW;
	x = mX;
	y = mY;
	z = mZ;
}
void Quaternion::setQuaternion(const double w, const double x, const double y, const double z)
{
	mW = w;
	mX = x;
	mY = y;
	mZ = z;
}

typedef std::map<std::string, Quaternion> QMap;

class QMapStamp
{
public:
	double timeStamp;
	QMap qmap;
};


///@brief Avatar class to control the man-nii-v2 model.
class ManNiiController: public AvatarController
{
	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	///@brief Split the message.
	static void splitString(const std::string &inputString, const std::string &delimiter, std::vector<std::string> &output);

	///@brief Make the Quaternion-map by received message.
	void makeQMapFromReceivedMessage(const std::string &message, std::string &firstOfMessage, QMap &qmap);

	///@brief Store the Quaternion-map to the quaternion record.
	void storeQuaternion(const std::string &firstOfMessage, const QMap &qmap, const double currentTimeStamp);

	///@brief Play back.
	void playBack(const std::string &firstOfMessage, QMap &qmap, const double currentTimeStamp);

	///@brief Generate behavior(whole body quaternion) depending on the modes.
	void generateQuaternionDependOnModes(const std::string &firstOfMessage, const double currentTimeStamp, QMap &qmap);

	void reverseArm(const std::string &firstOfMessage, QMap &qmap);
	///@brief Play back.
	//void playBackForOculus(QMap &qmap, const double currentTimeStamp);

	///@brief Generate time stamp.
	const double generateCurrentTimeStamp();

	///@brief Get quaternion map from
	void setQMapFromJointNameAndQuaternionForKinect(QMap &qmap, const std::vector<std::string> &jointNameAndQuaternionsVector);

	void setQMapFromJointNameAndQuaternionForOculus(QMap &qmap, const std::string &message);

private:

	///@brief Pointer to the Kinect service.
	BaseService *kinectService;

	///@brief Pointer to the Oculus service.
	BaseService *oculusService;

	///@brief Mode of robot (reverse right hand, reverse left hand, etc.).
	std::string reverseMode;

	///@brief Flag of delay.
	bool delayFlag;

	///@brief Delay time[ms] of robot reaction.
	double inputDelayTime;

	///@brief Flag of use Kinect.
	bool usingKinect;

	///@brief Flag of use Oculus.
	bool usingOculus;
	bool chk_srv;

	///@brief Mode of received message.
	//std::string kindOfMessage;

	///@brief Size of record to generate delay motion.
	static const int recordBufferSize = 500;

	///@brief Time stamp on initialize.
	struct timeval initTimeVal;

	///@brief Total frames from the process started.
	unsigned long frameNumber;

	///@brief Record of Quaternion-map and its timestamp.
	std::vector<QMapStamp> quaternionRecordForKinect;

	///@brief Record of Quaternion-map and its timestamp.
	std::vector<QMapStamp> quaternionRecordForOculus;

	///@brief Valuables for Oculus service.
	double m_posx, m_posy, m_posz;
	double m_yrot;
	double m_range;
	int m_maxsize;
	double m_qw, m_qy, m_qx, m_qz;
	double pyaw, ppitch, proll;

};

void ManNiiController::onInit(InitEvent &evt)
{
	//this->reverseMode = "NOREVERSE";
	this->reverseMode = "REVERSE:RIGHTHAND";
	this->delayFlag = true;
	this->inputDelayTime = 1000.0;
	//mKindOfMessage = "NO RECEIVE";
	this->quaternionRecordForKinect = std::vector<QMapStamp>(recordBufferSize);
	this->quaternionRecordForOculus = std::vector<QMapStamp>(recordBufferSize);
	gettimeofday(&initTimeVal, NULL);
	this->frameNumber = 0;
	this->usingOculus = false;

	// For oculus service.
	SimObj *myself = getObj(myname());
    m_posx = myself->x();
	m_posy = myself->y();
	m_posz = myself->z();
	m_range = 0.1;
	m_maxsize = 15;
	double qw = myself->qw();
	double qy = myself->qy();
	m_yrot = acos(fabs(qw))*2;
	if (qw*qy > 0)
		m_yrot = -1*m_yrot;

	m_qw = 1.0;
	m_qx = 0.0;
	m_qy = 0.0;
	m_qz = 0.0;

	pyaw = ppitch = proll = 0.0;
}

double ManNiiController::onAction(ActionEvent &evt)
{

	if (chk_srv==NULL) {
		bool ch_oculus = checkService("SIGORS");

		if (ch_oculus) {
			oculusService = connectToService("SIGORS");
		}
	}
	  //regular check of availability of the service
	  bool available = checkService("SIGKINECT");

	  //Service available
	  if (available && this->kinectService == NULL) {
	    //connect to the service
	    this->kinectService = connectToService("SIGKINECT");
	    }
	    //Service unavailable
	    else if (!available && this->kinectService != NULL)
	    {
	      this->kinectService = NULL;
	    }
	return 0.2;
}

void ManNiiController::onRecvMsg(RecvMsgEvent &evt)
{
	try {

		// Get myself.
		SimObj* my = getObj(myname());

		// Get message from service provider.
		const std::string message = evt.getMsg();
//		fprintf(stdout, "\r%s", message.c_str());
//		fflush(stdout);

		// Get current timestamp.
		const double currentTimeStamp = this->generateCurrentTimeStamp();

		// Get quaternion map from received message.
		QMap qmap;
		std::string firstOfMessage;
		makeQMapFromReceivedMessage(message, firstOfMessage, qmap);

		// Make delay motion.
		if (delayFlag)
		{
			this->playBack(firstOfMessage, qmap, currentTimeStamp);
		}

		// Get mode of motion from received message.
		if (firstOfMessage == "DELAY")
		{
			// Divide the message by delimiter to vector of "JONT-NAME" and "QUATERNION".
			const std::string delimiterOfColon = ":";
			std::vector<std::string> tmpMessage;
			splitString(message, delimiterOfColon, tmpMessage);
			this->inputDelayTime = atof(tmpMessage[1].c_str());
			return;
		}
		else if (firstOfMessage == "REVERSE:RIGHTHAND")
		{
			this->reverseMode = "REVERSE:RIGHTHAND";
			return;
		}
		else if (firstOfMessage == "NOREVERSE")
		{
			this->reverseMode = "NOREVERSE";
			return;
		}

		// Set joint quaternion.
		std::map<std::string, Quaternion>::iterator it = qmap.begin();
		for (; it != qmap.end(); it++)
		{
			const std::string jointName = (*it).first;
			if (jointName == "RARM_JOINT5") continue;
			const Quaternion q = (*it).second;
			my->setJointQuaternion(jointName.c_str(), q.w(), q.x(), q.y(), q.z());
		}

		// Increment the frame number.
		this->frameNumber++;
	}
	catch (SimObj::NoAttributeException &err)
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
}

void ManNiiController::splitString(const std::string &inputString, const std::string &delimiter, std::vector<std::string>& output)
{
	std::string::size_type index = inputString.find_first_of(delimiter);
	if (index != std::string::npos)
	{
		output.push_back(inputString.substr(0, index));
		splitString(inputString.substr(index + 1), delimiter, output);
	} else
	{
		output.push_back(inputString);
	}
}

void ManNiiController::makeQMapFromReceivedMessage(const std::string &message, std::string &firstOfMessage, QMap &qmap)
{
	// Define the delimiter of received message. Basically, it allows only " ".
	const std::string delimiterOfReceivedMessage = " ";

	// Divide the message by delimiter to vector of "JOINT-NAME:QUATERNION".
	std::vector<std::string> jointNameAndQuaternionsVector;
	ManNiiController::splitString(message, delimiterOfReceivedMessage, jointNameAndQuaternionsVector);

	// Define the target joints and quaternions.

	// When it couldn't be divided, throw exception.
	if ((int)jointNameAndQuaternionsVector.size() < 1)
	{
		throw SimObj::Exception();
	}
	// When first word of received message is "KINECT_DATA", start the following process.
	else {

		firstOfMessage = jointNameAndQuaternionsVector[0];

		if (firstOfMessage == "KINECT_DATA")
		{
			// Get quaternion map from kinect data.
			this->setQMapFromJointNameAndQuaternionForKinect(qmap, jointNameAndQuaternionsVector);
		}
		else if (firstOfMessage == "ORS_DATA")
		{
			this->setQMapFromJointNameAndQuaternionForOculus(qmap, message);
		}
		else if (firstOfMessage == "DELAY")
		{


		}
	}
}

void ManNiiController::storeQuaternion(const std::string &firstOfMessage, const QMap &qmap, const double currentTimeStamp)
{
	// Store the quaternion before delay.
	// Make index in range of record buffer size.
	const unsigned long index = this->frameNumber % this->recordBufferSize;
	QMapStamp tmpQmapstamp;
	tmpQmapstamp.timeStamp = currentTimeStamp;
	tmpQmapstamp.qmap = qmap;
	if (firstOfMessage == "ORS_DATA")
	{
		this->quaternionRecordForOculus[index] = tmpQmapstamp;
	}
	else if (firstOfMessage == "KINECT_DATA")
	{
		this->quaternionRecordForKinect[index] = tmpQmapstamp;
	}
}

void ManNiiController::playBack(const std::string &firstOfMessage, QMap &qmap, const double currentTimeStamp)
{
	// Store the quaternion before delay process.
	this->storeQuaternion(firstOfMessage, qmap, currentTimeStamp);

	// Get past quaternions.
	if (currentTimeStamp > this->inputDelayTime)
	{
		// Find the body quaternion that has the nearest timestamp from record of quaternions.
		// 指定した秒数だけさかのぼった時刻（に最も近い時刻）に取得したクォータニオンをみつける。
		const double idealPastTime = currentTimeStamp - (this->inputDelayTime);

		double nearestTimeDistance = DBL_MAX;
		int nearestIndex = 0;
		for (int i = recordBufferSize - 1; i >= 0; i--)
		{
			double tmpTimeDistance = abs(idealPastTime - this->quaternionRecordForKinect[i].timeStamp);
			if (firstOfMessage == "ORS_DATA")
			{
				tmpTimeDistance = abs(idealPastTime - this->quaternionRecordForOculus[i].timeStamp);
			}

			if (tmpTimeDistance < nearestTimeDistance)
			{
				nearestTimeDistance = tmpTimeDistance;
				nearestIndex = i;
			}
		}

		if (firstOfMessage == "KINECT_DATA")
		{
			//qmap = this->quaternionRecordForKinect[nearestIndex].qmap;

			// 本来は多分ここに書くべきではないと思うけど苦肉の策として
			if (reverseMode == "REVERSE:RIGHTHAND")
			{
				// Reverse past right hand.
				QMap tmpQmap = this->quaternionRecordForKinect[nearestIndex].qmap;
				double w,x,y,z;
				tmpQmap["LARM_JOINT2"].getQuaternion(w, x, y, z);
				qmap["RARM_JOINT2"].setQuaternion(w, -x, -y, -z);
				tmpQmap["LARM_JOINT3"].getQuaternion(w, x, y, z);
				qmap["RARM_JOINT3"].setQuaternion(w, -x, -y, -z);
				tmpQmap["LARM_JOINT5"].getQuaternion(w, x, y, z);
				qmap["RARM_JOINT5"].setQuaternion(w, -x, -y, -z);
			}
			else if (reverseMode == "NOREVERSE")
			{
				// Past body information.
				qmap = this->quaternionRecordForKinect[nearestIndex].qmap;
			}

		}
		else if (firstOfMessage == "ORS_DATA")
		{
			qmap = this->quaternionRecordForOculus[nearestIndex].qmap;
		}
	}
}

void ManNiiController::generateQuaternionDependOnModes(const std::string &firstOfMessage, const double currentTimeStamp, QMap &qmap)
{
	// Store the quaternion before delay process.
	this->storeQuaternion(firstOfMessage, qmap, currentTimeStamp);

	// Get past quaternions.
	if (currentTimeStamp > this->inputDelayTime)
	{
		// Find the body quaternion that has the nearest timestamp from record of quaternions.
		// 指定した秒数だけさかのぼった時刻（に最も近い時刻）に取得したクォータニオンをみつける。
		const double idealPastTime = currentTimeStamp - (this->inputDelayTime);

		double nearestTimeDistance = DBL_MAX;
		int nearestIndex = 0;
		for (int i = recordBufferSize - 1; i >= 0; i--)
		{
			double tmpTimeDistance = abs(idealPastTime - this->quaternionRecordForKinect[i].timeStamp);
			if (firstOfMessage == "ORS_DATA")
			{
				tmpTimeDistance = abs(idealPastTime - this->quaternionRecordForOculus[i].timeStamp);
			}

			if (tmpTimeDistance < nearestTimeDistance)
			{
				nearestTimeDistance = tmpTimeDistance;
				nearestIndex = i;
			}
		}

		if (firstOfMessage == "KINECT_DATA")
		{
			//qmap = this->quaternionRecordForKinect[nearestIndex].qmap;

			// 本来は多分ここに書くべきではないと思うけど苦肉の策として
			if (reverseMode == "REVERSE:RIGHTHAND")
			{
				// Reverse past right hand.
				QMap tmpQmap = this->quaternionRecordForKinect[nearestIndex].qmap;
				double w,x,y,z;
				tmpQmap["LARM_JOINT2"].getQuaternion(w, x, y, z);
				qmap["RARM_JOINT2"].setQuaternion(w, -x, -y, -z);
				tmpQmap["LARM_JOINT3"].getQuaternion(w, x, y, z);
				qmap["RARM_JOINT3"].setQuaternion(w, -x, -y, -z);
				tmpQmap["LARM_JOINT5"].getQuaternion(w, x, y, z);
				qmap["RARM_JOINT5"].setQuaternion(w, -x, -y, -z);
			}
			else if (reverseMode == "NOREVERSE")
			{
				// Past body information.
				qmap = this->quaternionRecordForKinect[nearestIndex].qmap;
			}

		}
		else if (firstOfMessage == "ORS_DATA")
		{
			qmap = this->quaternionRecordForOculus[nearestIndex].qmap;
		}
	}

}

void ManNiiController::setQMapFromJointNameAndQuaternionForKinect(QMap &qmap, const std::vector<std::string> &jointNameAndQuaternionsVector)
{
	// Parsing the message.
	for (int i = 1; i < (int)jointNameAndQuaternionsVector.size(); i++)
	{
		// Get each joint name and joint's quaternion in received message.

		// Divide the message by delimiter to vector of "JONT-NAME" and "QUATERNION".
		const std::string delimiterOfColon = ":";
		std::vector<std::string> eachJointNameAndQuaternion;
		splitString(jointNameAndQuaternionsVector[i], delimiterOfColon, eachJointNameAndQuaternion);

		if ((int)eachJointNameAndQuaternion.size() < 1)
		{
			// When there isn't ":" in the message, break this loop.
			break;
		}
		else
		{
			// Get joint name from the divided message.
			const std::string jointName = eachJointNameAndQuaternion[0];

			if (jointName == "END") break;

			if (jointName == "POSITION" || jointName == "WAIST") continue;
			//if (jointName == "LLEG_JONT6" || jointName == "RLEG_JOINT4") continue;

			if (this->usingOculus && jointName == "HEAD_JOINT0") continue;

			// Divide message to get quaternion from the divided message.
			std::vector<std::string> tmpQuaternionsString;
			const std::string delimiterOfComma = ",";
			splitString(eachJointNameAndQuaternion[1], delimiterOfComma, tmpQuaternionsString);

			// Get quaternion.
			const double w = atof(tmpQuaternionsString[0].c_str());
			const double x = atof(tmpQuaternionsString[1].c_str());
			const double y = atof(tmpQuaternionsString[2].c_str());
			const double z = atof(tmpQuaternionsString[3].c_str());
			const Quaternion tmpQuaternion(w, x, y, z);

			// Set target joint and its quaternion.
			qmap.insert(std::make_pair(jointName, tmpQuaternion));
		}
	}
}

void ManNiiController::setQMapFromJointNameAndQuaternionForOculus(QMap &qmap, const std::string &message)
{
	usingOculus = true;
	// Get quaternion map from oculus data.
	std::string ss = message;
	int strPos1 = 0;
	int strPos2;
	std::string headss;
	std::string tmpss;
	strPos2 = ss.find(" ", strPos1);
	headss.assign(ss, strPos1, strPos2-strPos1);

	if (headss == "ORS_DATA") {
		//    LOG_MSG((all_msg));
		//  }
		//  if(headss == "HMD_DATA"){

		double yaw, pitch, roll;
		strPos1 = strPos2+1;
		tmpss = "";

		strPos2 = ss.find(",", strPos1);
		tmpss.assign(ss, strPos1, strPos2-strPos1);
		yaw = atof(tmpss.c_str());

		strPos1 = strPos2+1;
		strPos2 = ss.find(",", strPos1);
		tmpss.assign(ss, strPos1, strPos2-strPos1);
		pitch = atof(tmpss.c_str());

		strPos1 = strPos2+1;
		strPos2 = ss.find(",", strPos1);
		tmpss.assign(ss, strPos1, strPos2-strPos1);
		roll = atof(tmpss.c_str());

		if (yaw == pyaw && pitch == ppitch && roll == proll)  return;
		else {
			pyaw = yaw;
			ppitch = pitch;
			proll = roll;
		}

		dQuaternion qyaw;
		dQuaternion qpitch;
		dQuaternion qroll;

		qyaw[0] = cos(-yaw/2.0);
		qyaw[1] = 0.0;
		qyaw[2] = sin(-yaw/2.0);
		qyaw[3] = 0.0;

		qpitch[0] = cos(-pitch/2.0);
		qpitch[1] = sin(-pitch/2.0);
		qpitch[2] = 0.0;
		qpitch[3] = 0.0;

		qroll[0] = cos(-roll/2.0);
		qroll[1] = 0.0;
		qroll[2] = 0.0;
		qroll[3] = sin(-roll/2.0);
		dQuaternion tmpQ1;
		dQuaternion tmpQ2;

		dQMultiply0(tmpQ1, qyaw, qpitch);
		dQMultiply0(tmpQ2, tmpQ1, qroll);

		dQuaternion bodyQ;
		bodyQ[0] = m_qw;
		bodyQ[1] = m_qx;
		bodyQ[2] = m_qy;
		bodyQ[3] = m_qz;

		dQuaternion tmpQ3;
		dQMultiply1(tmpQ3, bodyQ, tmpQ2);
		//printf("\r HEAD_JOINT0:%1.3f,%1.3f,%1.3f,%1.3f", tmpQ3[0], tmpQ3[1], tmpQ3[2], tmpQ3[3]);
		//fflush(stdout);
		//my->setJointQuaternion("HEAD_JOINT0", tmpQ3[0], tmpQ3[1], tmpQ3[2], tmpQ3[3]);
		const Quaternion tmpQuaternion(tmpQ3[0], tmpQ3[1], tmpQ3[2], tmpQ3[3]);
		qmap.insert(std::make_pair("HEAD_JOINT0", tmpQuaternion));
	}
}

const double ManNiiController::generateCurrentTimeStamp()
{
	// Get current timestamp.
	struct timeval currentTimeVal;
	gettimeofday(&currentTimeVal, NULL);
	const double currentTimeStamp = (currentTimeVal.tv_sec - this->initTimeVal.tv_sec) * 1000.0 + (currentTimeVal.tv_usec - this->initTimeVal.tv_usec) / 1000.0;
	return currentTimeStamp;
}

extern "C" Controller * createController()
{
	return new ManNiiController;
}
