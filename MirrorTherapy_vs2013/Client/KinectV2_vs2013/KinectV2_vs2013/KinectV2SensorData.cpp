#include "KinectV2SensorData.h"

///@brief Orientation (w, x, y, z) to string.
struct orientation2Message : public std::string {
	orientation2Message(const Vector4 &orientation) {
		std::stringstream ssOrientation;
		ssOrientation << std::setprecision(KinectV2SensorData::orientationPrecision);
		ssOrientation << "(" << orientation.w << "," << orientation.x << "," << orientation.y << "," << orientation.z << ")";
		assign(ssOrientation.str());
	}
};

///@brief JointOrientation (jointname, w, x, y, z) to string.
struct jointOrientation2Message : public std::string {
	jointOrientation2Message(const KinectV2JointOrientation &jo) {
		std::stringstream ss;
		ss << jointType2Message(jo.jointType) << ":" << orientation2Message(jo.orientation);
		assign(ss.str());
	}
};

///@brief Three dimensional position (x, y, z) to string.
struct position2Message : public std::string {
	position2Message(const Vector3 &position) {
		std::stringstream ssPosition;
		ssPosition << std::setprecision(KinectV2SensorData::rootPositionPrecision);
		ssPosition << "SpBs_P:" << "(" << position.x << "," << position.y << "," << position.z << ")";
		assign(ssPosition.str());
	}
};
///@brief Generate message by posture.
std::string KinectV2SensorData::convertSensorData2Message()
{
	std::stringstream ss;
	ss << position2Message(this->rootPosition) << ";"
		<< jointOrientation2Message(this->jointOrientations[SpineBase]) << ";"
		<< jointOrientation2Message(this->jointOrientations[SpineMid]) << ";"
		<< jointOrientation2Message(this->jointOrientations[Neck]) << ";"
		<< jointOrientation2Message(this->jointOrientations[Head]) << ";"
		<< jointOrientation2Message(this->jointOrientations[ShoulderLeft]) << ";"
		<< jointOrientation2Message(this->jointOrientations[ElbowLeft]) << ";"
		<< jointOrientation2Message(this->jointOrientations[WristLeft]) << ";"
		<< jointOrientation2Message(this->jointOrientations[HandLeft]) << ";"
		<< jointOrientation2Message(this->jointOrientations[ShoulderRight]) << ";"
		<< jointOrientation2Message(this->jointOrientations[ElbowRight]) << ";"
		<< jointOrientation2Message(this->jointOrientations[WristRight]) << ";"
		<< jointOrientation2Message(this->jointOrientations[HandRight]) << ";"
		<< jointOrientation2Message(this->jointOrientations[HipLeft]) << ";"
		<< jointOrientation2Message(this->jointOrientations[KneeLeft]) << ";"
		<< jointOrientation2Message(this->jointOrientations[AnkleLeft]) << ";"
		<< jointOrientation2Message(this->jointOrientations[FootLeft]) << ";"
		<< jointOrientation2Message(this->jointOrientations[HipRight]) << ";"
		<< jointOrientation2Message(this->jointOrientations[KneeRight]) << ";"
		<< jointOrientation2Message(this->jointOrientations[AnkleRight]) << ";"
		<< jointOrientation2Message(this->jointOrientations[FootRight]) << ";"
		<< jointOrientation2Message(this->jointOrientations[SpineShoulder]) << ";"
		<< jointOrientation2Message(this->jointOrientations[HandTipLeft]) << ";"
		<< jointOrientation2Message(this->jointOrientations[ThumbLeft]) << ";"
		<< jointOrientation2Message(this->jointOrientations[HandTipRight]) << ";"
		<< jointOrientation2Message(this->jointOrientations[ThumbRight]);
	return(ss.str());
}
