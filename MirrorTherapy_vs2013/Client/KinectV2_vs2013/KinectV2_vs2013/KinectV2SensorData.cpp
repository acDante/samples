#include "KinectV2SensorData.h"

///@brief Orientation (w, x, y, z) to string.
struct orientation2Message : public std::string {
	orientation2Message(const Vector4 &orientation, const std::string &valueDelim) {
		std::stringstream ssOrientation;
		ssOrientation << std::setprecision(KinectV2SensorData::orientationPrecision);
		ssOrientation << "(" << orientation.w << valueDelim << orientation.x << valueDelim << orientation.y << valueDelim << orientation.z << ")";
		assign(ssOrientation.str());
	}
};

///@brief JointOrientation (jointname, w, x, y, z) to string.
struct jointOrientation2Message : public std::string {
	jointOrientation2Message(const KinectV2JointOrientation &jo, const std::string &keyDelim, const std::string &valueDelim) {
		std::stringstream ss;
		ss << jointType2Message(jo.jointType) << keyDelim << orientation2Message(jo.orientation, valueDelim);
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
std::string KinectV2SensorData::convertSensorData2Message(const std::string &keyDelim, const std::string &recordDelim, const std::string &valueDelim)
{
	std::stringstream ss;
	ss << position2Message(this->rootPosition) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[SpineBase], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[SpineMid], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[Neck], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[Head], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[ShoulderLeft], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[ElbowLeft], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[WristLeft], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[HandLeft], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[ShoulderRight], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[ElbowRight], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[WristRight], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[HandRight], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[HipLeft], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[KneeLeft], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[AnkleLeft], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[FootLeft], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[HipRight], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[KneeRight], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[AnkleRight], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[FootRight], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[SpineShoulder], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[HandTipLeft], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[ThumbLeft], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[HandTipRight], keyDelim, valueDelim) << recordDelim
		<< jointOrientation2Message(this->jointOrientations[ThumbRight], keyDelim, valueDelim);
	return(ss.str());
}
