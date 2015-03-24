#include "KinectV2SensorData.h"

///@brief Orientation (w, x, y, z) to string.
struct orientation2Message : public std::string {
	orientation2Message(const Vector4 &orientation, const std::string &vectorDelim) {
		std::stringstream ssOrientation;
		ssOrientation << std::setprecision(KinectV2SensorData::orientationPrecision);
		ssOrientation << "(" << orientation.w << vectorDelim << orientation.x << vectorDelim << orientation.y << vectorDelim << orientation.z << ")";
		assign(ssOrientation.str());
	}
};

///@brief JointOrientation (jointname, w, x, y, z) to string.
struct jointOrientation2Message : public std::string {
	jointOrientation2Message(const KinectV2JointOrientation &jo, const std::string &keyValueDelim, const std::string &vectorDelim) {
		std::stringstream ss;
		ss << jointType2ShortJointName(jo.jointType) << keyValueDelim << orientation2Message(jo.orientation, vectorDelim);
		assign(ss.str());
	}
};

///@brief Three dimensional position (x, y, z) to string.
struct position2Message : public std::string {
	position2Message(const Vector3 &position) {
		std::stringstream ssPosition;
		ssPosition << std::setprecision(KinectV2SensorData::rootPositionPrecision);
		ssPosition << "SpBs_P" << keyValueDelimDefault << "(" << position.x << vectorDelimDefault << position.y << vectorDelimDefault << position.z << ")";
		assign(ssPosition.str());
	}
};
///@brief Generate message by posture.
std::string KinectV2SensorData::encodeSensorData2Message(const std::string &pairsDelim, const std::string &keyValueDelim, const std::string &vectorDelim)
{
	std::stringstream ss;
	ss << position2Message(this->rootPosition) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[SpineBase], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[SpineMid], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[Neck], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[Head], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[ShoulderLeft], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[ElbowLeft], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[WristLeft], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[HandLeft], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[ShoulderRight], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[ElbowRight], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[WristRight], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[HandRight], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[HipLeft], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[KneeLeft], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[AnkleLeft], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[FootLeft], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[HipRight], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[KneeRight], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[AnkleRight], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[FootRight], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[SpineShoulder], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[HandTipLeft], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[ThumbLeft], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[HandTipRight], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[ThumbRight], keyValueDelim, vectorDelim);
	return(ss.str());
}

void KinectV2SensorData::decodeMessage2SensorData(const std::string &message, const std::string &pairsDelim, const std::string &keyValueDelim, const std::string &vectorDelim)
{
	// Generate map<string, vector<string> from message. Just split message by ";" and ":".
	std::map<std::string, std::vector<std::string>> messageMap = this->convertMessage2Map(message);

	// Generate map<string, vector<string>, which has only JointType.
	std::map<std::string, std::vector<std::string>> jointMap;
	std::vector<std::string> emptyStrings(4, "");
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("SpBs_P", emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("SpBs_Q", emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("SpMd_Q", emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("Neck_Q", emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("Head_Q", emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("ShL_Q" , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("LbL_Q" , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("WrL_Q" , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("HndL_Q", emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("ShR_Q" , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("LbR_Q" , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("WrR_Q" , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("HndR_Q", emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("HpL_Q" , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("NeeL_Q", emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("AnkL_Q", emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("FtL_Q" , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("HpR_Q" , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("NeeR_Q", emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("AnkR_Q", emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("FtR_Q" , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("SpSh_Q", emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("HTL_Q" , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("ThmL_Q", emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("HTR_Q" , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string>>::value_type("ThmR_Q", emptyStrings));

	for (auto i = messageMap.begin(); i != messageMap.end(); i++) {
		const std::string key = (*i).first;
		if (jointMap.find(key) != jointMap.end()) {
			if ((*i).first == "SpBs_P") {
				Vector3 tmpPosition;
				tmpPosition.x = atof((*i).second[0].c_str());
				tmpPosition.y = atof((*i).second[1].c_str());
				tmpPosition.z = atof((*i).second[2].c_str());
				this->rootPosition = tmpPosition;
				break;

			}
			//std::cout << (*i).first << ":" << (*i).second[0] << "," << (*i).second[1] << "," << (*i).second[2] << "," << (*i).second[3] << ";" << std::endl;

			KinectV2JointOrientation tmpJointOrientation;
			tmpJointOrientation.jointType = this->shortJointName2KinectV2JointType((*i).first);
			tmpJointOrientation.orientation.w = atof((*i).second[0].c_str());
			tmpJointOrientation.orientation.x = atof((*i).second[1].c_str());
			tmpJointOrientation.orientation.y = atof((*i).second[2].c_str());
			tmpJointOrientation.orientation.z = atof((*i).second[3].c_str());

			this->jointOrientations[tmpJointOrientation.jointType] = tmpJointOrientation;
		}
	}
}

void KinectV2SensorData::setKinectV2JointOrientation(KinectV2JointOrientation *kinectV2JointOrientations)
{
	for (int i = 0; i < KinectV2JointType_Count; i++) {
		this->jointOrientations[i].jointType = kinectV2JointOrientations[i].jointType;
		this->jointOrientations[i].orientation = kinectV2JointOrientations[i].orientation;
	}
}

KinectV2JointType KinectV2SensorData::shortJointName2KinectV2JointType(const std::string &shortJointName)
{
	try {
		if (shortJointName == "SpBs_Q") {
			return SpineBase;
		}
		else if (shortJointName == "SpMd_Q") {
			return SpineMid;
		}
		else if (shortJointName == "Neck_Q") {
			return Neck;
		}
		else if (shortJointName == "Head_Q") {
			return Head;
		}
		else if (shortJointName == "ShL_Q") {
			return ShoulderLeft;
		}
		else if (shortJointName == "LbL_Q") {
			return ElbowLeft;
		}
		else if (shortJointName == "WrL_Q") {
			return WristLeft;
		}
		else if (shortJointName == "HndL_Q") {
			return HandLeft;
		}
		else if (shortJointName == "ShR_Q") {
			return ShoulderRight;
		}
		else if (shortJointName == "LbR_Q") {
			return ElbowRight;
		}
		else if (shortJointName == "WrR_Q") {
			return WristRight;
		}
		else if (shortJointName == "HndR_Q") {
			return HandRight;
		}
		else if (shortJointName == "HpL_Q") {
			return HipLeft;
		}
		else if (shortJointName == "NeeL_Q") {
			return KneeLeft;
		}
		else if (shortJointName == "AnkL_Q") {
			return AnkleLeft;
		}
		else if (shortJointName == "FtL_Q") {
			return FootLeft;
		}
		else if (shortJointName == "HpR_Q") {
			return HipRight;
		}
		else if (shortJointName == "NeeR_Q") {
			return KneeRight;
		}
		else if (shortJointName == "AnkR_Q") {
			return AnkleRight;
		}
		else if (shortJointName == "FtR_Q") {
			return FootRight;
		}
		else if (shortJointName == "SpSh_Q") {
			return SpineShoulder;
		}
		else if (shortJointName == "HTL_Q") {
			return HandTipLeft;
		}
		else if (shortJointName == "ThmL_Q") {
			return ThumbLeft;
		}
		else if (shortJointName == "HTR_Q") {
			return HandTipRight;
		}
		else if (shortJointName == "ThmR_Q") {
			return ThumbRight;
		}
	}
	catch (std::exception &ex) {
		std::cout << ex.what() << std::endl;
	}
}