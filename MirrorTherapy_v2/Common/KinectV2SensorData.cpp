#include "KinectV2SensorData.h"
//#include <sigverse/SimObj.h>
//#include <sigverse/Logger.h>

///@brief Generate message by posture.
std::string KinectV2SensorData::encodeSensorData(const std::string &pairsDelim, const std::string &keyValueDelim, const std::string &vectorDelim) const
{
	std::stringstream ss;
	ss  << position2Message        (this->rootPosition,                     keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[SpineBase],     keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[SpineMid],      keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[Neck],          keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[Head],          keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[ShoulderLeft],  keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[ElbowLeft],     keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[WristLeft],     keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[HandLeft],      keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[ShoulderRight], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[ElbowRight],    keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[WristRight],    keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[HandRight],     keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[HipLeft],       keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[KneeLeft],      keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[AnkleLeft],     keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[FootLeft],      keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[HipRight],      keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[KneeRight],     keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[AnkleRight],    keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[FootRight],     keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[SpineShoulder], keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[HandTipLeft],   keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[ThumbLeft],     keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[HandTipRight],  keyValueDelim, vectorDelim) << pairsDelim
		<< jointOrientation2Message(this->jointOrientations[ThumbRight],    keyValueDelim, vectorDelim);
	return(ss.str());
}


bool KinectV2SensorData::setSensorData(const std::map<std::string, std::vector<std::string> > &sensorDataMap)
{
	// Generate map<string, vector<string>, which has only JointType.
	std::map<std::string, std::vector<std::string> > jointMap;
	std::vector<std::string> emptyStrings(4, "");

	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(MSG_KEY_SPBS_P,                          emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(SpineBase),     emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(SpineMid),      emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(Neck),          emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(Head),          emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(ShoulderLeft) , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(ElbowLeft) ,    emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(WristLeft) ,    emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(HandLeft),      emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(ShoulderRight), emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(ElbowRight) ,   emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(WristRight) ,   emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(HandRight),     emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(HipLeft) ,      emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(KneeLeft),      emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(AnkleLeft),     emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(FootLeft) ,     emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(HipRight) ,     emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(KneeRight),     emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(AnkleRight),    emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(FootRight) ,    emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(SpineShoulder), emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(HandTipLeft),   emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(ThumbLeft),     emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(HandTipRight) , emptyStrings));
	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(jointType2ShortJointName(ThumbRight),    emptyStrings));

	std::map<std::string, std::vector<std::string> >::const_iterator it;

	for (it = sensorDataMap.begin(); it != sensorDataMap.end(); it++)
	{
		const std::string key = (*it).first;

		if (jointMap.find(key) == jointMap.end()){ continue; }

		if ((*it).first == MSG_KEY_SPBS_P)
		{
			Vector3 tmpPosition;
			tmpPosition.x = (float)atof((*it).second[0].c_str());
			tmpPosition.y = (float)atof((*it).second[1].c_str());
			tmpPosition.z = (float)atof((*it).second[2].c_str());
			this->rootPosition = tmpPosition;
			continue;
		}
		//std::cout << (*i).first << ":" << (*i).second[0] << "," << (*i).second[1] << "," << (*i).second[2] << "," << (*i).second[3] << ";" << std::endl;
		else
		{
			try
			{
				KinectV2JointOrientation tmpJointOrientation;
				tmpJointOrientation.jointType = this->shortJointName2KinectV2JointType((*it).first);
				tmpJointOrientation.orientation.w = (float)atof((*it).second[0].c_str());
				tmpJointOrientation.orientation.x = (float)atof((*it).second[1].c_str());
				tmpJointOrientation.orientation.y = (float)atof((*it).second[2].c_str());
				tmpJointOrientation.orientation.z = (float)atof((*it).second[3].c_str());

				this->jointOrientations[tmpJointOrientation.jointType] = tmpJointOrientation;
			}
			catch(std::string &ex)
			{
				std::cout << "Exception: " << ex << std::endl;
			}
			continue;
		}
	}

	return true;
}

void KinectV2SensorData::setKinectV2JointOrientation(KinectV2JointOrientation *kinectV2JointOrientations)
{
	for (int i = 0; i < KinectV2JointType_Count; i++)
	{
		this->jointOrientations[i].jointType   = kinectV2JointOrientations[i].jointType;
		this->jointOrientations[i].orientation = kinectV2JointOrientations[i].orientation;
	}
}

void KinectV2SensorData::getKinectV2JointOrientation(KinectV2JointOrientation* destination)
{
	for (int i = 0; i < KinectV2JointType_Count; i++)
	{
		destination[i].jointType   = this->jointOrientations[i].jointType;
		destination[i].orientation = this->jointOrientations[i].orientation;
	}
}

///@brief Get string enum of JointType.
std::string KinectV2SensorData::jointType2String(KinectV2SensorData::KinectV2JointType e) const
{
	switch (e)
	{
		break; case SpineBase:      { return("SpineBase"); }
		break; case SpineMid:       { return("SpineMid"); }
		break; case Neck:           { return("Neck"); }
		break; case Head:           { return("Head"); }
		break; case ShoulderLeft:   { return("ShoulderLeft"); }
		break; case ElbowLeft:      { return("ElbowLeft"); }
		break; case WristLeft:      { return("WristLeft"); }
		break; case HandLeft:       { return("HandLeft"); }
		break; case ShoulderRight:  { return("ShoulderRight"); }
		break; case ElbowRight:     { return("ElbowRight"); }
		break; case WristRight:     { return("WristRight"); }
		break; case HandRight:      { return("HandRight"); }
		break; case HipLeft:        { return("HipLeft"); }
		break; case KneeLeft:       { return("KneeLeft"); }
		break; case AnkleLeft:      { return("AnkleLeft"); }
		break; case FootLeft:       { return("FootLeft"); }
		break; case HipRight:       { return("HipRight"); }
		break; case KneeRight:      { return("KneeRight"); }
		break; case AnkleRight:     { return("AnkleRight"); }
		break; case FootRight:      { return("FootRight"); }
		break; case SpineShoulder:  { return("SpineShoulder"); }
		break; case HandTipLeft:    { return("HandTipLeft"); }
		break; case ThumbLeft:      { return("ThumbLeft"); }
		break; case HandTipRight:   { return("HandTipRight"); }
		break; case ThumbRight:     { return("ThumbRight"); }
		break; default:             { return("illegal"); }
	}
}

///@brief Get string enum of JointType to genrate message.
std::string KinectV2SensorData::jointType2ShortJointName(KinectV2JointType e) const
{
	switch (e)
	{
		break; case SpineBase:      { return("SpBs_Q"); }
		break; case SpineMid:       { return("SpMd_Q"); }
		break; case Neck:           { return("Neck_Q"); }
		break; case Head:           { return("Head_Q"); }
		break; case ShoulderLeft:   { return("ShL_Q"); }
		break; case ElbowLeft:      { return("LbL_Q"); }
		break; case WristLeft:      { return("WrL_Q"); }
		break; case HandLeft:       { return("HndL_Q"); }
		break; case ShoulderRight:  { return("ShR_Q"); }
		break; case ElbowRight:     { return("LbR_Q"); }
		break; case WristRight:     { return("WrR_Q"); }
		break; case HandRight:      { return("HndR_Q"); }
		break; case HipLeft:        { return("HpL_Q"); }
		break; case KneeLeft:       { return("NeeL_Q"); }
		break; case AnkleLeft:      { return("AnkL_Q"); }
		break; case FootLeft:       { return("FtL_Q"); }
		break; case HipRight:       { return("HpR_Q"); }
		break; case KneeRight:      { return("NeeR_Q"); }
		break; case AnkleRight:     { return("AnkR_Q"); }
		break; case FootRight:      { return("FtR_Q"); }
		break; case SpineShoulder:  { return("SpSh_Q"); }
		break; case HandTipLeft:    { return("HTL_Q"); }
		break; case ThumbLeft:      { return("ThmL_Q"); }
		break; case HandTipRight:   { return("HTR_Q"); }
		break; case ThumbRight:     { return("ThmR_Q"); }
		break; default:             { return("illegal"); }
	}
}


KinectV2SensorData::KinectV2JointType KinectV2SensorData::shortJointName2KinectV2JointType(const std::string &shortJointName) const
{
	if      (shortJointName == "SpBs_Q"){ return SpineBase; }
	else if (shortJointName == "SpMd_Q"){ return SpineMid; }
	else if (shortJointName == "Neck_Q"){ return Neck; }
	else if (shortJointName == "Head_Q"){ return Head; }
	else if (shortJointName == "ShL_Q") { return ShoulderLeft; }
	else if (shortJointName == "LbL_Q") { return ElbowLeft; }
	else if (shortJointName == "WrL_Q") { return WristLeft; }
	else if (shortJointName == "HndL_Q"){ return HandLeft; }
	else if (shortJointName == "ShR_Q") { return ShoulderRight; }
	else if (shortJointName == "LbR_Q") { return ElbowRight; }
	else if (shortJointName == "WrR_Q") { return WristRight; }
	else if (shortJointName == "HndR_Q"){ return HandRight; }
	else if (shortJointName == "HpL_Q") { return HipLeft; }
	else if (shortJointName == "NeeL_Q"){ return KneeLeft; }
	else if (shortJointName == "AnkL_Q"){ return AnkleLeft; }
	else if (shortJointName == "FtL_Q") { return FootLeft; }
	else if (shortJointName == "HpR_Q") { return HipRight; }
	else if (shortJointName == "NeeR_Q"){ return KneeRight; }
	else if (shortJointName == "AnkR_Q"){ return AnkleRight; }
	else if (shortJointName == "FtR_Q") { return FootRight;}
	else if (shortJointName == "SpSh_Q"){ return SpineShoulder; }
	else if (shortJointName == "HTL_Q") { return HandTipLeft; }
	else if (shortJointName == "ThmL_Q"){ return ThumbLeft; }
	else if (shortJointName == "HTR_Q") { return HandTipRight; }
	else if (shortJointName == "ThmR_Q"){ return ThumbRight; }
	else { throw std::string(("This short joint name is invalid. : "+shortJointName).c_str()); }
}


///@brief Orientation (w, x, y, z) to string.
std::string KinectV2SensorData::orientation2Message(const SensorData::Vector4 &orientation, const std::string &valuesDelim) const
{
	std::stringstream ssOrientation;
	ssOrientation << std::setprecision(KinectV2SensorData::orientationPrecision);
	ssOrientation << orientation.w << valuesDelim << orientation.x << valuesDelim << orientation.y << valuesDelim << orientation.z;
	return ssOrientation.str();
}

///@brief JointOrientation (jointname, w, x, y, z) to string.
std::string KinectV2SensorData::jointOrientation2Message(const KinectV2JointOrientation &jo, const std::string &keyValueDelim, const std::string &valuesDelim) const
{
	std::stringstream ss;
	ss << jointType2ShortJointName(jo.jointType) << keyValueDelim << orientation2Message(jo.orientation, valuesDelim);
	return ss.str();
}

///@brief Three dimensional position (x, y, z) to string.
std::string KinectV2SensorData::position2Message(const SensorData::Vector3 &position, const std::string &keyValueDelim, const std::string &valuesDelim) const
{
	std::stringstream ssPosition;
	ssPosition << std::setprecision(KinectV2SensorData::rootPositionPrecision);
	ssPosition << MSG_KEY_SPBS_P << keyValueDelim  << position.x << valuesDelim << position.y << valuesDelim << position.z;
	return ssPosition.str();
}

