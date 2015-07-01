#include "KinectV2SensorData.h"


///@brief Default Constructor
KinectV2SensorData::KinectV2SensorData()
{
}

///@brief Constructor
KinectV2SensorData::KinectV2SensorData(std::string sensorDataModeStr)
{
	this->setSensorDataMode(sensorDataModeStr);
}

void KinectV2SensorData::setSensorDataMode(std::string sensorDataModeStr)
{
	if(sensorDataModeStr == "Quaternion")
	{
		this->sensorDataMode = Quaternion;
	}
	else if (sensorDataModeStr == "Position")
	{
		this->sensorDataMode = Position; 
	}
	else 
	{ 
		throw std::string("invalid sensor data mode:"+sensorDataModeStr); 
	}
}


std::string KinectV2SensorData::getSensorDataModeStr()
{
	switch (this->sensorDataMode)
	{
		case Quaternion:{ return "Quaternion"; break; }
		case Position:  { return "Position";   break; }
		default:        { return "";           break; }
	}
};



///@brief Generate message by posture.
std::string KinectV2SensorData::encodeSensorData(const std::string &pairsDelim, const std::string &keyValueDelim, const std::string &vectorDelim) const
{
	std::stringstream ss;

	switch (this->sensorDataMode)
	{
		case Position:
		{
			ss  << rootPosition2Message (this->rootPosition,                  keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[SpineBase],     keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[SpineMid],      keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[Neck],          keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[Head],          keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[ShoulderLeft],  keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[ElbowLeft],     keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[WristLeft],     keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[HandLeft],      keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[ShoulderRight], keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[ElbowRight],    keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[WristRight],    keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[HandRight],     keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[HipLeft],       keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[KneeLeft],      keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[AnkleLeft],     keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[FootLeft],      keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[HipRight],      keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[KneeRight],     keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[AnkleRight],    keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[FootRight],     keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[SpineShoulder], keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[HandTipLeft],   keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[ThumbLeft],     keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[HandTipRight],  keyValueDelim, vectorDelim) << pairsDelim
				<< jointPosition2Message(this->jointPositions[ThumbRight],    keyValueDelim, vectorDelim);
			break;
		}
		case Quaternion:
		{
			ss  << rootPosition2Message    (this->rootPosition,                     keyValueDelim, vectorDelim) << pairsDelim
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
			break;
		}
	}

	return(ss.str());
}


bool KinectV2SensorData::setSensorData(const std::map<std::string, std::vector<std::string> > &sensorDataMap)
{
	// Generate map<string, vector<string>, which has only JointType.
	std::map<std::string, std::vector<std::string> > jointMap;
	std::vector<std::string> emptyStrings(4, "");

	jointMap.insert(std::map<std::string, std::vector<std::string> >::value_type(MSG_KEY_ROOT_P,                          emptyStrings));
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

		if ((*it).first == MSG_KEY_ROOT_P)
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
				switch (this->sensorDataMode)
				{
					case Position:
					{

						KinectV2JointPosition tmpJointPosition;
						tmpJointPosition.jointType     = this->shortJointName2KinectV2JointType((*it).first);
						tmpJointPosition.position.x    = (float)atof((*it).second[1].c_str());
						tmpJointPosition.position.y    = (float)atof((*it).second[2].c_str());
						tmpJointPosition.position.z    = (float)atof((*it).second[3].c_str());
						tmpJointPosition.trackingState = (KinectV2TrackingState)atoi((*it).second[0].c_str());

						this->jointPositions[tmpJointPosition.jointType] = tmpJointPosition;
						break;
					}
					case Quaternion:
					{

						KinectV2JointOrientation tmpJointOrientation;
						tmpJointOrientation.jointType = this->shortJointName2KinectV2JointType((*it).first);
						tmpJointOrientation.orientation.w = (float)atof((*it).second[0].c_str());
						tmpJointOrientation.orientation.x = (float)atof((*it).second[1].c_str());
						tmpJointOrientation.orientation.y = (float)atof((*it).second[2].c_str());
						tmpJointOrientation.orientation.z = (float)atof((*it).second[3].c_str());

						this->jointOrientations[tmpJointOrientation.jointType] = tmpJointOrientation;
						break;
					}
				}
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

///@brief Set root position.
void KinectV2SensorData::setRootPosition(const Vector3 &position)
{
	this->rootPosition = position;
}


void KinectV2SensorData::setKinectV2JointPosition(KinectV2JointPosition *kinectV2JointPositions)
{
	for (int i = 0; i < KinectV2JointType_Count; i++)
	{
		this->jointPositions[i].jointType     = kinectV2JointPositions[i].jointType;
		this->jointPositions[i].position      = kinectV2JointPositions[i].position;
		this->jointPositions[i].trackingState = kinectV2JointPositions[i].trackingState;
	}
}

void KinectV2SensorData::getKinectV2JointPosition(KinectV2JointPosition* destination)
{
	for (int i = 0; i < KinectV2JointType_Count; i++)
	{
		destination[i].jointType     = this->jointPositions[i].jointType;
		destination[i].position      = this->jointPositions[i].position;
		destination[i].trackingState = this->jointPositions[i].trackingState;
	}
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
		case SpineBase:      { return("SpineBase");     break; }
		case SpineMid:       { return("SpineMid");      break; }
		case Neck:           { return("Neck");          break; }
		case Head:           { return("Head");          break; }
		case ShoulderLeft:   { return("ShoulderLeft");  break; }
		case ElbowLeft:      { return("ElbowLeft");     break; }
		case WristLeft:      { return("WristLeft");     break; }
		case HandLeft:       { return("HandLeft");      break; }
		case ShoulderRight:  { return("ShoulderRight"); break; }
		case ElbowRight:     { return("ElbowRight");    break; }
		case WristRight:     { return("WristRight");    break; }
		case HandRight:      { return("HandRight");     break; }
		case HipLeft:        { return("HipLeft");       break; }
		case KneeLeft:       { return("KneeLeft");      break; }
		case AnkleLeft:      { return("AnkleLeft");     break; }
		case FootLeft:       { return("FootLeft");      break; }
		case HipRight:       { return("HipRight");      break; }
		case KneeRight:      { return("KneeRight");     break; }
		case AnkleRight:     { return("AnkleRight");    break; }
		case FootRight:      { return("FootRight");     break; }
		case SpineShoulder:  { return("SpineShoulder"); break; }
		case HandTipLeft:    { return("HandTipLeft");   break; }
		case ThumbLeft:      { return("ThumbLeft");     break; }
		case HandTipRight:   { return("HandTipRight");  break; }
		case ThumbRight:     { return("ThumbRight");    break; }
		default:             { return("illegal");       break; }
	}
}

///@brief Get string enum of JointType to genrate message.
std::string KinectV2SensorData::jointType2ShortJointName(KinectV2JointType e) const
{
	std::string sensorDataModeType;

	switch (this->sensorDataMode)
	{
		case Quaternion:{ sensorDataModeType = "Q"; break; }
		case Position:  { sensorDataModeType = "P"; break; }
	}

	switch (e)
	{
		case SpineBase:      { return "SpBs_" + sensorDataModeType; break; }
		case SpineMid:       { return "SpMd_" + sensorDataModeType; break; }
		case Neck:           { return "Neck_" + sensorDataModeType; break; }
		case Head:           { return "Head_" + sensorDataModeType; break; }
		case ShoulderLeft:   { return "ShL_"  + sensorDataModeType; break; }
		case ElbowLeft:      { return "LbL_"  + sensorDataModeType; break; }
		case WristLeft:      { return "WrL_"  + sensorDataModeType; break; }
		case HandLeft:       { return "HndL_" + sensorDataModeType; break; }
		case ShoulderRight:  { return "ShR_"  + sensorDataModeType; break; }
		case ElbowRight:     { return "LbR_"  + sensorDataModeType; break; }
		case WristRight:     { return "WrR_"  + sensorDataModeType; break; }
		case HandRight:      { return "HndR_" + sensorDataModeType; break; }
		case HipLeft:        { return "HpL_"  + sensorDataModeType; break; }
		case KneeLeft:       { return "NeeL_" + sensorDataModeType; break; }
		case AnkleLeft:      { return "AnkL_" + sensorDataModeType; break; }
		case FootLeft:       { return "FtL_"  + sensorDataModeType; break; }
		case HipRight:       { return "HpR_"  + sensorDataModeType; break; }
		case KneeRight:      { return "NeeR_" + sensorDataModeType; break; }
		case AnkleRight:     { return "AnkR_" + sensorDataModeType; break; }
		case FootRight:      { return "FtR_"  + sensorDataModeType; break; }
		case SpineShoulder:  { return "SpSh_" + sensorDataModeType; break; }
		case HandTipLeft:    { return "HTL_"  + sensorDataModeType; break; }
		case ThumbLeft:      { return "ThmL_" + sensorDataModeType; break; }
		case HandTipRight:   { return "HTR_"  + sensorDataModeType; break; }
		case ThumbRight:     { return "ThmR_" + sensorDataModeType; break; }
		default:             { return "illegal";                    break; }
	}
}


KinectV2SensorData::KinectV2JointType KinectV2SensorData::shortJointName2KinectV2JointType(const std::string &shortJointName) const
{
	if (shortJointName.length==0)
	{
		throw std::string("Short joint name is blank.");
	}

	std::string  shortJointNameForJudge = shortJointName;
	shortJointNameForJudge.erase(--shortJointName.end());

	if      (shortJointName == "SpBs_"){ return SpineBase; }
	else if (shortJointName == "SpMd_"){ return SpineMid; }
	else if (shortJointName == "Neck_"){ return Neck; }
	else if (shortJointName == "Head_"){ return Head; }
	else if (shortJointName == "ShL_") { return ShoulderLeft; }
	else if (shortJointName == "LbL_") { return ElbowLeft; }
	else if (shortJointName == "WrL_") { return WristLeft; }
	else if (shortJointName == "HndL_"){ return HandLeft; }
	else if (shortJointName == "ShR_") { return ShoulderRight; }
	else if (shortJointName == "LbR_") { return ElbowRight; }
	else if (shortJointName == "WrR_") { return WristRight; }
	else if (shortJointName == "HndR_"){ return HandRight; }
	else if (shortJointName == "HpL_") { return HipLeft; }
	else if (shortJointName == "NeeL_"){ return KneeLeft; }
	else if (shortJointName == "AnkL_"){ return AnkleLeft; }
	else if (shortJointName == "FtL_") { return FootLeft; }
	else if (shortJointName == "HpR_") { return HipRight; }
	else if (shortJointName == "NeeR_"){ return KneeRight; }
	else if (shortJointName == "AnkR_"){ return AnkleRight; }
	else if (shortJointName == "FtR_") { return FootRight;}
	else if (shortJointName == "SpSh_"){ return SpineShoulder; }
	else if (shortJointName == "HTL_") { return HandTipLeft; }
	else if (shortJointName == "ThmL_"){ return ThumbLeft; }
	else if (shortJointName == "HTR_") { return HandTipRight; }
	else if (shortJointName == "ThmR_"){ return ThumbRight; }
	else { throw std::string(("Short joint name is invalid. : "+shortJointName).c_str()); }
}


///@brief root position (Root_P, x, y, z) to string.
std::string KinectV2SensorData::rootPosition2Message(const SensorData::Vector3 &position, const std::string &keyValueDelim, const std::string &valuesDelim) const
{
	std::stringstream ss;
	ss << MSG_KEY_ROOT_P << keyValueDelim << position2Message(position, valuesDelim);
	return ss.str();
}


///@brief Three dimensional position (x, y, z) to string.
std::string KinectV2SensorData::position2Message(const SensorData::Vector3 &position, const std::string &valuesDelim) const
{
	std::stringstream ssPosition;
	ssPosition << std::setprecision(KinectV2SensorData::rootPositionPrecision);
	ssPosition << position.x << valuesDelim << position.y << valuesDelim << position.z;
	return ssPosition.str();
}

///@brief JointPosition (jointname, x, y, z) to string.
std::string KinectV2SensorData::jointPosition2Message(const KinectV2JointPosition &jp, const std::string &keyValueDelim, const std::string &valuesDelim) const
{
	std::stringstream ss;
	ss << jointType2ShortJointName(jp.jointType) << keyValueDelim << position2Message(jp.position, valuesDelim) << valuesDelim << jp.trackingState;
	return ss.str();
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


