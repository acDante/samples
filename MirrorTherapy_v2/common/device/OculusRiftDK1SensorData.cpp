#include "OculusRiftDK1SensorData.h"
#include <sstream>


///@brief generate message by posture.
std::string OculusRiftDK1SensorData::encodeSensorData(const std::string &itemsDelim, const std::string &keyValueDelim, const std::string &valuesDelim) const
{

	std::vector<std::string> vec;

	std::stringstream ssValue;

	ssValue << this->eulerAngle.yaw;   vec.push_back(ssValue.str()); ssValue.clear(); ssValue.str("");
	ssValue << this->eulerAngle.pitch; vec.push_back(ssValue.str()); ssValue.clear(); ssValue.str("");
	ssValue << this->eulerAngle.roll;  vec.push_back(ssValue.str());

	std::map<std::string, std::vector<std::string> > messageMap;

	messageMap.insert(make_pair(MSG_KEY_EULER, vec) );

	return this->convertMap2Message(messageMap, itemsDelim, keyValueDelim, valuesDelim);
}


bool OculusRiftDK1SensorData::setSensorData(const std::map<std::string, std::vector<std::string> > &sensorDataMap)
{
	// Get values (vector data) from euler value "(yaw, pitch, roll)".

	std::map<std::string, std::vector<std::string> >::const_iterator it;

	for (it = sensorDataMap.begin(); it != sensorDataMap.end(); it++)
	{
		if((*it).first==MSG_KEY_EULER)
		{
			if((*it).second.size()!=3)
			{
				std::cout << "Not euler value. Number of value items is " << (*it).second.size() << std::endl;
				return false;
			}

			// Extract euler angle.
			const float yaw   = (float)atof((*it).second[0].c_str());
			const float pitch = (float)atof((*it).second[1].c_str());
			const float roll  = (float)atof((*it).second[2].c_str());

//			std::cout << "yaw, pitch, roll = " << yaw << "," << pitch << "," << roll << std::endl;

			// Set extracted angle to sensor data.
			this->setEulerAngle(yaw, pitch, roll);

			continue;
		}
	}

	return true;
}


///@brief Set sensor data.
void OculusRiftDK1SensorData::setEulerAngle(const float &yaw, const float &pitch, const float &roll)
{
	this->eulerAngle.yaw   = yaw;
	this->eulerAngle.pitch = pitch;
	this->eulerAngle.roll  = roll;
}
