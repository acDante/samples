#include <sigverse/common/device/OculusRiftDK2SensorData.h>
#include <sstream>

///@brief generate message by posture.
std::string OculusRiftDK2SensorData::encodeSensorData(const std::string &itemsDelim, const std::string &keyValueDelim, const std::string &valuesDelim) const
{
	std::vector<std::string> vec;

	std::stringstream ssValue;

	ssValue << this->quaternion.w;   vec.push_back(ssValue.str()); ssValue.clear(); ssValue.str("");
	ssValue << this->quaternion.x; vec.push_back(ssValue.str()); ssValue.clear(); ssValue.str("");
	ssValue << this->quaternion.y;  vec.push_back(ssValue.str()); ssValue.clear(); ssValue.str("");
	ssValue << this->quaternion.z;  vec.push_back(ssValue.str());

	std::map<std::string, std::vector<std::string> > messageMap;

	messageMap.insert(make_pair(MSG_KEY_QUATERNION, vec));

	return this->convertMap2Message(messageMap, itemsDelim, keyValueDelim, valuesDelim);
}


bool OculusRiftDK2SensorData::setSensorData(const std::map<std::string, std::vector<std::string> > &sensorDataMap)
{
	// Get values (vector data) from euler value "(yaw, pitch, roll)".

	std::map<std::string, std::vector<std::string> >::const_iterator it;

	for (it = sensorDataMap.begin(); it != sensorDataMap.end(); it++)
	{
		if ((*it).first == MSG_KEY_QUATERNION)
		{
			if ((*it).second.size() != 4)
			{
				std::cout << "Not quaternion value. Number of value items is " << (*it).second.size() << std::endl;
				return false;
			}

			// Extract quaternion.
			const float qw = (float)atof((*it).second[0].c_str());
			const float qx = (float)atof((*it).second[1].c_str());
			const float qy = (float)atof((*it).second[2].c_str());
			const float qz = (float)atof((*it).second[3].c_str());

			// Set extracted angle to sensor data.
			this->setQuaternion(qw, qx, qy, qz);

			continue;
		}
	}

	return true;
}


///@brief Set sensor data.
void OculusRiftDK2SensorData::setQuaternion(const float &w, const float &x, const float &y, const float &z)
{
	this->quaternion.w = w;
	this->quaternion.x = x;
	this->quaternion.y = y;
	this->quaternion.z = z;
}

///@brief Get sensor data.
SigCmn::Vector4 OculusRiftDK2SensorData::getQuaternion() const
{
	return this->quaternion;
}