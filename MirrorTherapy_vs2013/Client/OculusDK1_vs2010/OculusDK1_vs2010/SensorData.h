#pragma once
#include <string>

class SensorData
{
public:
	//virtual ~SensorData() {};

	///@brief Convert posture information to string.
	virtual std::string convertSensorData2Message(const std::string &keyDelim, const std::string &recordDelim, const std::string &valueDelim) = 0;
	//virtual void convertMessage2SensorData() = 0;
};

