#pragma once
#include <string>

class SensorData
{
public:
	//virtual ~SensorData() {};

	///@brief Convert posture information to string.
	virtual std::string convertSensorData2Message() = 0;
	//virtual void convertMessage2SensorData() = 0;
};

