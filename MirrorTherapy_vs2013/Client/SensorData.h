#pragma once
#include <string>

class SensorData
{
public:
	virtual ~SensorData() {};

	///@brief Convert posture information to string.
	///@param keyDelim Symbol to split KEY and VALUE.
	///@param recordDelim Symbol to split records. Record is pair of KEY and VALUE.
	///@param valueDelim Symbol to split each componet of VALUE. When VALUE is multidimensional data, you use this symbol.
	virtual std::string convertSensorData2Message(const std::string &keyDelim, const std::string &recordDelim, const std::string &valueDelim) = 0;
	//virtual void convertMessage2SensorData() = 0;
};

