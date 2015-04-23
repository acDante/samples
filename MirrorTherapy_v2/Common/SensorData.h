#ifndef SensorData_h
#define SensorData_h

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <map>
#include <iostream>
#include <sstream>

#include "DeviceCommon.h"


class SensorData
{
public:
	///@brief For expression position.
	typedef struct _Vector3
	{
		float x;
		float y;
		float z;
	} Vector3;

	///@brief For expression quaternion.
	typedef struct _Vector4
	{
		float x;
		float y;
		float z;
		float w;
	} Vector4;

	///@brief For expression position.
	typedef struct
	{
		float yaw;
		float pitch;
		float roll;
	} EulerAngleType;

//	static std::string intToStr(const int x)
//	{
//		std::stringstream ss;
//		ss << x;
//		return ss.str();
//	}

	SensorData() {};
	virtual ~SensorData() {};

	///@brief Convert sensor information to string.
	///@param itemsDelim Symbol to split items.
	///@param keyValueDelim Symbol to split KEY and VALUE.
	///@param valuesDelim Symbol to split each componet of VALUE. When VALUE is multidimensional data, you use this symbol.
	virtual std::string encodeSensorData(const std::string &itemsDelim = ITEMS_DELIMITER, const std::string &keyValueDelim = KEY_AND_VALUE_DELIMITER, const std::string &valuesDelim = VALUES_DELIMITER) const;
	
	///@brief Convert message(string) to sensor data map.
	///@param message string of message.
	///@param itemsDelim Symbol to split items.
	///@param keyValueDelim Symbol to split KEY and VALUE.
	///@param valuesDelim Symbol to split each componet of VALUE. When VALUE is multidimensional data, you use this symbol.
	virtual std::map<std::string, std::vector<std::string> > decodeSensorData(const std::string &message, const std::string &itemsDelim = ITEMS_DELIMITER, const std::string &keyValueDelim = KEY_AND_VALUE_DELIMITER, const std::string &valuesDelim = VALUES_DELIMITER) const;

	///@brief Set to sensor data.
	///@param Map of sensor data;
	virtual bool setSensorData(const std::map<std::string, std::vector<std::string> > &sensorDataMap);

	///@brief Convert message to std::map. 
	///@param itemsDelim Symbol to split items.
	///@param keyValueDelim Symbol to split KEY and VALUE.
	///@param valuesDelim Symbol to split each componet of VALUE. When VALUE is multidimensional data, you use this symbol.
	static std::map<std::string, std::vector<std::string> > convertMessage2Map(const std::string &message, const std::string &itemsDelim = ITEMS_DELIMITER, const std::string &keyValueDelim = KEY_AND_VALUE_DELIMITER, const std::string &valuesDelim = VALUES_DELIMITER);

	///@brief Convert std::map to message.
	///@param itemsDelim Symbol to split items.
	///@param keyValueDelim Symbol to split KEY and VALUE.
	///@param valuesDelim Symbol to split each componet of VALUE. When VALUE is multidimensional data, you use this symbol.
	static std::string convertMap2Message(const std::map<std::string, std::vector<std::string> > &map, const std::string &itemsDelim = ITEMS_DELIMITER, const std::string &keyValueDelim = KEY_AND_VALUE_DELIMITER, const std::string &valuesDelim = VALUES_DELIMITER);

//	///@brief Delete parentheses '(' and ')' from string.
//	static std::string deleteParenthesesFromString(const std::string &input, const std::string &pBegin = "(", const std::string &pEnd = ")");
};

#endif //SensorData_h
