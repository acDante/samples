#ifndef __OCULUS_RIFT_DK2_SENSOR_DATA_H__
#define __OCULUS_RIFT_DK2_SENSOR_DATA_H__

#include <sigverse/common/SigCmn.h>
#include <sigverse/common/device/SensorData.h>

#define MSG_KEY_QUATERNION "QUATERNION"


class OculusRiftDK2SensorData :
	public SensorData
{
private:
	SigCmn::Vector4 quaternion;

public:

	///@brief Convert sensor information to string.
	///@param itemsDelim Symbol to split items.
	///@param keyValueDelim Symbol to split KEY and VALUE.
	///@param valuesDelim Symbol to split each componet of VALUE. When VALUE is multidimensional data, you use this symbol.
	std::string encodeSensorData(const std::string &itemsDelim = ITEMS_DELIMITER, const std::string &keyValueDelim = KEY_AND_VALUE_DELIMITER, const std::string &valuesDelim = VALUES_DELIMITER) const;

	///@brief Set to sensor data.
	///@param Map of sensor data;
	bool setSensorData(const std::map<std::string, std::vector<std::string> > &sensorDataMap);

	///@brief Set sensor data.
	void setQuaternion(const float &w, const float &x, const float &y, const float &z);

	///@brief Get sensor data
	SigCmn::Vector4 getQuaternion() const;
};
#endif //__OCULUS_RIFT_DK2_SENSOR_DATA_H__
