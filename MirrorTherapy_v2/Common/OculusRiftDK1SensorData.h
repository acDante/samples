#pragma once
#include <Common\SensorData.h>

#ifndef _EulerAngleType_
#define _EulerAngleType_

#define SENSOR_KEY "EULER"
static const std::string sensorKey = SENSOR_KEY;

///@brief For expression position.
typedef struct
{
	float yaw;
	float pitch;
	float roll;
} EulerAngleType;
#endif

class OculusRiftDK1SensorData :
	public SensorData
{
private:
	EulerAngleType eulerAngle;

public:

	///@brief Convert posture information to string.
	std::string encodeSensorData2Message(const std::string &keyDelim, const std::string &recordDelim, const std::string &valueDelim);

	void decodeMessage2SensorData(const std::string &message, const std::string &keyDelim, const std::string &recordDelim, const std::string &valueDelim);

	///@brief Set sensor data.
	void setEulerAngle(const float &yaw, const float &pitch, const float &roll);

	///@brief Get sensor data.
	float yaw() {
		return this->eulerAngle.yaw;
	}
	float pitch() {
		return this->eulerAngle.pitch;
	}
	float roll() {
		return this->eulerAngle.roll;
	}
};
