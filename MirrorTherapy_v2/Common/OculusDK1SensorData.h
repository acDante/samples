#pragma once
#include "sensordata.h"

#ifndef _EulerAngleType_
#define _EulerAngleType_
///@brief For expression position.
typedef struct _EulerAngleType
{
	float yaw;
	float pitch;
	float roll;
} 	EulerAngleType;
#endif

class OculusDK1SensorData :
	public SensorData
{
private:
	EulerAngleType eulerAngle;

public:

	///@brief Convert posture information to string.
	std::string convertSensorData2Message(const std::string &keyDelim, const std::string &recordDelim, const std::string &valueDelim);

	///@brief Set sensor data.
	void setEulerAngle(const float &yaw, const float &pitch, const float &roll);
};

