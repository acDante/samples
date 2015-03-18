#include "OculusDK1SensorData.h"
#include <sstream>
std::string OculusDK1SensorData::convertSensorData2Message(const std::string &keyDelim, const std::string &recordDelim, const std::string &valueDelim)
{
	std::stringstream ss;
	ss << "EULER" << keyDelim << 
		"(" << this->eulerAngle.yaw << valueDelim << this->eulerAngle.pitch << valueDelim << this->eulerAngle.roll << ")" << recordDelim;
	return ss.str();
}

///@brief Set sensor data.
void OculusDK1SensorData::setEulerAngle(const float &yaw, const float &pitch, const float &roll)
{
	this->eulerAngle.yaw = yaw;
	this->eulerAngle.pitch = pitch;
	this->eulerAngle.roll = roll;
}
