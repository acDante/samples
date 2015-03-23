#include "OculusRiftDK1SensorData.h"
#include <sstream>
std::string OculusRiftDK1SensorData::encodeSensorData2Message(const std::string &keyDelim, const std::string &recordDelim, const std::string &valueDelim)
{
	std::stringstream ss;
	ss << sensorKey << keyDelim << 
		"(" << this->eulerAngle.yaw << valueDelim << this->eulerAngle.pitch << valueDelim << this->eulerAngle.roll << ")" << recordDelim;
	return ss.str();
}

void OculusRiftDK1SensorData::decodeMessage2SensorData(const std::string &message, const std::string &keyDelim, const std::string &recordDelim, const std::string &valueDelim)
{
	// Generate map<string, string> from message. Just split message by ";" and ":".
	std::map<std::string, std::string> messageMap = this->convertMessage2Map(message, keyDelim, recordDelim);

	// Get values (vector data) from euler value "(yaw, pitch, roll)".
	const std::string valuesString = messageMap[sensorKey];

	try {
		// 「(」と「)」を削除する．
		// Delete "(" and ")" from message.
		const std::string valuesStringDeletedParentheses = this->deleteParenthesesFromString(valuesString);

		// Split values by ",".
		std::vector<std::string> tmpValuesStringVector;
		boost::split(tmpValuesStringVector, valuesStringDeletedParentheses, boost::is_any_of(valueDelim));
	
		// 3次元の値でないときは，入力データを出力して終了
		if((int)tmpValuesStringVector.size() != 3) {
			std::cout << "Not euler value: " << valuesString << std::endl;
			return;
		}

		// Euler角を抽出してセットする．
		// Extract euler angle.
		const float yaw		 = atof(tmpValuesStringVector[0].c_str());
		const float pitch	 = atof(tmpValuesStringVector[1].c_str());
		const float roll	 = atof(tmpValuesStringVector[2].c_str());
		// Set extracted angle to sensor data.
		this->setEulerAngle(yaw, pitch, roll);
	}
	catch (std::exception &ex) {
		std::cout << ex.what() << std::endl;
	}

}

///@brief Set sensor data.
void OculusRiftDK1SensorData::setEulerAngle(const float &yaw, const float &pitch, const float &roll)
{
	this->eulerAngle.yaw = yaw;
	this->eulerAngle.pitch = pitch;
	this->eulerAngle.roll = roll;
}
