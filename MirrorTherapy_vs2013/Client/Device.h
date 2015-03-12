#pragma once
#include "SIGService.h"
#include "SensorData.h"

// Define delimiters to split string.
#define KEY_DELIMITER ":"
#define RECORD_DELIMITER ";"
#define VALUE_DELIMITER ","
static const std::string keyDelimDefault = KEY_DELIMITER;
static const std::string recordDelimDefault = RECORD_DELIMITER;
static const std::string valueDelimDefault = VALUE_DELIMITER;

// Define message header keys.
#define DEV_TYPE_KEY "DEV_TYPE"
#define DEV_ID_KEY "DEV_ID"
static const std::string devTypeKey = DEV_TYPE_KEY;
static const std::string devIDKey = DEV_ID_KEY;

class Device
{
protected:

	///@brief Sensor data.
	SensorData *sensorData;
	
	///@brief Device type.
	std::string deviceType;

	///@brief Device unique ID.
	std::string deviceUniqueID;

	///@brief Parameter file name.
	std::string parameterFileName;

	///@brief Command line parameter. IP address.
	std::string serverAddress;

	///@brief Command line parameter. Port number.
	int portNumber;

	///@brief Service name on SIGService.
	std::string serviceName;

	///@brief Flag of send message to server.
	bool sendMessageFlag;

public:

	virtual ~Device() {};

	///@brief Initialize SIGService.
	virtual void initializeSigService(sigverse::SIGService &sigService);

	///@brief Read parameter file.
	virtual bool readIniFile();

	///@brief Set service name.
	virtual void setSigServiceName() = 0;

	virtual const void sendMessage(sigverse::SIGService &sigService, const std::string &message);

	const std::string getDeviceType() {
		return this->deviceType;
	}

	const std::string getDeviceUniqueID() {
		return this->deviceUniqueID;
	}

	void setDeviceType(const std::string &deviceTypeString) {
		this->deviceType = deviceTypeString;
	}

	void setDeviceUniquID(const std::string &deviceUniqueIDString) {
		this->deviceUniqueID = deviceUniqueIDString;
	}

	std::string generateMessageHeader() {
		const std::string tmp = devTypeKey + keyDelimDefault + this->deviceType + recordDelimDefault + devIDKey + keyDelimDefault + this->deviceUniqueID + recordDelimDefault;
		return tmp;
	}
};

