#pragma once
#include "SIGService.h"
#include "SensorData.h"


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
};

