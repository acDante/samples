#ifndef __DEVICE_H__
#define __DEVICE_H__

#include <SIGService.h>
#include "../../common/device/DeviceCommon.h"
#include "../../common/device/SensorData.h"


class Device
{
protected:
	///@brief Device type.
	std::string deviceType;

	///@brief Device unique ID.
	std::string deviceUniqueID;

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
	///@param sigService Pointer to SIGService.
	virtual void initializeSigService(sigverse::SIGService &sigService);


	///@brief Send message to SIGServer.
	///@param sigService Pointer to SIGService.
	///@param message Message you want to send to SIGServer.
	virtual const void sendMessage(sigverse::SIGService &sigService, const std::string &message);


	///@brief Get device type. For example, device type is "KINECTV2".
	const std::string getDeviceType() 
	{
		return this->deviceType;
	}

	///@brief Get device unique ID.
	const std::string getDeviceUniqueID() 
	{
		return this->deviceUniqueID;
	}

	///@brief Set device type.
	void setDeviceType(const std::string &deviceTypeString) 
	{
		this->deviceType = deviceTypeString;
	}

	///@brief Set device unique ID.
	void setDeviceUniqueID(const std::string &deviceUniqueIDString) 
	{
		this->deviceUniqueID = deviceUniqueIDString;
	}

	///@brief Generate message header that is made by device type and unique ID.
	std::string generateMessageHeader(const std::string &itemsDelim = ITEMS_DELIMITER, const std::string &keyValueDelim = KEY_AND_VALUE_DELIMITER, const std::string &valuesDelim = VALUES_DELIMITER) const
	{
		return MSG_KEY_DEV_TYPE + keyValueDelim + this->deviceType     + itemsDelim + 
			   MSG_KEY_DEV_UNIQUE_ID   + keyValueDelim + this->deviceUniqueID + itemsDelim;
	}
};

#endif //__DEVICE_H__
