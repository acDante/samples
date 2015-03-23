#pragma once
#include <SIGService.h>
#include <Common/SensorData.h>

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
	///@param sigService Pointer to SIGService.
	virtual void initializeSigService(sigverse::SIGService &sigService);

	///@brief Read parameter file.
	///@return Success flag to read parameter.
	virtual bool readIniFile();

	///@brief Set service name.
	virtual void setSigServiceName() = 0;

	///@brief Send message to SIGServer.
	///@param sigService Pointer to SIGService.
	///@param message Message you want to send to SIGServer.
	virtual const void sendMessage(sigverse::SIGService &sigService, const std::string &message);
	// メッセージの生成規則について
	// (1) 基本的に，「KEY:VALUE;」というKEYとVALUEのペアが「;」で区切られているものを必要なだけ繋げてメッセージにします．
	//		例)　DEV_TYPE:KINECTV2;DEV_ID:0;
	//		説明)　KEYがDEV_TYPEで，VALUEがKINECTV2のペアと，KEYがDEV_IDで，VALUEが0の2つのペアをメッセージにする場合の例．
	// (2) VALUEがベクトル(多次元)の場合，値ごとに「,」で区切り，先頭に「(」，末尾に「)」を付けます．
	//		例)　EULER:(0.3,-0.2,1.0);
	// (3) メッセージの冒頭にはKEYが「DEV_TYPE」のペアとKEYが「DEV_ID」のペアの，二つのペアの情報を含むべきです．
	// (4) メッセージ中にはスペースを含むべきではないと考えます．

	///@brief Get device type. For example, device type is "KINECTV2".
	const std::string getDeviceType() {
		return this->deviceType;
	}

	///@brief Get device unique ID.
	const std::string getDeviceUniqueID() {
		return this->deviceUniqueID;
	}

	void setDeviceType(const std::string &deviceTypeString) {
		this->deviceType = deviceTypeString;
	}

	void setDeviceUniqueID(const std::string &deviceUniqueIDString) {
		this->deviceUniqueID = deviceUniqueIDString;
	}

	///@brief Generate message header that is made by device type and unique ID.
	std::string generateMessageHeader() {
		const std::string tmp = devTypeKey + keyDelimDefault + this->deviceType + recordDelimDefault + devIDKey + keyDelimDefault + this->deviceUniqueID + recordDelimDefault;
		return tmp;
	}
};

