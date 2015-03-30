#pragma once
#include <Plugin/PluginCommon/Device.h>
#include <conio.h>
#include <winsock2.h>
#include <NatNetTypes.h>
#include <NatNetClient.h>

// Default service name of this program. Must be defined.
// SIGServerとの通信で必要になるサービス名の初期値を記述する．
#define DEFAULT_SERVICE_NAME "SIGMOCAP"
static const std::string defaultServiceName = DEFAULT_SERVICE_NAME;

// Default device type name for Opti-track.
#define DEFAULT_DEVICE_TYPE "OPTITRACK"
static const std::string defaultDeviceType = DEFAULT_DEVICE_TYPE;

// Default device unique ID for Opti-track.
#define DEFAULT_DEVICE_ID "0"
static const std::string defaultDeviceUniqueID = DEFAULT_DEVICE_ID;

// Default port number for Opti-track server (motive).
#define MOTIVE_DATAPORT 3130;
#define MOTIVE_COMMANDPORT 3131;
static const unsigned int defaultMotiveDataPort = MOTIVE_DATAPORT;
static const unsigned int defaultMotiveCommandPort = MOTIVE_COMMANDPORT;

// Default buffer size for IP adrress.
#define BUFFERSIZE 128
static const int bufferSizeForIPAddress = BUFFERSIZE;

// Callback methods.

///@brief receives NatNet error mesages
void __cdecl MessageHandler(int msgType, char* msg);

///@brief Receives data from the server.
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);


class OptiTrackDevice :
	public Device
{
private:
	///@brief NatNet client structure?
	//NatNetClient *natNetClient;

	///@brief NatNet connection type
	int connectionType;

	char szMyIPAddress[bufferSizeForIPAddress];
	char szServerIPAddress[bufferSizeForIPAddress];

	sigverse::SIGService sigService;

public:
	OptiTrackDevice(){};
	OptiTrackDevice(int argc, char **argv);

	///@brief Set service name.
	void setSigServiceName() {
		this->serviceName = defaultServiceName;
	};

	///@brief 
	int createClient();

	///@brief
	void resetClient();

	///@brief Run oculus dk1 device.
	int run();

};

///@brief Send message to SIGServer.
///@param sigService Pointer to SIGService.
///@param message Message you want to send to SIGServer.
void sendMessage_(sigverse::SIGService &sigService, const std::string &message);
