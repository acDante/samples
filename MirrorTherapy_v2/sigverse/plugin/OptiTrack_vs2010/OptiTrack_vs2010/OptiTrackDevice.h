#ifndef __OPTITRACK_DEVICE_H__
#define __OPTITRACK_DEVICE_H__

#include "../../PluginCommon/Device.h"

#include <conio.h>
#include <winsock2.h>
#include <NatNetTypes.h>
#include <NatNetClient.h>

// Parameter file name. Must be defined.
#define PARAM_FILE_NAME_OPTITRACK_INI  "OptiTrack.ini"

// Default buffer size for IP adrress.
#define BUFFER_SIZE_FOR_IP_ADDRESS 128

// Callback methods.

///@brief receives NatNet error mesages
void __cdecl MessageHandler(int msgType, char* msg);

///@brief receives data from the server.
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);


class OptiTrackDevice : public Device
{
private:
	///@brief Parameter file name.
	std::string parameterFileName;

	///@brief NatNet connection type
	int natNetConnectionType;

	char szMyIPAddress    [BUFFER_SIZE_FOR_IP_ADDRESS];
	char szServerIPAddress[BUFFER_SIZE_FOR_IP_ADDRESS];

	///@brief Read parameter file.
	///@return Success flag to read parameter.
	void readIniFile();

public:
	OptiTrackDevice(int argc, char **argv);

	///@brief establish NatNet Client connection.
	int createNatNetClient();

	///@brief reset NatNet Client connection.
	void resetNatNetClient();

	///@brief run OptiTrack device.
	int run();

	///@brief Send message to SIGServer.
	///@param message Message you want to send to SIGServer.
	void sendMessage(const std::string &message);

	///@brief NatNet client instance
//	NatNetClient *natNetClient;

	///@brief SIGService instance 
	sigverse::SIGService *sigService;
};

#endif // __OPTITRACK_DEVICE_H__
