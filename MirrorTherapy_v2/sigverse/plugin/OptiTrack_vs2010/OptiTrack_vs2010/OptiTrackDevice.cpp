#include <sigverse/common/device/OptiTrackSensorData.h>
#include <sigverse/plugin/common/CheckRecvSIGServiceData.h>
#include <sigverse/plugin/OptiTrack_vs2010/OptiTrack_vs2010/OptiTrackDevice.h>

#include <sstream>

#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>

///@brief NatNet client instance
NatNetClient *natNetClient;

/**
 * @brief constructer
 */
OptiTrackDevice::OptiTrackDevice(int argc, char **argv)
{
	if (argc == 3)
	{
		std::cout << "SIGServer IP address: " << argv[1] << std::endl;
		std::cout << "Port number: " << argv[2] << std::endl;
		
		this->serverAddress   = argv[1];
		this->portNumber      = atoi(argv[2]);
		this->sendMessageFlag = true;
	}
	else 
	{
		std::cout << "Please execute with SIGServer address and port number." << std::endl;
		exit(1);
	}

	this->parameterFileName = PARAM_FILE_NAME_OPTITRACK_INI;
}


/**
 * @brief establish an NatNet Client connection.
 */
int OptiTrackDevice::createNatNetClient() 
{
	strcpy_s(this->szServerIPAddress, "");
	strcpy_s(this->szMyIPAddress, "");


	this->natNetConnectionType = ConnectionType_Multicast;

	printf("%d\n", this->natNetConnectionType);

	if (::natNetClient) 
	{
		::natNetClient->Uninitialize();
		delete ::natNetClient;
	}

	// create NatNet client
	::natNetClient = new NatNetClient(this->natNetConnectionType);

	// print version info
	unsigned char ver[4];
	::natNetClient->NatNetVersion(ver);
	printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);
	
	// set callback handlers
	::natNetClient->SetMessageCallback(::MessageHandler);
	::natNetClient->SetVerbosityLevel(::Verbosity_Info);
	::natNetClient->SetDataCallback(::DataHandler, this);

	// init Client and connect to NatNet server
	// to use NatNet default port assigments
	int retCode = ::natNetClient->Initialize(this->szMyIPAddress, this->szServerIPAddress);

	// to use a different port for commands and/or data:
	//int retCode = ::natNetClient->Initialize(szMyIPAddress, szServerIPAddress, MyServersCommandPort, MyServersDataPort);
	if (retCode != ErrorCode_OK)
	{
		printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
		return ErrorCode_Internal;
	}
	else
	{
		// print server info
		sServerDescription ServerDescription;
		memset(&ServerDescription, 0, sizeof(ServerDescription));
		::natNetClient->GetServerDescription(&ServerDescription);

		if(!ServerDescription.HostPresent)
		{
			printf("Unable to connect to server. Host not present. Exiting.");
			return 1;
		}
		
		printf("[SampleClient] Server application info:\n");
		printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0], ServerDescription.HostAppVersion[1], ServerDescription.HostAppVersion[2],ServerDescription.HostAppVersion[3]);
		printf("NatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1], ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
		printf("Client IP:%s\n", szMyIPAddress);
		printf("Server IP:%s\n", szServerIPAddress);
		printf("Server Name:%s\n\n", ServerDescription.szHostComputerName);
	}

	return ErrorCode_OK;
}


/**
 * @brief main routine
 */
int OptiTrackDevice::run()
{
	//-- Prepare to use SIGService.
	this->readIniFile();

	// create sigservice.
	this->sigService = new sigverse::SIGService(this->serviceName);
	this->initializeSigService(*this->sigService);

	// check receive SIGService data by another thread
	CheckRecvSIGServiceData checkRecvSIGServiceData;
	boost::thread thCheckRecvData(&CheckRecvSIGServiceData::run, &checkRecvSIGServiceData, this->sigService);

	// create NatNet Client
	int iResult = this->createNatNetClient();

	if(iResult != ErrorCode_OK)
	{
		printf("Error initializing client.  See log for details.  Exiting");
		return 1;
	}
	else
	{
		printf("Client initialized and ready.\n");
	}

	// send/receive test request
	printf("[SampleClient] Sending Test Request\n");

	void* response;
	int nBytes;
	iResult = ::natNetClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	
	if (iResult == ErrorCode_OK)
	{
		printf("[SampleClient] Received: %s", (char*)response);
	}

	// retrieve Data Descriptions from server
	printf("\n\n[SampleClient] Requesting Data Descriptions...");
	sDataDescriptions* pDataDefs = NULL;
	int nBodies = ::natNetClient->GetDataDescriptions(&pDataDefs);
	
	if(!pDataDefs)
	{
		printf("[SampleClient] Unable to retrieve Data Descriptions.");
	}
	else
	{
		printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions );
		
		for(int i=0; i < pDataDefs->nDataDescriptions; i++)
		{
			printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
			
			if(pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
			{
				// MarkerSet
				sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
				
				printf("MarkerSet Name : %s\n", pMS->szName);
				for(int i=0; i < pMS->nMarkers; i++){ printf("%s\n", pMS->szMarkerNames[i]); }
			}
			else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
			{
				// RigidBody
				sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
				
				printf("RigidBody Name : %s\n", pRB->szName);
				printf("RigidBody ID : %d\n", pRB->ID);
				printf("RigidBody Parent ID : %d\n", pRB->parentID);
				printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
			}
			else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
			{
				// Skeleton
				sSkeletonDescription* pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
				
				printf("Skeleton Name : %s\n", pSK->szName);
				printf("Skeleton ID : %d\n", pSK->skeletonID);
				printf("RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
				
				for(int j=0; j < pSK->nRigidBodies; j++)
				{
					sRigidBodyDescription* pRB = &pSK->RigidBodies[j];
					
					printf("  RigidBody Name : %s\n", pRB->szName);
					printf("  RigidBody ID : %d\n", pRB->ID);
					printf("  RigidBody Parent ID : %d\n", pRB->parentID);
					printf("  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
				}
			}
			else
			{
				printf("Unknown data type.");
				// Unknown
			}
		}
	}
	
	// ready to receive marker stream!
	printf("\nClient is connected to server and listening for data...\n");
	int c;
	bool bExit = false;

	while(c =_getch())
	{
		switch(c)
		{
			case 'q':
			{
				bExit = true;
				break;
			}
			//case 'r':
			//{
			//	resetNatNetClient();
			//	break;
			//}
			//case 'p':
			//{
			//	sServerDescription ServerDescription;
			//	memset(&ServerDescription, 0, sizeof(ServerDescription));
			//	::natNetClient->GetServerDescription(&ServerDescription);
			//	
			//	if(!ServerDescription.HostPresent)
			//	{
			//		printf("Unable to connect to server. Host not present. Exiting.");
			//		return 1;
			//	}
			//	break;
			//}
			//case 'f':
			//{
			//	sFrameOfMocapData* pData = ::natNetClient->GetLastFrameOfData();
			//	printf("Most Recent Frame: %d", pData->iFrame);
			//	break;
			//}
			//case 'm': // change to multicast
			//{
			//	this->natNetConnectionType = ConnectionType_Multicast;
			//	iResult = this->createNatNetClient();
			//	
			//	if(iResult == ErrorCode_OK)
			//	{
			//		printf("Client connection type changed to Multicast.\n\n");
			//	}
			//	else
			//	{
			//		printf("Error changing client connection type to Multicast.\n\n");
			//	}
			//	break;
			//}
			//case 'u': // change to unicast
			//{
			//	this->natNetConnectionType = ConnectionType_Unicast;
			//	iResult = this->createNatNetClient();
			//	
			//	if(iResult == ErrorCode_OK)
			//	{
			//		printf("Client connection type changed to Unicast.\n\n");
			//	}
			//	else
			//	{
			//		printf("Error changing client connection type to Unicast.\n\n");
			//	}
			//	break;
			//}
			default:
			{
				break;
			}
		}
		
		if(bExit){ break; }
	}
	
	// Done - clean up.
	::natNetClient->Uninitialize();
	//_WriteFooter(fp);
	//fclose(fp);

	return ErrorCode_OK;
}


///@brief reset NatNet Client connection.
void OptiTrackDevice::resetNatNetClient()
{
	int iSuccess;

	printf("\n\nre-setting Client\n\n.");

	iSuccess = ::natNetClient->Uninitialize();
	if(iSuccess != 0){ printf("error un-initting Client\n"); }

	iSuccess = ::natNetClient->Initialize(szMyIPAddress, szServerIPAddress);
	if(iSuccess != 0){ printf("error re-initting Client\n"); }
}


///@brief receives data from the server.
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	OptiTrackDevice* otDev = (OptiTrackDevice*) pUserData;
	
	OptiTrackSensorData::sRigidBodyDataSgv tmpRigidBodies[MAX_RIGIDBODIES];

	memcpy(tmpRigidBodies, data->RigidBodies, sizeof(data->RigidBodies));

	OptiTrackSensorData otSensor;

	otSensor.setSensorData(data->nRigidBodies, tmpRigidBodies);

	std::string sensorDataStr = otSensor.encodeSensorData();

	if(sensorDataStr==""){ return; }

	std::ostringstream ss;
	
	ss << otDev->generateMessageHeader() << sensorDataStr;

	otDev->sendMessage(ss.str().c_str());
}


///@brief receives NatNet error mesages
void __cdecl MessageHandler(int msgType, char* msg)
{
	printf("\n%s\n", msg);
}


///@breaf send message to SIGServer
void OptiTrackDevice::sendMessage(const std::string &message)
{
	try 
	{
		std::vector<std::string> connectedEntitiesName = sigService->getAllConnectedEntitiesName();
			
		for (int i = 0; i < (int)connectedEntitiesName.size(); i++) 
		{
			sigService->sendMsgToCtr(connectedEntitiesName[i].c_str(), message);
		}
		
//		std::cout << "msg:" << message << std::endl;
	}
	catch (...) 
	{
		std::cout << "catch (...) in OptiTrackDevice::sendMessage" << std::endl;
		Sleep(20000);
	}
}


///@brief Read parameter file.
///@return When couldn't read parameter file, return false;
void OptiTrackDevice::readIniFile()
{
	std::ifstream ifs(this->parameterFileName);

	// Parameter file is "not" exists.
	if (ifs.fail()) 
	{
		std::cout << "Not exist : " << this->parameterFileName << std::endl;
		std::cout << "Use default parameter." << std::endl;

		this->serviceName    = SERVICE_NAME_OPTITRACK;
		this->deviceType     = DEV_TYPE_OPTITRACK;
		this->deviceUniqueID = DEV_UNIQUE_ID_0;
	}
	// Parameter file is exists.
	else 
	{
		try 
		{
			std::cout << "Read " << this->parameterFileName << std::endl;
			boost::property_tree::ptree pt;
			boost::property_tree::read_ini(this->parameterFileName, pt);

			this->serviceName    = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME);
			this->deviceType     = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE);
			this->deviceUniqueID = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID);
		}
		catch (boost::exception &ex) 
		{
			std::cout << this->parameterFileName << " ERR :" << *boost::diagnostic_information_what(ex) << std::endl;
		}
	}

	std::cout << PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME     << ":" << this->serviceName    << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE      << ":" << this->deviceType     << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID << ":" << this->deviceUniqueID << std::endl;
}

