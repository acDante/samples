#include "OptiTrackDevice.h"
#include <sstream>

///TODO: natNetClientとsigService が大域変数になっている．OptTrackDevice のメンバ変数にするべき．

///@brief NatNet client structure?
NatNetClient *natNetClient;

sigverse::SIGService *sigService;

OptiTrackDevice::OptiTrackDevice(int argc, char **argv)
{
	if (argc == 3) {
		std::cout << "SIGServer IP address: " << argv[1] << std::endl;
		std::cout << "Port number: " << argv[2] << std::endl;
		this->sendMessageFlag = true;
		this->serverAddress = argv[1];
		this->portNumber = atoi(argv[2]);
		this->setDeviceType(defaultDeviceType);
		this->setDeviceUniqueID(defaultDeviceUniqueID);
		//this->sigService = sigverse::SIGService(this->serviceName);
	} else {
		//std::cout << "Please execute with SIGServer address and port number." << std::endl;
		//exit(1);
		this->setDeviceType(defaultDeviceType);
		this->setDeviceUniqueID(defaultDeviceUniqueID);
	}
}

///@brief Establish an NatNet Client connection.
int OptiTrackDevice::createClient() 
{
	strcpy_s(this->szServerIPAddress, "");
	strcpy_s(this->szMyIPAddress, "");


	this->connectionType = ConnectionType_Multicast;

	printf("%d\n", this->connectionType);

	if (natNetClient) {
		natNetClient->Uninitialize();
		delete natNetClient;
	}

	// Create NatNet client
	natNetClient = new NatNetClient(this->connectionType);

    // print version info
    unsigned char ver[4];
	natNetClient->NatNetVersion(ver);
    printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);
	
	// Set callback handlers
	natNetClient->SetMessageCallback(::MessageHandler);
	natNetClient->SetVerbosityLevel(::Verbosity_Debug);
	natNetClient->SetDataCallback(DataHandler, natNetClient);

	    // Init Client and connect to NatNet server
    // to use NatNet default port assigments
	int retCode = natNetClient->Initialize(this->szMyIPAddress, this->szServerIPAddress);
    // to use a different port for commands and/or data:
    //int retCode = this->natNetClient->Initialize(szMyIPAddress, szServerIPAddress, MyServersCommandPort, MyServersDataPort);
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
		natNetClient->GetServerDescription(&ServerDescription);
        if(!ServerDescription.HostPresent)
        {
            printf("Unable to connect to server. Host not present. Exiting.");
            return 1;
        }
        printf("[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
            ServerDescription.HostAppVersion[1],ServerDescription.HostAppVersion[2],ServerDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1],
            ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", szMyIPAddress);
        printf("Server IP:%s\n", szServerIPAddress);
        printf("Server Name:%s\n\n", ServerDescription.szHostComputerName);
    }

	return ErrorCode_OK;
}

int OptiTrackDevice::run()
{
	
	// Create sigservice.
	this->setSigServiceName();
	sigverse::SIGService tmpSigService = sigverse::SIGService::SIGService(this->serviceName);
	this->initializeSigService(tmpSigService);
	this->sigService = tmpSigService;
	::sigService = &this->sigService;

	//sendMessage_(this->sigService, "hogehoge");

	//printf("hogehogehoge\n");

	// Create NatNet Client
    int iResult = this->createClient();
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
	iResult = natNetClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("[SampleClient] Received: %s", (char*)response);
	}

	// Retrieve Data Descriptions from server
	printf("\n\n[SampleClient] Requesting Data Descriptions...");
	sDataDescriptions* pDataDefs = NULL;
	int nBodies = natNetClient->GetDataDescriptions(&pDataDefs);
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
                for(int i=0; i < pMS->nMarkers; i++)
                    printf("%s\n", pMS->szMarkerNames[i]);

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

	
	// Create data file for writing received stream into
	//char szFile[MAX_PATH];
	//char szFolder[MAX_PATH];
	//const char *szFolder = boost::filesystem::current_path().string().c_str();
	//GetCurrentDirectory(MAX_PATH, szFolder);
	
	//if(argc > 3)
		//sprintf(szFile, "%s\\%s", szFolder, argv[3]);
		//sprintf_s(szFile, "%s\\%s", szFolder, argv[3]);
	//else
		//sprintf(szFile, "%s\\Client-output.pts",szFolder);
		//sprintf_s(szFile, "%s\\Client-output.pts",szFolder);
	//fp = fopen(szFile, "w");
	//fopen_s(&fp, szFile, "W");
	//if(!fp)
	//{
	//	printf("error opening output file %s.  Exiting.", szFile);
	//	exit(1);
	//}
	//if(pDataDefs)
	//	_WriteHeader(fp, pDataDefs);

	// Ready to receive marker stream!
	printf("\nClient is connected to server and listening for data...\n");
	int c;
	bool bExit = false;
	while(c =_getch())
	{
		switch(c)
		{
			case 'q':
				bExit = true;		
				break;	
			case 'r':
				resetClient();
				break;	
            case 'p':
                sServerDescription ServerDescription;
                memset(&ServerDescription, 0, sizeof(ServerDescription));
                natNetClient->GetServerDescription(&ServerDescription);
                if(!ServerDescription.HostPresent)
                {
                    printf("Unable to connect to server. Host not present. Exiting.");
                    return 1;
                }
                break;	
            case 'f':
                {
                    sFrameOfMocapData* pData = natNetClient->GetLastFrameOfData();
                    printf("Most Recent Frame: %d", pData->iFrame);
                }
                break;	
            case 'm':	                        // change to multicast
				this->connectionType = ConnectionType_Multicast;
                iResult = this->createClient();
                if(iResult == ErrorCode_OK)
                    printf("Client connection type changed to Multicast.\n\n");
                else
                    printf("Error changing client connection type to Multicast.\n\n");
                break;
            case 'u':	                        // change to unicast
				this->connectionType = ConnectionType_Unicast;
                iResult = this->createClient();
                if(iResult == ErrorCode_OK)
                    printf("Client connection type changed to Unicast.\n\n");
                else
                    printf("Error changing client connection type to Unicast.\n\n");
                break;


			default:
				break;
		}
		if(bExit)
			break;
	}

	// Done - clean up.
	natNetClient->Uninitialize();
	//_WriteFooter(fp);
	//fclose(fp);

	return ErrorCode_OK;

}


void OptiTrackDevice::resetClient()
{
	int iSuccess;

	printf("\n\nre-setting Client\n\n.");

	iSuccess = natNetClient->Uninitialize();
	if(iSuccess != 0)
		printf("error un-initting Client\n");

	iSuccess = natNetClient->Initialize(szMyIPAddress, szServerIPAddress);
	if(iSuccess != 0)
		printf("error re-initting Client\n");
}


///@brief Receives data from the server.
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	NatNetClient* pClient = (NatNetClient*) pUserData;

	//if(fp)
	//	_WriteFrame(fp,data);
	
    int i=0;

    //printf("FrameID : %d\n", data->iFrame);
    //printf("Timestamp :  %3.2lf\n", data->fTimestamp);
    //printf("Latency :  %3.2lf\n", data->fLatency);
    
    // FrameOfMocapData params
    //bool bIsRecording = data->params & 0x01;  // これだと警告が出る．int(ここではshort)型をboolにしているから．
    //bool bTrackedModelsChanged = data->params & 0x02; 
	bool bIsRecording = (data->params & 0x01) != 0; 
    bool bTrackedModelsChanged = (data->params & 0x02) != 0;
    //if(bIsRecording)
    //    printf("RECORDING\n");
    //if(bTrackedModelsChanged)
    //    printf("Models Changed.\n");
	
        
    // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
	int hour, minute, second, frame, subframe;
	bool bValid = pClient->DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
	// decode to friendly string
	char szTimecode[128] = "";
	pClient->TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
	//printf("Timecode : %s\n", szTimecode);

	//// Other Markers
	//printf("Other Markers [Count=%d]\n", data->nOtherMarkers);
	//for(i=0; i < data->nOtherMarkers; i++)
	//{
	//	printf("Other Marker %d : %3.2f\t%3.2f\t%3.2f\n",
	//		i,
	//		data->OtherMarkers[i][0],
	//		data->OtherMarkers[i][1],
	//		data->OtherMarkers[i][2]);
	//}

	// Rigid Bodies
	//printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
	for(i=0; i < data->nRigidBodies; i++)
	{
        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame
        bool bTrackingValid = data->RigidBodies[i].params & 0x01;

		//printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
		//printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
		//printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
		//	data->RigidBodies[i].x,
		//	data->RigidBodies[i].y,
		//	data->RigidBodies[i].z,
		//	data->RigidBodies[i].qx,
		//	data->RigidBodies[i].qy,
		//	data->RigidBodies[i].qz,
		//	data->RigidBodies[i].qw);

		//sendMessage_(*::sigService, "HOGEHOGE");
		
		std::ostringstream ss;
		ss << "DEV_TYPE:OPTITRACK;" << "DEV_ID:0;" <<
			"QUATERNION:(" << data->RigidBodies[i].qw << "," << data->RigidBodies[i].qx << "," << data->RigidBodies[i].qy << "," << data->RigidBodies[i].qz << ");";

		sendMessage_(::sigService, ss.str().c_str());

		//printf("\tRigid body markers [Count=%d]\n", data->RigidBodies[i].nMarkers);
		//for(int iMarker=0; iMarker < data->RigidBodies[i].nMarkers; iMarker++)
		//{
  //          printf("\t\t");
  //          if(data->RigidBodies[i].MarkerIDs)
  //              printf("MarkerID:%d", data->RigidBodies[i].MarkerIDs[iMarker]);
  //          if(data->RigidBodies[i].MarkerSizes)
  //              printf("\tMarkerSize:%3.2f", data->RigidBodies[i].MarkerSizes[iMarker]);
  //          if(data->RigidBodies[i].Markers)
  //              printf("\tMarkerPos:%3.2f,%3.2f,%3.2f\n" ,
  //                  data->RigidBodies[i].Markers[iMarker][0],
  //                  data->RigidBodies[i].Markers[iMarker][1],
  //                  data->RigidBodies[i].Markers[iMarker][2]);
  //      }
	}

	//// skeletons
	//printf("Skeletons [Count=%d]\n", data->nSkeletons);
	//for(i=0; i < data->nSkeletons; i++)
	//{
	//	sSkeletonData skData = data->Skeletons[i];
	//	printf("Skeleton [ID=%d  Bone count=%d]\n", skData.skeletonID, skData.nRigidBodies);
	//	for(int j=0; j< skData.nRigidBodies; j++)
	//	{
	//		sRigidBodyData rbData = skData.RigidBodyData[j];
	//		printf("Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
	//			rbData.ID, rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy, rbData.qz, rbData.qw );

	//		printf("\tRigid body markers [Count=%d]\n", rbData.nMarkers);
	//		for(int iMarker=0; iMarker < rbData.nMarkers; iMarker++)
	//		{
	//			printf("\t\t");
	//			if(rbData.MarkerIDs)
	//				printf("MarkerID:%d", rbData.MarkerIDs[iMarker]);
	//			if(rbData.MarkerSizes)
	//				printf("\tMarkerSize:%3.2f", rbData.MarkerSizes[iMarker]);
	//			if(rbData.Markers)
	//				printf("\tMarkerPos:%3.2f,%3.2f,%3.2f\n" ,
	//				data->RigidBodies[i].Markers[iMarker][0],
	//				data->RigidBodies[i].Markers[iMarker][1],
	//				data->RigidBodies[i].Markers[iMarker][2]);
	//		}
	//	}
	//}

	//// labeled markers
 //   bool bOccluded;     // marker was not visible (occluded) in this frame
 //   bool bPCSolved;     // reported position provided by point cloud solve
 //   bool bModelSolved;  // reported position provided by model solve
	//printf("Labeled Markers [Count=%d]\n", data->nLabeledMarkers);
	//for(i=0; i < data->nLabeledMarkers; i++)
	//{
 //       //bOccluded = data->LabeledMarkers[i].params & 0x01;
 //       //bPCSolved = data->LabeledMarkers[i].params & 0x02;
 //       bOccluded = (data->LabeledMarkers[i].params & 0x01) != 0;
 //       bPCSolved = (data->LabeledMarkers[i].params & 0x02) != 0;
 //       bModelSolved = (data->LabeledMarkers[i].params & 0x04) != 0;
	//	sMarker marker = data->LabeledMarkers[i];
	//	printf("Labeled Marker [ID=%d, Occluded=%d, PCSolved=%d, ModelSolved=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
 //           marker.ID, bOccluded, bPCSolved, bModelSolved,  marker.size, marker.x, marker.y, marker.z);
	//}


}

///@brief receives NatNet error mesages
void __cdecl MessageHandler(int msgType, char* msg)
{
	printf("\n%s\n", msg);
}

void sendMessage_(sigverse::SIGService *sigService, const std::string &message)
{
	try {
		if (!sigService->checkRecvData(0)) {
			std::cout << "SIGService::chkRecvData ERR." << std::endl;
		}
		else {
			std::vector<std::string> connectedEntitiesName = sigService->getAllConnectedEntitiesName();
			for (int i = 0; i < (int)connectedEntitiesName.size(); i++) {
				sigService->sendMsgToCtr(connectedEntitiesName[i].c_str(), message);
			}
		}
	}
	catch (...) {
		std::cout << "catch (...) " << std::endl;
		Sleep(2000);
	}
}