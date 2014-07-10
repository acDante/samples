#include<stdio.h>
#include<stdlib.h>

#include"SIGService.h"
#include <windows.h>
#include "OVR.h"
#include <iostream>
#include <conio.h>
#include <winsock2.h>



using namespace OVR;

#undef new  // reset the redefinition in OVR*.h headers to normal one


struct IPv4{

  int b1,b2,b3,b4;
 };

class OculusRiftService : public sigverse::SIGService{
	Ptr<DeviceManager>  pManager;
	Ptr<HMDDevice>      pHMD;
	Ptr<SensorDevice>   pSensor;
	SensorFusion*       pFusionResult;
	HMDInfo             Info;
	bool                InfoLoaded;

public:
    OculusRiftService(std::string name) : SIGService(name){};
    ~OculusRiftService();

	// initialization for whole procedure on this service
	void onInit();

	// get information of Oculus Rift
	void getInfoOculusRift();

	// function to be called if this service receives messages
	//void onRecvMsg(sigverse::RecvMsgEvent &evt);
	
	// function to be called periodically
	double onAction();
	
	// an utility function for conversion of data type from float to string.
	std::string DoubleToString(float x);	
  //  std::string getMyIP(struct IPv4* myIP);

};




std::string getMyIP(struct IPv4* myIP )
{
    char szBuffer[1024];
    std::string str = "";

    WSADATA wsaData;
    WORD wVersionRequested = MAKEWORD(2, 0);
    if(WSAStartup(wVersionRequested, &wsaData) != 0)
        return false;



    if(gethostname(szBuffer, sizeof(szBuffer)) == SOCKET_ERROR)
    {

      WSACleanup();

      return false;
    }

   hostent *host = gethostbyname(szBuffer);
    if(host == NULL)
    {

      WSACleanup();

      return false;
    }
    in_addr addr;
    //Obtain the computer's IP
   // for (int i = 0; host->h_addr_list[i] != 0; ++i)
   // {

        memcpy(&addr, host->h_addr_list[0], sizeof(in_addr));
    //    std::cout << "IPv4 Address:  " << inet_ntoa(addr) << std::endl;



  //  }

    myIP->b1 = (int)addr.S_un.S_un_b.s_b1;
    myIP->b2 = (int)addr.S_un.S_un_b.s_b2;
    myIP->b3 = (int)addr.S_un.S_un_b.s_b3;
    myIP->b4 = (int)addr.S_un.S_un_b.s_b4;
	



	char tm[32];
	sprintf_s(tm, 32, "%d", myIP->b1);
	 str += std::string(tm);
	 str += ".";

	
	sprintf_s(tm, 32, "%d", myIP->b2);
    str += std::string(tm);
	str += ".";

	
	sprintf_s(tm, 32, "%d", myIP->b3);
    str += std::string(tm);
	str += ".";

	
	sprintf_s(tm, 32, "%d", myIP->b4);
	str += std::string(tm);
	str += "|";

//std::cout << "IPv4 Address:  " << str << std::endl;

  //  WSACleanup();
    return str;
}


void OculusRiftService::onInit(){
	// initialization(prepare varibles for data acquisition of OculusRift's sensors)
	OVR::System::Init();
     std::string Service_name ="";
     IPv4 ip;
     std::string str ="";
      str = getMyIP(&ip);
      if( str == "" )
        {
          std::cout <<"Theres been an error !" << std::endl;
      
        }

   Service_name = "SIGORS ";
   Service_name += str;
  // sendMsg("multiuserctr_0",Service_name);


	pFusionResult = new SensorFusion();
	pManager = *DeviceManager::Create();

	pHMD = *pManager->EnumerateDevices<HMDDevice>().CreateDevice();

	if (pHMD){
		InfoLoaded = pHMD->GetDeviceInfo(&Info);
		pSensor = *pHMD->GetSensor();
	}
	else{
	   pSensor = *pManager->EnumerateDevices<SensorDevice>().CreateDevice();
	}

	if (pSensor){
	   pFusionResult->AttachToSensor(pSensor);
	}
}

void OculusRiftService::getInfoOculusRift(){
	//display info. of attached Oculus Rift
	//---------------------------------------------------------------
	std::cout << "----- Oculus Console -----" << std::endl;

	if (pHMD){
		std::cout << " [x] HMD Found" << std::endl;
	}
	else{
		std::cout << " [ ] HMD Not Found" << std::endl;
	}

	if (pSensor){
		std::cout << " [x] Sensor Found" << std::endl;
	}
	else{
		std::cout << " [ ] Sensor Not Found" << std::endl;
	}

	std::cout << "--------------------------" << std::endl;

	if (InfoLoaded){
		std::cout << " DisplayDeviceName: " << Info.DisplayDeviceName << std::endl;
		std::cout << " ProductName: " << Info.ProductName << std::endl;
		std::cout << " Manufacturer: " << Info.Manufacturer << std::endl;
		std::cout << " Version: " << Info.Version << std::endl;
		std::cout << " HResolution: " << Info.HResolution<< std::endl;
		std::cout << " VResolution: " << Info.VResolution<< std::endl;
		std::cout << " HScreenSize: " << Info.HScreenSize<< std::endl;
		std::cout << " VScreenSize: " << Info.VScreenSize<< std::endl;
		std::cout << " VScreenCenter: " << Info.VScreenCenter<< std::endl;
		std::cout << " EyeToScreenDistance: " << Info.EyeToScreenDistance << std::endl;
		std::cout << " LensSeparationDistance: " << Info.LensSeparationDistance << std::endl;
		std::cout << " InterpupillaryDistance: " << Info.InterpupillaryDistance << std::endl;
		std::cout << " DistortionK[0]: " << Info.DistortionK[0] << std::endl;
		std::cout << " DistortionK[1]: " << Info.DistortionK[1] << std::endl;
		std::cout << " DistortionK[2]: " << Info.DistortionK[2] << std::endl;
		std::cout << "--------------------------" << std::endl;
	}
}

OculusRiftService::~OculusRiftService(){
	// shutdown SIGService
    this->disconnect();
	
	// finalize Oculus Rift connection
	pSensor.Clear();
	pHMD.Clear();
	pManager.Clear();
	delete pFusionResult;
	OVR::System::Destroy();
}

//periodic procedure for sending messages to the controller
double OculusRiftService::onAction(){
  
     IPv4 ip;
     std::string str ="";
    str = getMyIP(&ip);
    if( str == "" )
        {
          std::cout <<"Theres been an error !" << std::endl;
      
        }
    // else
    //    std::cout << "IPv4 address stored in struct: ";
    //    std::cout << ip.b1<<"."<<ip.b2<<"."<<ip.b3<<"."<<ip.b4<< std::endl;
    
	float r_yaw, r_pitch, r_roll;

	Quatf q = pFusionResult->GetOrientation();
	Matrix4f bodyFrameMatrix(q);
	q.GetEulerAngles<Axis_Y, Axis_X, Axis_Z>(&r_yaw, &r_pitch, &r_roll);

	std::vector<std::string> names;
	names = this->getAllConnectedEntitiesName();
	int entSize = names.size();

   // std::cout << "IPv4 Address:  " << str << std::endl;
   // std::cout << " entity " <<  entSize << std::endl;


	for(int i = 0; i < entSize; i++) {
    std::string msg = "ORS_DATA ";	
	    msg += str;
		msg += DoubleToString(r_yaw);
		msg += DoubleToString(r_pitch);
		msg += DoubleToString(r_roll);

		this->sendMsgToCtr(names[i], msg);
    std::cout << msg << std::endl;
	}


	return 0.1;  //time period
}

// utility function to convert data type from 'double' to 'string'
std::string OculusRiftService::DoubleToString(float x){
	char tmp[32];
	sprintf_s(tmp, 32, "%.3f", x);
	std::string str = std::string(tmp);
	str += ",";
	return str;
}

//entry point
void main(int argc, char* argv[]){
	// create instance
     std::string Service_name ="";
     IPv4 ip;
     std::string str ="";
    str = getMyIP(&ip);
    if( str == "" )
        {
          std::cout <<"Theres been an error !" << std::endl;
      
        }



   Service_name = "SIGORS ";
   Service_name += str;
  
	OculusRiftService srv(Service_name);
	char saddr[128];
	unsigned int portnumber;
	if (argc > 1) {
		sprintf_s(saddr, 128, "%s", argv[1]);
		portnumber = (unsigned int)atoi(argv[2]);
	}
	else{
		exit(0);
	}


	// connect to SIGServer
	if(srv.connect(saddr, portnumber)){
  
	// connect to SIGViewer
		if(srv.connectToViewer()){

		// set exit condition of main-loop automatic
		// if SIGViewer is disconnected from server, main loop will be broken up
			srv.setAutoExitLoop(true);
		}
	}
    srv.sendMsg("multiuserctr_0",Service_name);
	//Sleep(2);
	//srv.sendMsg("multiuserctr_0",Service_name);
	// start main loop
	srv.startLoop();  

	// finalize this process
	exit(0);
}

