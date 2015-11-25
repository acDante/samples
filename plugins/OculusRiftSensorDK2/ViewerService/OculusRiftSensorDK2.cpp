#include"SIGService.h"

#include "Kernel/OVR_Math.h"
#include <d3d11.h>
#include "OVR_CAPI.h"
#include <iostream>
#include <conio.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define RAD2DEG(RAD) (RAD * 180.) / M_PI

using namespace OVR;

class OculusRiftDK2Service : public sigverse::SIGService{
public:
	OculusRiftDK2Service(std::string name) : SIGService(name){};
	~OculusRiftDK2Service();

	// initialization for whole procedure on this service
	void onInit();

	void Process(Quatf* Q);

	void Release();

	// function to be called if this service receives messages
	//void onRecvMsg(sigverse::RecvMsgEvent &evt);

	// function to be called periodically
	double onAction();

	// an utility function for conversion of data type from float to string.
	std::string DoubleToString(float x);

	//Structures for the application
	ovrHmd             HMD;
	ovrEyeRenderDesc   EyeRenderDesc[2];
	ovrRecti           EyeRenderViewport[2];
};

void OculusRiftDK2Service::onInit(){
	HMD = 0;
	// Initializes LibOVR, and the Rift
	ovr_Initialize();
	if (!HMD)
	{
		HMD = ovrHmd_Create(0);
		if (!HMD)
		{
			MessageBoxA(NULL, "Oculus Rift not detected.", "", MB_OK);
			exit(-1);
		}
		if (HMD->ProductName[0] == '\0')
			MessageBoxA(NULL, "Rift detected, display not enabled.", "", MB_OK);
	}

	ovrHmd_SetEnabledCaps(HMD, ovrHmdCap_LowPersistence | ovrHmdCap_DynamicPrediction);

	// Start the sensor which informs of the Rift's pose and motion
	ovrHmd_ConfigureTracking(HMD, ovrTrackingCap_Orientation |
		ovrTrackingCap_MagYawCorrection |
		ovrTrackingCap_Position, 0);

	ovrHmd_RecenterPose(HMD);

	std::cout << "-------------- Function --------------\n" << std::endl;
	std::cout << "Press \"r\" key : Reset center position." << std::endl;
	std::cout << "\n--------------------------------------\n" << std::endl;
}

void OculusRiftDK2Service::Process(Quatf* Q)
{
	static ovrPosef eyeRenderPose[2];

	static ovrTrackingState HmdState;

	ovrVector3f hmdToEyeViewOffset[2] = { EyeRenderDesc[0].HmdToEyeViewOffset, EyeRenderDesc[1].HmdToEyeViewOffset };
	ovrHmd_GetEyePoses(HMD, 0, hmdToEyeViewOffset, eyeRenderPose, &HmdState);

	*Q = HmdState.HeadPose.ThePose.Orientation;
}

void OculusRiftDK2Service::Release(void)
{
	ovrHmd_Destroy(HMD);
	HMD = 0;

	// No OVR functions involving memory are allowed after this.
	ovr_Shutdown();
}

OculusRiftDK2Service::~OculusRiftDK2Service(){
	// shutdown SIGService
	this->disconnect();

	// finalize Oculus Rift connection
	Release();
	OVR_ASSERT(!_CrtDumpMemoryLeaks());
}

//periodic procedure for sending messages to the controller
double OculusRiftDK2Service::onAction(){
	if (_kbhit()){
		int key = _getch();
		if (key == 'r'){
			ovrHmd_RecenterPose(HMD);
			std::cout << "Reset center position" << std::endl;
		}
	}
	Quatf q;
	Process(&q);

	std::vector<std::string> names;
	names = this->getAllConnectedEntitiesName();
	int entSize = names.size();
	std::string msg;
	for (int i = 0; i < entSize; i++) {
		msg = "ORS_DATA ";
		msg += DoubleToString(q.w);
		msg += DoubleToString(-q.x);
		msg += DoubleToString(q.y);
		msg += DoubleToString(-q.z);
		msg += " END:";
		this->sendMsgToCtr(names[i], msg);
	}

	return 0.01;  //time period
}

// utility function to convert data type from 'double' to 'string'
std::string OculusRiftDK2Service::DoubleToString(float x){
	char tmp[32];
	sprintf_s(tmp, 32, "%.4f", x);
	std::string str = std::string(tmp);
	str += ",";
	return str;
}

//entry point
void main(int argc, char* argv[]){
	// create instance
	OculusRiftDK2Service srv("SIGORSDK2");

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
	if (srv.connect(saddr, portnumber)){
		std::cout << "connected." << std::endl << std::endl;
		// connect to SIGViewer
		if (srv.connectToViewer()){
			// set exit condition of main-loop automatic
			// if SIGViewer is disconnected from server, main loop will be broken up
			srv.setAutoExitLoop(true);
		}
	}

	// start main loop
	srv.startLoop();

	// finalize this process
	exit(0);
}
