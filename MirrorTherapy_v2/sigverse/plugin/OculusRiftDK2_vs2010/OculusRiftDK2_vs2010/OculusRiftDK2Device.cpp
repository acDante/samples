#include <sigverse/plugin/OculusRiftDK2_vs2010/OculusRiftDK2_vs2010/OculusRiftDK2Device.h>
#include <sigverse/common/device/OculusRiftDK2SensorData.h>
#include <sigverse/plugin/common/CheckRecvSIGServiceData.h>
#include <Windows.h>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>

OculusRiftDK2Device::OculusRiftDK2Device(int argc, char **argv)
{
	if (argc == 3) {
		std::cout << "SIGServer IP address: " << argv[1] << std::endl;
		std::cout << "Port number: " << argv[2] << std::endl;
		this->sendMessageFlag = true;
		this->serverAddress = argv[1];
		this->portNumber = atoi(argv[2]);
		this->setDeviceType(DEV_TYPE_OCULUS_DK2);
		this->setDeviceUniqueID(DEV_UNIQUE_ID_0);
	}
	else {
		std::cout << "Please execute with SIGServer address and port number." << std::endl;
		exit(1);
	}
	this->parameterFileName = PARAM_FILE_NAME_OCULUS_RIFT_DK2_INI;
}

OculusRiftDK2Device::~OculusRiftDK2Device()
{
	// finalize Oculus Rift connection
	Release();
	OVR_ASSERT(!_CrtDumpMemoryLeaks());
}

void OculusRiftDK2Device::init()
{
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

void OculusRiftDK2Device::Process(Quatf* Q)
{
	static ovrPosef eyeRenderPose[2];

	static ovrTrackingState HmdState;

	ovrVector3f hmdToEyeViewOffset[2] = { EyeRenderDesc[0].HmdToEyeViewOffset, EyeRenderDesc[1].HmdToEyeViewOffset };
	ovrHmd_GetEyePoses(HMD, 0, hmdToEyeViewOffset, eyeRenderPose, &HmdState);

	*Q = HmdState.HeadPose.ThePose.Orientation;
}

void OculusRiftDK2Device::Release(void)
{
	ovrHmd_Destroy(HMD);
	HMD = 0;

	// No OVR functions involving memory are allowed after this.
	ovr_Shutdown();
}

void OculusRiftDK2Device::run()
{
	init();

	this->readIniFile();

	sigverse::SIGService sigService(this->serviceName);
	this->initializeSigService(sigService);

	// check receive SIGService data by another thread
	CheckRecvSIGServiceData checkRecvSIGServiceData;
	boost::thread thCheckRecvData(&CheckRecvSIGServiceData::run, &checkRecvSIGServiceData, &sigService);

	bool sendSuccessPrev = false;

	while (1) {

		if (_kbhit()){
			int key = _getch();
			if (key == 'r'){
				ovrHmd_RecenterPose(HMD);
				std::cout << "Reset center position" << std::endl;
			}
		}
		Quatf q;
		Process(&q);

		OculusRiftDK2SensorData sensorData;
		sensorData.setQuaternion(q.w, q.x, q.y, q.z);

		if (this->sendMessageFlag) {
			const std::string sensorDataMessage = sensorData.encodeSensorData();
			const std::string messageHeader = this->generateMessageHeader();
			const std::string message = messageHeader + sensorDataMessage;

			this->sendMessage(sigService, message);

			OculusRiftDK2SensorData tmp;
			std::map<std::string, std::vector<std::string> > sensorDataMap = tmp.decodeSensorData(message);
			tmp.setSensorData(sensorDataMap);
		}
		Sleep(100);
	}

	sigService.disconnect();

	std::cout << "end program" << std::endl;
	Sleep(2000);
}

///@brief Read parameter file.
///@return When couldn't read parameter file, return false;
void OculusRiftDK2Device::readIniFile()
{
	std::ifstream ifs(this->parameterFileName);

	// Parameter file is "not" exists.
	if (ifs.fail())
	{
		std::cout << "Not exist : " << this->parameterFileName << std::endl;
		std::cout << "Use default parameter." << std::endl;

		this->serviceName = SERVICE_NAME_OCULUS_DK2;
		this->deviceType = DEV_TYPE_OCULUS_DK2;
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

			this->serviceName = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME);
			this->deviceType = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE);
			this->deviceUniqueID = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID);
		}
		catch (boost::exception &ex)
		{
			std::cout << this->parameterFileName << " ERR :" << *boost::diagnostic_information_what(ex) << std::endl;
		}
	}

	std::cout << PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME << ":" << this->serviceName << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE << ":" << this->deviceType << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID << ":" << this->deviceUniqueID << std::endl;
}
