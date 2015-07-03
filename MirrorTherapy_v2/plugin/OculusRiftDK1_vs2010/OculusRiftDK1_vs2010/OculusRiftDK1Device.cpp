#include "OculusRiftDK1Device.h"
#include "common/device/OculusRiftDK1SensorData.h"
#include "../../PluginCommon/CheckRecvSIGServiceData.h"
#include <Windows.h>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>

OculusRiftDK1Device::OculusRiftDK1Device(int argc, char **argv)
{
	if (argc == 3) 
	{
		std::cout << "SIGServer IP address: " << argv[1] << std::endl;
		std::cout << "Port number: " << argv[2] << std::endl;

		this->sendMessageFlag = true;
		this->serverAddress = argv[1];
		this->portNumber = atoi(argv[2]);
		this->setDeviceType(DEV_TYPE_OCULUS_DK1);
		this->setDeviceUniqueID(DEV_UNIQUE_ID_0);
	} 
	else 
	{
		std::cout << "Please execute with SIGServer address and port number." << std::endl;
		exit(EXIT_FAILURE);
	}
	this->parameterFileName = PARAM_FILE_NAME_OCULUS_RIFT_DK1_INI;
}

OculusRiftDK1Device::~OculusRiftDK1Device()
{
	// finalize Oculus Rift connection
	pSensor.Clear();
	pHMD.Clear();
	pManager.Clear();
	delete pFusionResult;
	OVR::System::Destroy();
}

/**
 * @brief initialization
 */
void OculusRiftDK1Device::init()
{
	// initialization(prepare varibles for data acquisition of OculusRift's sensors)
	OVR::System::Init();

	pFusionResult = new OVR::SensorFusion();
	pManager = *OVR::DeviceManager::Create();

	pHMD = *pManager->EnumerateDevices<OVR::HMDDevice>().CreateDevice();

	if (pHMD)
	{
		InfoLoaded = pHMD->GetDeviceInfo(&Info);
		pSensor = *pHMD->GetSensor();
	}
	else
	{
		pSensor = *pManager->EnumerateDevices<OVR::SensorDevice>().CreateDevice();
	}

	if (pSensor)
	{
		pFusionResult->AttachToSensor(pSensor);
	}
}

/**
 * @brief main routine
 */
void OculusRiftDK1Device::run()
{
	init();

	this->readIniFile();

	//this->setSigServiceName();
	
	sigverse::SIGService sigService(this->serviceName);
	this->initializeSigService(sigService);

	// check receive SIGService data by another thread
	CheckRecvSIGServiceData checkRecvSIGServiceData;
	boost::thread thCheckRecvData(&CheckRecvSIGServiceData::run, &checkRecvSIGServiceData, &sigService);

	bool sendSuccessPrev = false;

	while (1) 
	{
		if (_kbhit()){
			int key = _getch();
			if (key == 'r'){
				pFusionResult->Reset();
				std::cout << "Reset Orientation." << std::endl;
			}
		}

		float r_yaw, r_pitch, r_roll;

		OVR::Quatf q = pFusionResult->GetOrientation();
		OVR::Matrix4f bodyFrameMatrix(q);
		q.GetEulerAngles<OVR::Axis_Y, OVR::Axis_X, OVR::Axis_Z>(&r_yaw, &r_pitch, &r_roll);

//		std::cout << "yaw, pitch, roll = " << r_yaw << "," << r_pitch << "," << r_roll << std::endl;

		OculusRiftDK1SensorData sensorData;
		sensorData.setEulerAngle(r_yaw, r_pitch, r_roll);

		if (this->sendMessageFlag) 
		{
			const std::string sensorDataMessage = sensorData.encodeSensorData();
			const std::string messageHeader = this->generateMessageHeader();
			const std::string message = messageHeader + sensorDataMessage;

			this->sendMessage(sigService, message);

			OculusRiftDK1SensorData tmp;
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
void OculusRiftDK1Device::readIniFile()
{
	std::ifstream ifs(this->parameterFileName);

	// Parameter file is "not" exists.
	if (ifs.fail()) 
	{
		std::cout << "Not exist : " << this->parameterFileName << std::endl;
		std::cout << "Use default parameter." << std::endl;

		this->serviceName    = SERVICE_NAME_OCULUS_DK1;
		this->deviceType     = DEV_TYPE_OCULUS_DK1;
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

