#include "OculusRiftDK1Device.h"
#include <Common\OculusRiftDK1SensorData.h>
#include <Windows.h>

OculusRiftDK1Device::OculusRiftDK1Device(int argc, char **argv)
{
	if (argc == 3) {
		std::cout << "SIGServer IP address: " << argv[1] << std::endl;
		std::cout << "Port number: " << argv[2] << std::endl;
		this->sendMessageFlag = true;
		this->serverAddress = argv[1];
		this->portNumber = atoi(argv[2]);
		this->setDeviceType(defaultDeviceType);
		this->setDeviceUniqueID(defaultDeviceUniqueID);
	} else {
		std::cout << "Please execute with SIGServer address and port number." << std::endl;
		exit(1);
	}
	this->parameterFileName = defaultParameterFileName;
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

void OculusRiftDK1Device::setSigServiceName()
{
	// パラメータファイルを読み込んでサービスネームを設定します．
	// パラメータファイルが見つからなかった場合，ヘッダファイルに記述したサービスネームを設定します．
	if (!this->Device::readIniFile()) {
		this->serviceName = defaultServiceName;
		std::cout << "Set default service name : " << this->serviceName << std::endl;
	}
}

void OculusRiftDK1Device::init()
{
	// initialization(prepare varibles for data acquisition of OculusRift's sensors)
	OVR::System::Init();

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

void OculusRiftDK1Device::run()
{
	init();

	this->setSigServiceName();
	sigverse::SIGService sigService(this->serviceName);
	this->initializeSigService(sigService);

	bool sendSuccessPrev = false;

	while (1) {

		float r_yaw, r_pitch, r_roll;

		Quatf q = pFusionResult->GetOrientation();
		Matrix4f bodyFrameMatrix(q);
		q.GetEulerAngles<Axis_Y, Axis_X, Axis_Z>(&r_yaw, &r_pitch, &r_roll);

		OculusRiftDK1SensorData oculusRiftDK1SensorData;
		oculusRiftDK1SensorData.setEulerAngle(r_yaw, r_pitch, r_roll);
		this->sensorData = &oculusRiftDK1SensorData;

		if (this->sendMessageFlag) {
			const std::string sensorDataMessage = this->sensorData->encodeSensorData2Message(keyDelimDefault, recordDelimDefault, valueDelimDefault);
			const std::string messageHeader = this->generateMessageHeader();
			const std::string message = messageHeader + sensorDataMessage;

			this->sendMessage(sigService, message);
		}
		Sleep(100);
	}

	sigService.disconnect();

	std::cout << "end program" << std::endl;
	Sleep(2000);
}

