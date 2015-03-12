#include "OculusDK1Device.h"
#include "OculusDK1SensorData.h"
#include <Windows.h>

OculusDK1Device::OculusDK1Device(int argc, char **argv)
{
	if (argc == 3) {
		std::cout << "SIGServer IP address: " << argv[1] << std::endl;
		std::cout << "Port number: " << argv[2] << std::endl;
		this->sendMessageFlag = true;
		this->serverAddress = argv[1];
		this->portNumber = atoi(argv[2]);
		this->setDeviceType(defaultDeviceType);
		this->setDeviceUniquID(defaultDeviceUniqueID);
	} else {
		std::cout << "Please execute with SIGServer address and port number." << std::endl;
		exit(1);
	}
	this->parameterFileName = defaultParameterFileName;
}

OculusDK1Device::~OculusDK1Device()
{
	// finalize Oculus Rift connection
	pSensor.Clear();
	pHMD.Clear();
	pManager.Clear();
	delete pFusionResult;
	OVR::System::Destroy();
}

void OculusDK1Device::setSigServiceName()
{
	// パラメータファイルを読み込んでサービスネームを設定します．
	// パラメータファイルが見つからなかった場合，ヘッダファイルに記述したサービスネームを設定します．
	if (!this->Device::readIniFile()) {
		this->serviceName = defaultServiceName;
		std::cout << "Set default service name : " << this->serviceName << std::endl;
	}
}

void OculusDK1Device::init()
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

void OculusDK1Device::run()
{
	init();

	this->setSigServiceName();
	sigverse::SIGService sigService(this->serviceName);
	this->initializeSigService(sigService);

	bool sendSuccessPrev = false;

	while (1) {

		//if (!sigService.checkRecvData(0)) {
		//	std::cout << "Disconnected to SIGServer." << std::endl;
		//	break;
		//}

		//if (!sigService.getIsConnectedView()) {
		//	std::cout << "Disconnected to SIGViewer." << std::endl;
		//	break;
		//}

		float r_yaw, r_pitch, r_roll;

		Quatf q = pFusionResult->GetOrientation();
		Matrix4f bodyFrameMatrix(q);
		q.GetEulerAngles<Axis_Y, Axis_X, Axis_Z>(&r_yaw, &r_pitch, &r_roll);

		OculusDK1SensorData oculusDK1SensorData;
		oculusDK1SensorData.setEulerAngle(r_yaw, r_pitch, r_roll);
		this->sensorData = &oculusDK1SensorData;

		if (this->sendMessageFlag) {
			const std::string sensorDataMessage = this->sensorData->convertSensorData2Message(keyDelimDefault, recordDelimDefault, valueDelimDefault);
			const std::string messageHeader = this->generateMessageHeader();
			const std::string message = messageHeader + sensorDataMessage;

			this->sendMessage(sigService, message);

			//if (sendSuccessPrev && !sendSuccess) {
			//	std::cout << "送れなくなったのでおわり" << std::endl;
			//	break;
			//}
			//if (!sendSuccess) {
			//	std::cout << "send failed." << std::endl;
			//	Sleep(3000);
			//	break;
			//}
			//std::cout << message << std::endl;
		}
		Sleep(100);
	}

	sigService.disconnect();

	std::cout << "end program" << std::endl;
	Sleep(2000);
}

