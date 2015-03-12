#pragma once
#include "device.h"

#include "SIGService.h"

#include "OVR.h"
#include <iostream>
#include <conio.h>

#undef new

// Parameter file name. Must be defined.
// �p�����[�^�t�@�C���̃t�@�C�������L�q����D
#define PARAM_FILE_NAME "OculusDK1.conf"
static const std::string defaultParameterFileName = PARAM_FILE_NAME;

// Default service name of this program. Must be defined.
// SIGServer�Ƃ̒ʐM�ŕK�v�ɂȂ�T�[�r�X���̏����l���L�q����D
// ��L��PARAM_FILE_NAME��������Ȃ��ꍇ�C�����Œ�`����鏉���l���g����D
#define DEFAULT_SERVICE_NAME "SIGOCULUS"
static const std::string defaultServiceName = DEFAULT_SERVICE_NAME;

// Default device type name for oculus dk1.
#define DEFAULT_DEVICE_TYPE "OCULUSDK1"
static const std::string defaultDeviceType = DEFAULT_DEVICE_TYPE;

// Default device unique ID for oculus dk1.
#define DEFAULT_DEVICE_ID "0"
static const std::string defaultDeviceUniqueID = DEFAULT_DEVICE_ID;

using namespace OVR;

class OculusDK1Device :
	public Device
{
private:
	Ptr<DeviceManager>  pManager;
	Ptr<HMDDevice>      pHMD;
	Ptr<SensorDevice>   pSensor;
	SensorFusion*       pFusionResult;
	HMDInfo             Info;
	bool                InfoLoaded;

public:
	OculusDK1Device(){};
	OculusDK1Device(int argc, char **argv);
	~OculusDK1Device();
	
	///@brief Set service name.
	void setSigServiceName();

	///@brief 
	void init();

	///@brief Run oculus dk1 device.
	void run();
};

