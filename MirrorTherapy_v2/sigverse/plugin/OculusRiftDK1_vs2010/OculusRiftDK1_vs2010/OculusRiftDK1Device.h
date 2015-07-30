#pragma once
#include <SIGService.h>
#include <sigverse/plugin/common/Device.h>

#include <OVR.h>
#include <conio.h>
#include <iostream>

#undef new

// Parameter file name. Must be defined.
#define PARAM_FILE_NAME_OCULUS_RIFT_DK1_INI  ("OculusRiftDK1.ini")

class OculusRiftDK1Device : public Device
{
private:
	///@brief Parameter file name.
	std::string parameterFileName;

	OVR::Ptr<OVR::DeviceManager>  pManager;
	OVR::Ptr<OVR::HMDDevice>      pHMD;
	OVR::Ptr<OVR::SensorDevice>   pSensor;
	OVR::SensorFusion*            pFusionResult;
	OVR::HMDInfo                  Info;
	bool                          InfoLoaded;

	///@brief Read parameter file.
	///@return Success flag to read parameter.
	void readIniFile();

public:
	OculusRiftDK1Device(){};
	OculusRiftDK1Device(int argc, char **argv);
	~OculusRiftDK1Device();
	
	/////@brief Set service name.
	//void setSigServiceName();

	///@brief 
	void init();

	///@brief Run oculus dk1 device.
	void run();
};

