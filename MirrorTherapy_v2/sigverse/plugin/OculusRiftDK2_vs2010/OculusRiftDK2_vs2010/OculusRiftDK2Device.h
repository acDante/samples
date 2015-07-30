#pragma once
#include <SIGService.h>
#include <sigverse\plugin\common\Device.h>
#include "Kernel/OVR_Math.h"
#include <d3d11.h>
#include "OVR_CAPI.h"
#include <conio.h>
#include <iostream>

#undef new

// Parameter file name. Must be defined.
#define PARAM_FILE_NAME_OCULUS_RIFT_DK2_INI  ("OculusRiftDK2.ini")

using namespace OVR;

class OculusRiftDK2Device :
	public Device
{
private:
	///@brief Parameter file name.
	std::string parameterFileName;

	//Structures for the application
	ovrHmd             HMD;
	ovrEyeRenderDesc   EyeRenderDesc[2];
	ovrRecti           EyeRenderViewport[2];

	///@brief Read parameter file.
	///@return Success flag to read parameter.
	void readIniFile();

public:
	OculusRiftDK2Device(){};
	OculusRiftDK2Device(int argc, char **argv);
	~OculusRiftDK2Device();

	void Process(Quatf* Q);

	void Release();

	///@brief Set service name.
	//void setSigServiceName();

	///@brief 
	void init();

	///@brief Run oculus dk2 device.
	void run();
};