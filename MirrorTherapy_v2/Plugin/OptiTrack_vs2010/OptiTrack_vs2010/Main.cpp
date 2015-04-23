/*==================================================================
	Sample code that uses OptiTrack (VC++2010).
	Based by NatNet_SDK_2.7\NatNetSDK\Samples\SampleClient\SampleClient.cpp
  ==================================================================*/

#include "OptiTrackDevice.h"

int main(int argc, char **argv) 
{
	try
	{
		OptiTrackDevice optiTrackDevice(argc, argv);
		optiTrackDevice.run();
	}
	catch (...) 
	{
		std::cout << " catch (...) " << std::endl;
	}

	return 0;
}

