#include <iostream>
#include "KinectV2Device.h"

int main(int argc, char **argv) 
{
	KinectV2Device kinectV2Device(argc, argv);

	kinectV2Device.run();

	return 0;
}