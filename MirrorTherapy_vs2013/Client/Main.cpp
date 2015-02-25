#include "KinectDevice.h"
#include <boost\thread.hpp>

int main(int argc, char **argv)
{
	KinectDevice device(argc, argv);

	boost::thread thDraw(&KinectDevice::runDrawingMethods, &device);

	boost::thread thSend(&KinectDevice::runSendingMethods, &device);

	thDraw.join();
	thSend.join();

	return 0;
}