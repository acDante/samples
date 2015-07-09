#include <iostream>
#include <sigverse/plugin/KinectV2_vs2013/KinectV2_vs2013/KinectV2Device.h>

int main(int argc, char **argv)
{
	try
	{
		KinectV2Device kinectV2Device(argc, argv);

		kinectV2Device.run();

		return 0;
	}
	catch (std::exception ex)
	{
		std::cout << ex.what() << std::endl;
	}
}