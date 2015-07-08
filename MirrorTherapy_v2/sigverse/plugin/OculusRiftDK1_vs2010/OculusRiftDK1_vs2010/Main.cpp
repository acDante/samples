#include <iostream>
#include "OculusRiftDK1Device.h"

int main(int argc, char* argv[])
{
	try 
	{
		OculusRiftDK1Device oculusRiftDK1Device(argc, argv);

		oculusRiftDK1Device.run();
	}
	catch (...) 
	{
		std::cout << "catch (...)" << std::endl;
	}

	return 0;
}