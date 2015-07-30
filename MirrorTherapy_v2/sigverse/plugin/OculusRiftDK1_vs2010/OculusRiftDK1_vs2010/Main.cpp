#include <iostream>
#include <sigverse/plugin/OculusRiftDK1_vs2010/OculusRiftDK1_vs2010/OculusRiftDK1Device.h>

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