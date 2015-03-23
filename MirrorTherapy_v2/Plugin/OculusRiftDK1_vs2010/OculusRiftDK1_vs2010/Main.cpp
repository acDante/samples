#include <iostream>
#include "OculusRiftDK1Device.h"

int main(int argc, char* argv[])
{
	try {
		OculusRiftDK1Device oculusRiftDK1Device(argc, argv);

		oculusRiftDK1Device.run();
	}
	//catch (std::exception &ex) {
	//	std::cout << "std::exception" << std::endl;
	//}
	//catch (std::string &ex) {
	//	std::cout << "std::string" << std::endl;
	//}
	catch (...) {
		std::cout << "catch (...)" << std::endl;
	}

	return 0;
}