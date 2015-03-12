#include <iostream>
#include "OculusDK1Device.h"

int main(int argc, char **argv)
{
	try {
		OculusDK1Device oculusDK1Device(argc, argv);

		oculusDK1Device.run();
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