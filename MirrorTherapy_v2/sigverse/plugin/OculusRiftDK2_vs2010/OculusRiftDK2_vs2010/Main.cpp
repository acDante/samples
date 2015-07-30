#include <iostream>
#include <sigverse/plugin/OculusRiftDK2_vs2010/OculusRiftDK2_vs2010/OculusRiftDK2Device.h>

int main(int argc, char* argv[])
{
	try {
		OculusRiftDK2Device oculusRiftDK2Device(argc, argv);

		oculusRiftDK2Device.run();
	}
	catch (...) {
		std::cout << "catch (...)" << std::endl;
	}

	return 0;
}