#include <iostream>
#include <fstream>
#include "Device.h"
#include "SIGService.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>

/////@brief Constructor
//Device::Device(int argc, char **argv)
//{
//	// It is running with SIGViewer.
//	if (argc == 1) {
//		// When it works without SIGViewer.
//		fprintf(stdout, "Work stand alone.\n");
//		this->sendMessageFlag = false;
//		this->serverAddress = "";
//		this->portNumber = -1;
//	}
//	else if (argc == 3) {
//		// Work with SIGViewer.
//		std::cout << "hghghgh " << std::endl;
//		fprintf(stdout, "SIGServer IP address: %s\n", argv[1]);
//		fprintf(stdout, "Port number: %s\n", argv[2]);
//		this->sendMessageFlag = true;
//		this->serverAddress = argv[1];
//		this->portNumber = atoi(argv[2]);
//		//this->readIniFile();
//	}
//
//	//this->kinectConnector = KinectV2Connector();
//	this->readIniFile();
//
//}
///@brief Initialize SIGService.
void Device::initializeSigService(sigverse::SIGService &sigService)
{
	if (this->sendMessageFlag) {
		// Initialize sigservice.
		try {
			if (sigService.connect(this->serverAddress, this->portNumber)) {
				std::cout << "Success to connect to SIGServer" <<
					"(IP: " << this->serverAddress <<
					", port: " << this->portNumber << " )." << std::endl;
			}
			if (sigService.connectToViewer()) {
				std::cout << "Success to connect to SIGViewer." << std::endl;
				sigService.setAutoExitLoop(true);
			}
		}
		catch (std::exception &ex) {
			std::cout << "Initialize sigservice ERR :" << ex.what() << std::endl;
		}
	}
}

///@brief Read parameter file.
///@return When couldn't read parameter file, return false;
// パラメータファイルが見つかった時の処理．
// パラメータファイル名は継承先で記述するべきなので，多少は回りくどい記述になっているかもしれない．
bool Device::readIniFile()
{
	std::ifstream ifs(this->parameterFileName);

	if (ifs.fail()) {
		// When there isn't parameter file, return false.
		// パラメータファイルが見つからなかった時は，パラメータのデフォルト値を
		// どうするべきかは継承先でしかわからないので，継承先の処理に任せる．
		std::cout << "Not exist " << this->parameterFileName << std::endl;
		return false;
	}
	else {
		try {
			// パラメータファイルが見つかった時は，書いてある内容からサービスネームを取得してセットする．
			std::cout << "Read " << this->parameterFileName << std::endl;
			boost::property_tree::ptree pt;
			boost::property_tree::read_ini(this->parameterFileName, pt);
			this->serviceName = pt.get<std::string>("General.service_name");
			std::cout << "service_name:" << this->serviceName << std::endl;
			return true;
		}
		catch (boost::exception &ex) {
			std::cout << this->parameterFileName << " ERR :" << *boost::diagnostic_information_what(ex) << std::endl;
		}
	}
	return true;
}

const void Device::sendMessage(sigverse::SIGService &sigService, const std::string &message)
{
	if (!sigService.checkRecvData(0)) {
		std::cout << "SIGService::chkRecvData ERR." << std::endl;
	}
	else {
		std::vector<std::string> connectedEntitiesName = sigService.getAllConnectedEntitiesName();
		for (int i = 0; i < (int)connectedEntitiesName.size(); i++) {
			sigService.sendMsgToCtr(connectedEntitiesName[i].c_str(), message);
		}
	}
}