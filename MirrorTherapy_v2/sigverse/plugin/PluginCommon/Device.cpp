#include <iostream>
#include <fstream>
#include "Device.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>


/**
 * @brief Initialize SIGService.
 */
void Device::initializeSigService(sigverse::SIGService &sigService)
{
	if (this->sendMessageFlag) 
	{
		// Initialize sigservice.
		try 
		{
			if (sigService.connect(this->serverAddress, this->portNumber)) 
			{
				std::cout << "Success to connect to SIGServer(IP: " << this->serverAddress << ", port: " << this->portNumber << " )." << std::endl;
				
				//if (sigService.connectToViewer()) 
				//{
				//	std::cout << "Success to connect to SIGViewer." << std::endl;
				//	//sigService.setAutoExitLoop(true);
				//}
			}
		}
		catch (std::exception &ex) 
		{
			std::cout << "Initialize sigservice ERR :" << ex.what() << std::endl;
		}
	}
}


/**
 * @brief send message to Controller
 */
const void Device::sendMessage(sigverse::SIGService &sigService, const std::string &message)
{
	try 
	{
		std::vector<std::string> connectedEntitiesName = sigService.getAllConnectedEntitiesName();
			
		for (int i = 0; i < (int)connectedEntitiesName.size(); i++) 
		{
			sigService.sendMsgToCtr(connectedEntitiesName[i].c_str(), message);
		}
	}
	catch (...) 
	{
		std::cout << "catch (...) in Device::sendMessage" << std::endl;
		Sleep(20000);
	}
}
