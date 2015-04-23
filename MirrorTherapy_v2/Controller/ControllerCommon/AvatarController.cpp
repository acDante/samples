///*
// * AvatarController.cpp
// *
// *  Created on: 2015/03/12
// *      Author: Nozaki
// */
//
//#include "AvatarController.h"
//
//std::string AvatarController::getMessageHeader(const std::string &message, const std::string &recordDelim, const std::string &keyDelim)
//{
//	// Split string to records by ";".
//	std::vector<std::string> records;
//	this->splitString(message, recordDelim, records);
//
//	//
//	if ((int)records.size() < 2) {
//		// want to throw Exception
//	}
//
//	const std::string messageHeader = records[0] + recordDelim + records[1] + recordDelim;
//
//	return messageHeader;
//}
//
//bool AvatarController::checkMessageHeader(const std::string &messageHeader, const std::string &recordDelim, const std::string &keyDelim)
//{
//	// Split string to records by ";".
//	std::vector<std::string> records;
//	AvatarController::splitString(messageHeader, recordDelim, records);
//
//	std::vector<std::string> deviceTypeRecord;
//	AvatarController::splitString(records[0], keyDelim, deviceTypeRecord);
//
//	if ((int)deviceTypeRecord.size() < 2) {
//		// want to throw Exception
//		std::cout << "ILLEGAL DEVICE TYPE : " << messageHeader << std::endl;
//		return false;
//	}
//	if (deviceTypeRecord[0] != devTypeKey) {
//		std::cout << "ILLEGAL DEVICE TYPE : " << messageHeader << std::endl;
//		return false;
//	}
//
//	std::vector<std::string> deviceIDRecord;
//	AvatarController::splitString(records[1], keyDelim, deviceIDRecord);
//	if ((int)deviceIDRecord.size() < 2) {
//		// want to throw Exception
//		std::cout << "ILLEGAL DEVICE ID : " << messageHeader << std::endl;
//		return false;
//	}
//	if (deviceIDRecord[0] != devIDKey) {
//		// want to throw Exception
//		std::cout << "ILLEGAL DEVICE ID : " << messageHeader << std::endl;
//		return false;
//	}
//
//	return true;
//}
//
//std::string AvatarController::getDeviceType(const std::string &messageHeader, const std::string &recordDelim, const std::string &keyDelim)
//{
//	// Split string to records by ";".
//	std::vector<std::string> records;
//	AvatarController::splitString(messageHeader, recordDelim, records);
//
//	std::vector<std::string> deviceTypeRecord;
//	AvatarController::splitString(records[0], keyDelim, deviceTypeRecord);
//
//	return deviceTypeRecord[1];
//}
