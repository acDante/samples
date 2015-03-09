/*
 * AvatarController.cpp
 *
 *  Created on: 2015/03/09
 *      Author: sigverse
 */

#include "AvatarController.h"

void AvatarController::splitString(const std::string &inputString, const std::string &delimiter, std::vector<std::string> &output)
{
	std::string::size_type index = inputString.find_first_of(delimiter);

	if (index != std::string::npos)
	{
		output.push_back(inputString.substr(0, index));
		splitString(inputString.substr(index + 1), delimiter, output);
	} else
	{
		output.push_back(inputString);
	}
}

std::string AvatarController::getMessageHeader(const std::string &message, const std::string &recordDelim, const std::string &keyDelim)
{
	// Split string to records by ";".
	std::vector<std::string> records;
	this->splitString(message, recordDelim, records);

	//
	if ((int)records.size() < 2) {
		// メッセージからヘッダを読み込めなかったときの処理
		// 例外を投げたい
	}

	const std::string messageHeader = records[0] + recordDelim + records[1] + recordDelim;

	return messageHeader;
}

bool AvatarController::checkMessageHeader(const std::string &messageHeader, const std::string &recordDelim, const std::string &keyDelim)
{
	// エラーチェックが甘い

	// Split string to records by ";".
	std::vector<std::string> records;
	AvatarController::splitString(messageHeader, recordDelim, records);

	std::vector<std::string> deviceTypeRecord;
	AvatarController::splitString(records[0], keyDelim, deviceTypeRecord);

	if ((int)deviceTypeRecord.size() < 2) {
		// 例外を投げたい
		std::cout << "ILLEGAL DEVICE TYPE : " << messageHeader << std::endl;
		return false;
	}
	if (deviceTypeRecord[0] != devTypeKey) {
		std::cout << "ILLEGAL DEVICE TYPE : " << messageHeader << std::endl;
		return false;
	}

	std::vector<std::string> deviceIDRecord;
	AvatarController::splitString(records[1], keyDelim, deviceIDRecord);
	if ((int)deviceIDRecord.size() < 2) {
		// 例外を投げたい
		std::cout << "ILLEGAL DEVICE ID : " << messageHeader << std::endl;
		return false;
	}
	if (deviceIDRecord[0] != devIDKey) {
		// 例外を投げたい
		std::cout << "ILLEGAL DEVICE ID : " << messageHeader << std::endl;
		return false;
	}

	return true;
}

std::string AvatarController::getDeviceType(const std::string &messageHeader, const std::string &recordDelim, const std::string &keyDelim)
{
	// Split string to records by ";".
	std::vector<std::string> records;
	AvatarController::splitString(messageHeader, recordDelim, records);

	std::vector<std::string> deviceTypeRecord;
	AvatarController::splitString(records[0], keyDelim, deviceTypeRecord);

	return deviceTypeRecord[1];
}
