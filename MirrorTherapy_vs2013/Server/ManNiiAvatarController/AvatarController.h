/*
 * AvatarController.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#ifndef AVATARCONTROLLER_H_
#define AVATARCONTROLLER_H_

#include <sigverse/Controller.h>
#include "Posture.h"

// Define delimiters to split string.
#define KEY_DELIMITER ":"
#define RECORD_DELIMITER ";"
#define VALUE_DELIMITER ","
static const std::string keyDelimDefault = KEY_DELIMITER;
static const std::string recordDelimDefault = RECORD_DELIMITER;
static const std::string valueDelimDefault = VALUE_DELIMITER;

// Define message header keys.
#define DEV_TYPE_KEY "DEV_TYPE"
#define DEV_ID_KEY "DEV_ID"
static const std::string devTypeKey = DEV_TYPE_KEY;
static const std::string devIDKey = DEV_ID_KEY;


class AvatarController : public Controller
{
private:
	//AvatarPosture *avatarPosture;

public:
	Posture *avatarPosture;

	virtual void convertMessage2Posture(const std::string &message) = 0;

	static void splitString(const std::string &inputString, const std::string &delimiter, std::vector<std::string> &output);

	static std::vector<double> splitValues(const std::string &valueString, const std::string &valueDelim = valueDelimDefault);

	std::string getMessageHeader(const std::string &message, const std::string &recordDelim = recordDelimDefault, const std::string &keyDelim = keyDelimDefault);

	static bool checkMessageHeader(const std::string &messageHeader, const std::string &recordDelim = recordDelimDefault, const std::string &keyDelim = keyDelimDefault);

	static std::string getDeviceType(const std::string &messageHeader, const std::string &recordDelim = recordDelimDefault, const std::string &keyDelim = keyDelimDefault);
};


class Record
{
public:
	std::string key;
	std::string value;

	Record(const std::string &recordString) {
		try {
			std::vector<std::string> tmp;
			AvatarController::splitString(recordString, keyDelimDefault, tmp);
			this->key = tmp[0];
			this->value = tmp[1];
		}
		catch (std::exception &ex) {
			std::cout << "Can't split message: " << recordString << std::endl;
		}
	}

	const std::string str() {
		return ("KEY:" + this->key + ", VALUE:" + this->value);
	}
};


#endif /* AVATARCONTROLLER_H_ */
