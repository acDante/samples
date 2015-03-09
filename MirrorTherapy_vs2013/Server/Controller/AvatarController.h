/*
 */

#ifndef AVATARCONTROLLER_H_
#define AVATARCONTROLLER_H_

#include <sigverse/Controller.h>
#include "AvatarPosture.h"

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
	AvatarPosture *avatarPosture;

	virtual void convertMessage2AvatarPosture(const std::string &message) = 0;

	static void splitString(const std::string &inputString, const std::string &delimiter, std::vector<std::string> &output);

	std::string getMessageHeader(const std::string &message, const std::string &recordDelim = recordDelimDefault, const std::string &keyDelim = keyDelimDefault);

	static bool checkMessageHeader(const std::string &messageHeader, const std::string &recordDelim = recordDelimDefault, const std::string &keyDelim = keyDelimDefault);

	static std::string getDeviceType(const std::string &messageHeader, const std::string &recordDelim = recordDelimDefault, const std::string &keyDelim = keyDelimDefault);
};

#endif /* AVATARCONTROLLER_H_ */
