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

protected:
	Posture *avatarPosture;
};

#endif /* AVATARCONTROLLER_H_ */
