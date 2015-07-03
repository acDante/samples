/*
 * ManNiiAvatarController.h
 *
 *  Created on: 2015/07/01
 *      Author: tome-yamada
 */

#ifndef __MAN_NII_AVATAR_CONTROLLER_H__
#define __MAN_NII_AVATAR_CONTROLLER_H__

#include "AvatarController.h"
#include <iostream>
#include <sstream>
#include <string>
#include "Posture.h"
#include "../../Common/SigCmn.h"

class ManNiiAvatarController : public AvatarController
{
protected:
	//Avatar joints
	enum SigVec
	{
		HIP = 0,
		HTOTOR,
		WAIST,
		RSHOULDER,
		LSHOULDER,
		RELBOW,
		LELBOW,
		LEG,
		FOOT
	};

	SigCmn::Vector3 getSigVec(int sigvec);
};

#endif /* __MAN_NII_AVATAR_CONTROLLER_H__ */

