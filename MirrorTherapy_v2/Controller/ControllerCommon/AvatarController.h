/*
 * AvatarController.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#ifndef __AVATAR_CONTROLLER_H__
#define __AVATAR_CONTROLLER_H__

#include <sigverse/Controller.h>
#include "Posture.h"

class AvatarController : public Controller
{
private:
	//AvatarPosture *avatarPosture;

protected:
	Posture *avatarPosture;
};

#endif /* __AVATAR_CONTROLLER_H__ */
