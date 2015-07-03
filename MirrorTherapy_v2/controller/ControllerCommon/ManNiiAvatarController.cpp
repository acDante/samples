/*
 * ManNiiAvatarController.cpp
 *
 *  Created on: 2015/07/01
 *      Author: tome-yamada
 */
#include "ManNiiAvatarController.h"

#include <iostream>
#include <sstream>
#include <cmath>

/*
 * Get initial vector of avatar joints.
 */
SigCmn::Vector3 ManNiiAvatarController::getSigVec(const int sigvec)
{
	SigCmn::Vector3 p;

	if (sigvec == WAIST || sigvec == HTOTOR)
	{
		p.x = 0;
		p.y = 1;
		p.z = 0;
	}
	else if (sigvec == RSHOULDER || sigvec == RELBOW)
	{
		p.x = -1;
		p.y = 0;
		p.z = 0;
	}
	else if (sigvec == HIP || sigvec == LSHOULDER || sigvec == LELBOW)
	{
		p.x = 1;
		p.y = 0;
		p.z = 0;
	}
	else if (sigvec == LEG)
	{
		p.x = 0;
		p.y = -1;
		p.z = 0;
	}
	else if (sigvec == FOOT)
	{
		p.x = 0;
		p.y = -0.5;
		p.z = +std::sqrt(3.0)/2.0;
	}
	return p;
}

