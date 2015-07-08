/*
 * SigCmn.h
 *
 *  Created on: 2015/07/01
 *      Author: tome-yamada
 */

#include <sigverse/common/SigCmn.h>
#include <cmath>

/*
 * Calculate Vector from 2 position.
 */
bool SigCmn::diffVec(SigCmn::Vector3 &rvec, const SigCmn::Vector3 &jvec, const SigCmn::Vector3 &kvec)
{
	rvec.x = -(kvec.x - jvec.x);
	rvec.y = +(kvec.y - jvec.y);
	rvec.z = -(kvec.z - jvec.z);

	//Normalization
	double length = std::sqrt(rvec.x*rvec.x + rvec.y*rvec.y + rvec.z*rvec.z);
	rvec.x = (float)(rvec.x / length);
	rvec.y = (float)(rvec.y / length);
	rvec.z = (float)(rvec.z / length);

	return true;
}
