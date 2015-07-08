/*
 * Quaternion.h
 *
 *  Created on: 2015/07/01
 *      Author: tome-yamada
 */

#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include <sigverse/common/SigCmn.h>
#include <string>


class Quaternion
{
public:

	double w;
	double x;
	double y;
	double z;

	Quaternion();
	~Quaternion();

	Quaternion(const double w, const double x, const double y, const double z);

	Quaternion(const Quaternion &v);

	void getQuaternion(double &w, double &x, double &y, double &z) const;
	void setQuaternion(const double w, const double x, const double y, const double z);

	///@param[in] q Input quaternion.
	///@param[in] r Input quaternion.
	///@return Cross product of two input quaternions.
	static const Quaternion calcCrossProduct(const Quaternion &q, const Quaternion &r);

	static const Quaternion calcQuaternionFromVector(const SigCmn::Vector3 &kvec, const SigCmn::Vector3 &svec);
	static void rotVec(SigCmn::Vector3 &v, const Quaternion &q);
};

#endif /* __QUATERNION_H__ */
