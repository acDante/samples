/*
 * Quaternion.cpp
 *
 *  Created on: 2015/07/01
 *      Author: tome-yamada
 */

#include "Quaternion.h"
#include <sstream>
#include <cmath>


Quaternion::Quaternion()
{
	this->w = 0.0;
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
};

Quaternion::~Quaternion()
{
};


Quaternion::Quaternion(const double w, const double x, const double y, const double z)
{
	this->w = w;
	this->x = x;
	this->y = y;
	this->z = z;
};

Quaternion::Quaternion(const Quaternion &v)
{
	this->w = v.w;
	this->x = v.x;
	this->y = v.y;
	this->z = v.z;
};


void Quaternion::getQuaternion(double &w, double &x, double &y, double &z)
{
	w = this->w;
	x = this->x;
	y = this->y;
	z = this->z;
}

void Quaternion::setQuaternion(const double w, const double x, const double y, const double z)
{
	this->w = w;
	this->x = x;
	this->y = y;
	this->z = z;
}

///@param[in] q Input quaternion.
///@param[in] r Input quaternion.
///@return Cross product of two input quaternions.
Quaternion Quaternion::calcCrossProduct(const Quaternion &q, const Quaternion &r)
{
	Quaternion tmp;
	tmp.w = q.w * r.w - q.x * r.x - q.y * r.y - q.z * r.z;
	tmp.x = q.w * r.x + q.x * r.w + q.y * r.z - q.z * r.y;
	tmp.y = q.w * r.y - q.x * r.z + q.y * r.w + q.z * r.x;
	tmp.z = q.w * r.z + q.x * r.y - q.y * r.x + q.z * r.w;
	return tmp;
};


/*
 * Calculate Quaternion from 2 Vectors
 */
Quaternion Quaternion::calcQuaternionFromVector(const SigCmn::Vector3 &kvec, const SigCmn::Vector3 &svec)
{
	Quaternion q;

	if (kvec.x == svec.x && kvec.y == svec.y && kvec.z == svec.z)
	{
		q.w = 1;
		q.x = 0;
		q.y = 0;
		q.z = 0;
		return q;
	}

	double x = kvec.y*svec.z - kvec.z*svec.y;
	double y = kvec.z*svec.x - kvec.x*svec.z;
	double z = kvec.x*svec.y - kvec.y*svec.x;

	//Normalization
	double sum = std::sqrt(x*x + y*y + z*z);
	x = x / sum;
	y = y / sum;
	z = z / sum;

	double angle = std::acos(kvec.x*svec.x + kvec.y*svec.y + kvec.z*svec.z);
	q.w = (float)(std::cos(angle / 2));
	q.x = (float)(x*std::sin(angle / 2));
	q.y = (float)(y*std::sin(angle / 2));
	q.z = (float)(z*std::sin(angle / 2));

	return q;
}

/*
 * Rotate Vector by Quaternion
 */
void Quaternion::rotVec(SigCmn::Vector3 &v, const Quaternion &q)
{
	double rx = v.x *  q.w + v.y * -q.z + v.z *  q.y;
	double ry = v.x *  q.z + v.y *  q.w + v.z * -q.x;
	double rz = v.x * -q.y + v.y *  q.x + v.z *  q.w;
	double rw = v.x *  q.x + v.y *  q.y + v.z *  q.z;

	v.x = (float)(q.x *  rw + q.y *  rz + q.z * -ry + q.w * rx);
	v.y = (float)(q.x * -rz + q.y *  rw + q.z *  rx + q.w * ry);
	v.z = (float)(q.x *  ry + q.y * -rx + q.z *  rw + q.w * rz);
}

