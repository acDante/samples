/*
 * Posture.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#ifndef __POSTURE_H__
#define __POSTURE_H__

#include <string>
#include <sstream>
#include "../../Common/SensorData.h"
#include <math.h>


/*!
 * @brief Translate degree to radian.
 */
#define DEG2RAD(DEG) ( M_PI * (DEG) / 180.0 )


class Quaternion
{
public:

	double w;
	double x;
	double y;
	double z;

	Quaternion()
	{
		w = 0;
		x = 0;
		y = 0;
		z = 0;
	};
	~Quaternion(){};

	Quaternion(const double w, const double x, const double y, const double z)
	{
		this->w = w;
		this->x = x;
		this->y = y;
		this->z = z;
	};

	Quaternion(const SensorData::Vector4 &v)
	{
		this->w = v.w;
		this->x = v.x;
		this->y = v.y;
		this->z = v.z;
	};


	///@param[in] q Input quaternion.
	///@param[in] r Input quaternion.
	///@return Cross product of two input quaternions.
	static Quaternion calcCrossProduct(const Quaternion &q, const Quaternion &r)
	{
		Quaternion tmp;
		tmp.w = q.w * r.w - q.x * r.x - q.y * r.y - q.z * r.z;
		tmp.x = q.w * r.x + q.x * r.w + q.y * r.z - q.z * r.y;
		tmp.y = q.w * r.y - q.x * r.z + q.y * r.w + q.z * r.x;
		tmp.z = q.w * r.z + q.x * r.y - q.y * r.x + q.z * r.w;
		return tmp;
	};

//	std::string str()
//	{
//		std::stringstream ss;
//		ss << "(" << w << "," << x << "," << y << "," << z << ")";
//		return ss.str();
//	}

	void getQuaternion(double &w, double &x, double &y, double &z)
	{
		w = this->w;
		x = this->x;
		y = this->y;
		z = this->z;
	}

	void setQuaternion(const double w, const double x, const double y, const double z)
	{
		this->w = w;
		this->x = x;
		this->y = y;
		this->z = z;
	}
};

class Posture
{
};

#endif /* __POSTURE_H__ */
