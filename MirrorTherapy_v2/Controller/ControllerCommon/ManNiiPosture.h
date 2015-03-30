/*
 * ManNiiPosture.h
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#ifndef MANNIIPOSTURE_H_
#define MANNIIPOSTURE_H_

#include "Posture.h"
#include <string>
#include <sstream>

#ifndef _ManNiiJointType_
#define _ManNiiJointType_

enum ManNiiJointType
{
	HEAD_JOINT0 = 0,
	HEAD_JOINT1 = 1,
	LARM_JOINT0 = 2,
	LARM_JOINT1 = 3,
	LARM_JOINT2 = 4,
	LARM_JOINT3 = 5,
	LARM_JOINT4 = 6,
	LARM_JOINT5 = 7,
	LARM_JOINT6 = 8,
	LARM_JOINT7 = 9,
	RARM_JOINT0 = 10,
	RARM_JOINT1 = 11,
	RARM_JOINT2 = 12,
	RARM_JOINT3 = 13,
	RARM_JOINT4 = 14,
	RARM_JOINT5 = 15,
	RARM_JOINT6 = 16,
	RARM_JOINT7 = 17,
	WAIST_JOINT0 = 18,
	WAIST_JOINT1 = 19,
	WAIST_JOINT2 = 20,
	ROOT_JOINT0 = 21,
	ROOT_JOINT1 = 22,
	ROOT_JOINT2 = 23,
	LLEG_JOINT2 = 24,
	LLEG_JOINT4 = 25,
	LLEG_JOINT6 = 26,
	RLEG_JOINT2 = 27,
	RLEG_JOINT4 = 28,
	RLEG_JOINT6 = 29,
	LEYE_JOINT1 = 30,
	LEYE_JOINT0 = 31,
	REYE_JOINT1 = 32,
	REYE_JOINT0 = 33,
	ManNiiJointType_Count = (REYE_JOINT0 + 1)
};

struct manNiiJointTypeStr : public std::string {
	manNiiJointTypeStr(ManNiiJointType e) {
		switch(e) {
		break; case HEAD_JOINT0 : { assign("HEAD_JOINT0" ); }
		break; case HEAD_JOINT1 : { assign("HEAD_JOINT1" ); }
		break; case LARM_JOINT0 : { assign("LARM_JOINT0" ); }
		break; case LARM_JOINT1 : { assign("LARM_JOINT1" ); }
		break; case LARM_JOINT2 : { assign("LARM_JOINT2" ); }
		break; case LARM_JOINT3 : { assign("LARM_JOINT3" ); }
		break; case LARM_JOINT4 : { assign("LARM_JOINT4" ); }
		break; case LARM_JOINT5 : { assign("LARM_JOINT5" ); }
		break; case LARM_JOINT6 : { assign("LARM_JOINT6" ); }
		break; case LARM_JOINT7 : { assign("LARM_JOINT7" ); }
		break; case RARM_JOINT0 : { assign("RARM_JOINT0" ); }
		break; case RARM_JOINT1 : { assign("RARM_JOINT1" ); }
		break; case RARM_JOINT2 : { assign("RARM_JOINT2" ); }
		break; case RARM_JOINT3 : { assign("RARM_JOINT3" ); }
		break; case RARM_JOINT4 : { assign("RARM_JOINT4" ); }
		break; case RARM_JOINT5 : { assign("RARM_JOINT5" ); }
		break; case RARM_JOINT6 : { assign("RARM_JOINT6" ); }
		break; case RARM_JOINT7 : { assign("RARM_JOINT7" ); }
		break; case WAIST_JOINT0: { assign("WAIST_JOINT0"); }
		break; case WAIST_JOINT1: { assign("WAIST_JOINT1"); }
		break; case WAIST_JOINT2: { assign("WAIST_JOINT2"); }
		break; case ROOT_JOINT0 : { assign("ROOT_JOINT0" ); }
		break; case ROOT_JOINT1 : { assign("ROOT_JOINT1" ); }
		break; case ROOT_JOINT2 : { assign("ROOT_JOINT2" ); }
		break; case LLEG_JOINT2 : { assign("LLEG_JOINT2" ); }
		break; case LLEG_JOINT4 : { assign("LLEG_JOINT4" ); }
		break; case LLEG_JOINT6 : { assign("LLEG_JOINT6" ); }
		break; case RLEG_JOINT2 : { assign("RLEG_JOINT2" ); }
		break; case RLEG_JOINT4 : { assign("RLEG_JOINT4" ); }
		break; case RLEG_JOINT6 : { assign("RLEG_JOINT6" ); }
		break; case LEYE_JOINT1 : { assign("LEYE_JOINT1" ); }
		break; case LEYE_JOINT0 : { assign("LEYE_JOINT0" ); }
		break; case REYE_JOINT1 : { assign("REYE_JOINT1" ); }
		break; case REYE_JOINT0 : { assign("REYE_JOINT0" ); }
		break; //default: { assign("illegal"); }
		}
	}
};

#endif // _ManJointType_

#ifndef _Vector3_
#define _Vector3_
///@brief For expression position.
typedef struct _Vector3
{
	float x;
	float y;
	float z;
} 	Vector3;
#endif

#ifndef _Vector4_
#define _Vector4_
///@brief For expression quaternion.
typedef struct _Vector4
{
	float x;
	float y;
	float z;
	float w;
} 	Vector4;
#endif // _Vector4_

class Quaternion
{
public:

	double w;
	double x;
	double y;
	double z;

	Quaternion(){};
	~Quaternion(){};

	Quaternion(const double w, const double x, const double y, const double z){
		this->w = w;
		this->x = x;
		this->y = y;
		this->z = z;
	};

	Quaternion(const Vector4 &v){
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

	std::string str()
	{
		std::stringstream ss;
		ss << "(" << w << "," << x << "," << y << "," << z << ")";
		return ss.str();
	}

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

#ifndef _EulerAngleType_
#define _EulerAngleType_
///@brief For expression position.
typedef struct _EulerAngleType
{
	float yaw;
	float pitch;
	float roll;
} 	EulerAngleType;
#endif

class ManNiiJointQuaternion
{
public:
	ManNiiJointType manNiiJointType;
	Quaternion quaternion;
};


class ManNiiPosture : public Posture
{
public:
	ManNiiJointQuaternion jointQuaternions[ManNiiJointType_Count];

	ManNiiPosture() {
		this->jointQuaternions[0].manNiiJointType = HEAD_JOINT0;
		this->jointQuaternions[1].manNiiJointType = HEAD_JOINT1;
		this->jointQuaternions[2].manNiiJointType = LARM_JOINT0,
		this->jointQuaternions[3].manNiiJointType = LARM_JOINT1;
		this->jointQuaternions[4].manNiiJointType = LARM_JOINT2;
		this->jointQuaternions[5].manNiiJointType = LARM_JOINT3;
		this->jointQuaternions[6].manNiiJointType = LARM_JOINT4;
		this->jointQuaternions[7].manNiiJointType = LARM_JOINT5;
		this->jointQuaternions[8].manNiiJointType = LARM_JOINT6;
		this->jointQuaternions[9].manNiiJointType = LARM_JOINT7;
		this->jointQuaternions[10].manNiiJointType = RARM_JOINT0;
		this->jointQuaternions[11].manNiiJointType = RARM_JOINT1;
		this->jointQuaternions[12].manNiiJointType = RARM_JOINT2;
		this->jointQuaternions[13].manNiiJointType = RARM_JOINT3;
		this->jointQuaternions[14].manNiiJointType = RARM_JOINT4;
		this->jointQuaternions[15].manNiiJointType = RARM_JOINT5;
		this->jointQuaternions[16].manNiiJointType = RARM_JOINT6;
		this->jointQuaternions[17].manNiiJointType = RARM_JOINT7;
		this->jointQuaternions[18].manNiiJointType = WAIST_JOINT0;
		this->jointQuaternions[19].manNiiJointType = WAIST_JOINT1;
		this->jointQuaternions[20].manNiiJointType = WAIST_JOINT2;
		this->jointQuaternions[21].manNiiJointType = ROOT_JOINT0;
		this->jointQuaternions[22].manNiiJointType = ROOT_JOINT1;
		this->jointQuaternions[23].manNiiJointType = ROOT_JOINT2;
		this->jointQuaternions[24].manNiiJointType = LLEG_JOINT2;
		this->jointQuaternions[25].manNiiJointType = LLEG_JOINT4;
		this->jointQuaternions[26].manNiiJointType = LLEG_JOINT6;
		this->jointQuaternions[27].manNiiJointType = RLEG_JOINT2;
		this->jointQuaternions[28].manNiiJointType = RLEG_JOINT4;
		this->jointQuaternions[29].manNiiJointType = RLEG_JOINT6;
		this->jointQuaternions[30].manNiiJointType = LEYE_JOINT1;
		this->jointQuaternions[31].manNiiJointType = LEYE_JOINT0;
		this->jointQuaternions[32].manNiiJointType = REYE_JOINT1;
		this->jointQuaternions[33].manNiiJointType = REYE_JOINT0;
		for (int i = 0; i < ManNiiJointType_Count; i++) {
			this->jointQuaternions[i].quaternion = Quaternion(0.0, 0.0, 0.0, 0.0);
		}
	}

};

#endif /* MANNIIPOSTURE_H_ */
