/*
 * ManNiiAvatarPosture.h
 *
 *  Created on: 2015/03/09
 *      Author: sigverse
 */

#ifndef MANNIIAVATARPOSTURE_H_
#define MANNIIAVATARPOSTURE_H_

#include "AvatarPosture.h"
#include <string>

#ifndef _ManNiiJointType_
#define _ManNiiJointType_

enum ManNiiJointType
{
	HEAD_JOINT0 = 0,
	HEAD_JOINT1,
	LARM_JOINT0,
	LARM_JOINT1,
	LARM_JOINT2,
	LARM_JOINT3,
	LARM_JOINT4,
	LARM_JOINT5,
	LARM_JOINT6,
	LARM_JOINT7,
	RARM_JOINT0,
	RARM_JOINT1,
	RARM_JOINT2,
	RARM_JOINT3,
	RARM_JOINT4,
	RARM_JOINT5,
	RARM_JOINT6,
	RARM_JOINT7,
	WAIST_JOINT0,
	WAIST_JOINT1,
	WAIST_JOINT2,
	ROOT_JOINT0,
	ROOT_JOINT1,
	ROOT_JOINT2,
	LLEG_JOINT2,
	LLEG_JOINT4,
	LLEG_JOINT6,
	RLEG_JOINT2,
	RLEG_JOINT4,
	RLEG_JOINT6,
	LEYE_JOINT1,
	LEYE_JOINT0,
	REYE_JOINT1,
	REYE_JOINT0,
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

	double w;
	double x;
	double y;
	double z;

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
	ManNiiJointType manJointType;
	Quaternion quaternion;
};


class ManNiiAvatarPosture : public AvatarPosture
{
public:
	ManNiiJointQuaternion joints[ManNiiJointType_Count];

};

#endif /* MANNIIAVATARPOSTURE_H_ */
