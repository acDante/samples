/*
 * ManNiiPosture.cpp
 *
 *  Created on: 2015/03/12
 *      Author: Nozaki
 */

#include <sigverse/controller/common/ManNiiPosture.h>

ManNiiPosture::ManNiiPosture()
{
	this->joint[HEAD_JOINT0].jointType = HEAD_JOINT0;
	this->joint[HEAD_JOINT1].jointType = HEAD_JOINT1;
	this->joint[LARM_JOINT0].jointType = LARM_JOINT0,
	this->joint[LARM_JOINT1].jointType = LARM_JOINT1;
	this->joint[LARM_JOINT2].jointType = LARM_JOINT2;
	this->joint[LARM_JOINT3].jointType = LARM_JOINT3;
	this->joint[LARM_JOINT4].jointType = LARM_JOINT4;
	this->joint[LARM_JOINT5].jointType = LARM_JOINT5;
	this->joint[LARM_JOINT6].jointType = LARM_JOINT6;
	this->joint[LARM_JOINT7].jointType = LARM_JOINT7;
	this->joint[RARM_JOINT0].jointType = RARM_JOINT0;
	this->joint[RARM_JOINT1].jointType = RARM_JOINT1;
	this->joint[RARM_JOINT2].jointType = RARM_JOINT2;
	this->joint[RARM_JOINT3].jointType = RARM_JOINT3;
	this->joint[RARM_JOINT4].jointType = RARM_JOINT4;
	this->joint[RARM_JOINT5].jointType = RARM_JOINT5;
	this->joint[RARM_JOINT6].jointType = RARM_JOINT6;
	this->joint[RARM_JOINT7].jointType = RARM_JOINT7;
	this->joint[WAIST_JOINT0].jointType = WAIST_JOINT0;
	this->joint[WAIST_JOINT1].jointType = WAIST_JOINT1;
	this->joint[WAIST_JOINT2].jointType = WAIST_JOINT2;
	this->joint[ROOT_JOINT0].jointType = ROOT_JOINT0;
	this->joint[ROOT_JOINT1].jointType = ROOT_JOINT1;
	this->joint[ROOT_JOINT2].jointType = ROOT_JOINT2;
	this->joint[LLEG_JOINT2].jointType = LLEG_JOINT2;
	this->joint[LLEG_JOINT4].jointType = LLEG_JOINT4;
	this->joint[LLEG_JOINT6].jointType = LLEG_JOINT6;
	this->joint[RLEG_JOINT2].jointType = RLEG_JOINT2;
	this->joint[RLEG_JOINT4].jointType = RLEG_JOINT4;
	this->joint[RLEG_JOINT6].jointType = RLEG_JOINT6;
	this->joint[LEYE_JOINT1].jointType = LEYE_JOINT1;
	this->joint[LEYE_JOINT0].jointType = LEYE_JOINT0;
	this->joint[REYE_JOINT1].jointType = REYE_JOINT1;
	this->joint[REYE_JOINT0].jointType = REYE_JOINT0;

	for (int i = 0; i < ManNiiJointType_Count; i++)
	{
		this->joint[i].quaternion = Quaternion(0.0, 0.0, 0.0, 0.0);
		this->joint[i].isValid    = false;
	}
}


void ManNiiPosture::clearJointValidFlag()
{
	for (int i = 0; i < ManNiiJointType_Count; i++)
	{
		this->joint[i].isValid = false;
	}
}


/*
 * Get initial vector of avatar joints.
 */
SigCmn::Vector3 ManNiiPosture::getSigVec(const SigVec &sigvec)
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
