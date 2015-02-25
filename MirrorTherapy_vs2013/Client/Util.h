#pragma once

#include "Kinect.h"
#include "KinectPosture.h"
#include <sstream>
#include <string>
#include <map>
#include <XnCppWrapper.h>

class Quaternion; // Forward decelaration.

class Util
{
public:

	///@brief Generate string from joint orientation information.
	///@param joint Joint name and joint quaternion.
	///@return Generated string.
	static std::string jointOrientation2String(const JointOrientation& joint);


	///@brief Cross product of quaternions.
	///@param[in] q Input quaternion.
	///@param[in] r Input quaternion.
	///@return Cross product of two input quaternions.
	static Quaternion calcCrossProduct(const Quaternion &q, const Quaternion &r);

	///@brief Generate massage from joint-name and quaternion map.
	static std::string generate(const std::map<std::string, Quaternion> &mq, const double x, const double y, const double z);

	///@brief Get joint name and its quaternion.
	static std::string getJointNameAndQuaternion(const std::string &jointName, const Quaternion &q);

	static Quaternion calcQuaternion(const XnPoint3D &kvec, const XnPoint3D &svec);

	static bool diffVec(XnPoint3D &rvec, const XnSkeletonJointPosition &jvec, const XnSkeletonJointPosition &kvec);

	static void rotateVec(XnPoint3D &v, const Quaternion &q);
	
	//values of joint angles in SIGVerse
	static XnPoint3D Util::getSigVec(const int sigvec);

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
};

class Quaternion
{
public:
	Quaternion();
	~Quaternion(){};

	Quaternion(const double w, const double x, const double y, const double z);

	Quaternion(const Vector4 &v);

	double w;
	double x;
	double y;
	double z;

};

