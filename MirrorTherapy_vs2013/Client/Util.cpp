#include "Util.h"

static const float SIGVEC_FOOT_Y_ORIENTATION = -0.50;
static const float SIGVEC_FOOT_Z_ORIENTATION = +0.8660254f;

///@brief Generate string from joint orientation information.
///@param joint Joint name and joint quaternion.
///@return Generated string.
std::string Util::jointOrientation2String(const JointOrientation& joint)
{
	std::stringstream ss;
	ss << jointType2String(joint.JointType) << ":" << orientation2String(joint.Orientation);
	return ss.str();
}

///@brief Constructor.
Quaternion::Quaternion()
{
}

///@brief Constructor.
Quaternion::Quaternion(const double w, const double x, const double y, const double z)
{
	this->w = w;
	this->x = x;
	this->y = y;
	this->z = z;
}

Quaternion::Quaternion(const Vector4 &v)
{
	this->w = v.w;
	this->x = v.x;
	this->y = v.y;
	this->z = v.z;
}


///@brief Cross product of quaternions.
///@param[in] q Input quaternion.
///@param[in] r Input quaternion.
///@return Cross product of two input quaternions.
Quaternion Util::calcCrossProduct(const Quaternion &q, const Quaternion &r)
{
	Quaternion tmp;
	tmp.w = q.w * r.w - q.x * r.x - q.y * r.y - q.z * r.z;
	tmp.x = q.w * r.x + q.x * r.w + q.y * r.z - q.z * r.y;
	tmp.y = q.w * r.y - q.x * r.z + q.y * r.w + q.z * r.x;
	tmp.z = q.w * r.z + q.x * r.y - q.y * r.x + q.z * r.w;
	return tmp;
}


std::string Util::generate(const std::map<std::string, Quaternion> &mq, const double x, const double y, const double z)
{

}

std::string Util::getJointNameAndQuaternion(const std::string &jointName, const Quaternion &q)
{
	std::stringstream ss;
	ss << jointName << ":" << q.w << "," << q.x << "," << q.y << "," << q.z;
	return ss.str();
}

/*!
* ------------------------------------------------------------------------------------------
* @brief calculate quaternion from 2 of 3D vectors
* @param[in] XnPoint3D input source vector
* @param[in] XnPoint3D output destination vector
* @return calculated quaternion
* ------------------------------------------------------------------------------------------
*/
Quaternion Util::calcQuaternion(const XnPoint3D &kvec, const XnPoint3D &svec) {
	Quaternion q;
	// return 0 angle quaternion if same position
	if (kvec.X == svec.X && kvec.Y == svec.Y && kvec.Z == svec.Z) {
		q.w = 1; q.x = 0; q.y = 0; q.z = 0;
		return q;
	}

	//get values of each axis
	float x = kvec.Y * svec.Z - kvec.Z * svec.Y;
	float y = kvec.Z * svec.X - kvec.X * svec.Z;
	float z = kvec.X * svec.Y - kvec.Y * svec.X;

	//normalizing:   P=(o; x,y,z) (a^2+b^2+c^2=1)
	float sum = sqrt(x * x + y * y + z * z);
	x = x / sum;
	y = y / sum;
	z = z / sum;

	//calculate quaternion from 2 vectors
	// Q=(cos(ƒÆ/2); a sin(ƒÆ/2), b sin(ƒÆ/2), c sin(ƒÆ/2))
	float angle = acos(kvec.X * svec.X + kvec.Y * svec.Y + kvec.Z * svec.Z);
	q.w = cos(angle / 2);
	q.x = x * sin(angle / 2);
	q.y = y * sin(angle / 2);
	q.z = z * sin(angle / 2);

	return q;
}

bool Util::diffVec(XnPoint3D &rvec, const XnSkeletonJointPosition &jvec, const XnSkeletonJointPosition &kvec)
{
	if (jvec.fConfidence < 0.5 || kvec.fConfidence < 0.5)
	{
		return false;
	}

	//in SIGVerse:
	//left for man:+
	//up for man:+
	//forward for man:+
	rvec.X = -(kvec.position.X - jvec.position.X);
	rvec.Y = kvec.position.Y - jvec.position.Y;
	rvec.Z = -(kvec.position.Z - jvec.position.Z);

	double length = sqrt(rvec.X*rvec.X + rvec.Y*rvec.Y + rvec.Z*rvec.Z);
	rvec.X = (XnFloat)(rvec.X / length);
	rvec.Y = (XnFloat)(rvec.Y / length);
	rvec.Z = (XnFloat)(rvec.Z / length);
	return true;
}

void Util::rotateVec(XnPoint3D &v, const Quaternion &q)
{
	double rx = (double)(v.X *  q.w + v.Y * -q.z + v.Z *  q.y);
	double ry = (double)(v.X *  q.z + v.Y *  q.w + v.Z * -q.x);
	double rz = (double)(v.X * -q.y + v.Y *  q.x + v.Z *  q.w);
	double rw = (double)(v.X *  q.x + v.Y *  q.y + v.Z *  q.z);

	v.X = (XnFloat)(q.x *  rw + q.y *  rz + q.z * -ry + q.w * rx);
	v.Y = (XnFloat)(q.x * -rz + q.y *  rw + q.z *  rx + q.w * ry);
	v.Z = (XnFloat)(q.x *  ry + q.y * -rx + q.z *  rw + q.w * rz);
}


//values of joint angles in SIGVerse
XnPoint3D Util::getSigVec(const int sigvec)
{
	XnPoint3D p;
	if (sigvec == WAIST || sigvec == HTOTOR)
	{
		p.X = 0;
		p.Y = 1;
		p.Z = 0;
	}
	else if (sigvec == RSHOULDER || sigvec == RELBOW)
	{
		p.X = -1;
		p.Y = 0;
		p.Z = 0;
	}
	else if (sigvec == HIP || sigvec == LSHOULDER || sigvec == LELBOW)
	{
		p.X = 1;
		p.Y = 0;
		p.Z = 0;
	}
	else if (sigvec == LEG)
	{
		p.X = 0;
		p.Y = -1;
		p.Z = 0;
	}
	else if (sigvec == FOOT)
	{
		p.X = 0;
		p.Y = SIGVEC_FOOT_Y_ORIENTATION;
		p.Z = SIGVEC_FOOT_Z_ORIENTATION;
	}
	return p;
}
