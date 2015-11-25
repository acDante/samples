/*!
* ==========================================================================================
*  @brief   caluculate inverted angle from left joint angle of Kinect, tlansmit it to each joint of right hand
*  @file   linkage.cpps
*  @date   2013/4/18
*  @author National Institute of Informatics
*  @par    1.0.0
* ==========================================================================================
*/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "linkage.h"

/*!
* ------------------------------------------------------------------------------------------
* @brief calculate vector of child bone from 2 joint angle 
* @param[out] XnPoint3D&             output vector
* @param[in]  XnSkeltonJointPosition input joint ange of parent bone
* @param[in]  XnSkeltonJointPosition input joint ange of child bone
* ------------------------------------------------------------------------------------------
*/
bool DiffVec(XnPoint3D &rvec, XnSkeletonJointPosition jvec, XnSkeletonJointPosition kvec) {

	if (jvec.fConfidence < 0.5 || kvec.fConfidence < 0.5) {
		return false;
	}
	rvec.X = -(kvec.position.X - jvec.position.X);
	rvec.Y = kvec.position.Y - jvec.position.Y;
	rvec.Z = -(kvec.position.Z - jvec.position.Z);

	//normalizing:   P=(o; x,y,z) (a^2+b^2+c^2=1)
	float length = sqrt(rvec.X * rvec.X + rvec.Y * rvec.Y + rvec.Z * rvec.Z);
	rvec.X = rvec.X / length;
	rvec.Y = rvec.Y / length;
	rvec.Z = rvec.Z / length;

	return true;
}

/*!
* ------------------------------------------------------------------------------------------
* @brief calculate quaternion from 2 of 3D vectors
* @param[in] XnPoint3D input source vector
* @param[in] XnPoint3D output destination vector
* @return calculated quaternion
* ------------------------------------------------------------------------------------------
*/
Quaternion CalcQuaternion(XnPoint3D kvec, XnPoint3D svec) {
	Quaternion q;
	// return 0 angle quaternion if same position
	if (kvec.X == svec.X && kvec.Y == svec.Y && kvec.Z == svec.Z) {
		q.qw = 1; q.qx = 0; q.qy = 0; q.qz = 0;
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
	// Q=(cos(θ/2); a sin(θ/2), b sin(θ/2), c sin(θ/2))
	float angle = acos(kvec.X * svec.X + kvec.Y * svec.Y + kvec.Z * svec.Z);
	q.qw = cos(angle / 2);
	q.qx = x * sin(angle / 2);
	q.qy = y * sin(angle / 2);
	q.qz = z * sin(angle / 2);

	return q;
}

/*!
* ------------------------------------------------------------------------------------------
* @brief （R×P×Q：backward rotation:forward rotation in SIGVerse）calculate 3D vector rotated from 3D vector and rotation quaternion 
* @param[in/out] XnPoint3D& input and output source 3D vector
* @param[in]     Quaterion& input rotation quaternion(parent bone: initial line, child bone: radius vector)
* @param[in]     bool       whether only X axis is valid or not
* ------------------------------------------------------------------------------------------
*/
void RotVec(XnPoint3D &v, Quaternion q, bool axis1) {

	//invalidate Y and Z axes rotation if only X axis is valid
	if (axis1 == true) {
		q.qy = 0.0;
	}

	// （R×P）calculate cross product of vector and quaternion
	float rw = v.X *  q.qx + v.Y *  q.qy + v.Z *  q.qz;
	float rx = v.X *  q.qw + v.Y * -q.qz + v.Z *  q.qy;
	float ry = v.X *  q.qz + v.Y *  q.qw + v.Z * -q.qx;
	float rz = v.X * -q.qy + v.Y *  q.qx + v.Z *  q.qw;

	// （P×Q）calculate vector rotated by quaternion
	v.X = q.qx *  rw + q.qy *  rz + q.qz * -ry + q.qw * rx;
	v.Y = q.qx * -rz + q.qy *  rw + q.qz *  rx + q.qw * ry;
	v.Z = q.qx *  ry + q.qy * -rx + q.qz *  rw + q.qw * rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @deprecated
* @brief （Q×P×R：forward rotation,backward rotation in SIGVerse）calculate 3D vector from 3D vector and rotation quaternion
* @param[in/out] XnPoint3D& input and output of source 3D vector
* @param[in]     Quaterion& input rotation quaternion(parent bone: initial line, child bone: radius vector)
* @param[in]     bool       wheter only X axis is valid or not
* ------------------------------------------------------------------------------------------
*/
void RotVec_Reverse(XnPoint3D &v, Quaternion q, bool axis1) {

	// （Q×P）calculate vector ratated by quaternion
	// forward rotation
	float rw = v.X * -q.qx + v.Y * -q.qy + v.Z * -q.qz;
	float rx = v.X *  q.qw + v.Y *  q.qz + v.Z * -q.qy;
	float ry = v.X * -q.qz + v.Y *  q.qw + v.Z *  q.qx;
	float rz = v.X *  q.qy + v.Y * -q.qx + v.Z *  q.qw;

	// （P×R）calculate cross product of vector and quaternion
	// forward rotation
	v.X = q.qx *  rw + q.qy * -rz + q.qz *  ry + q.qw * -rx;
	v.Y = q.qx *  rz + q.qy *  rw + q.qz * -rx + q.qw * -ry;
	v.Z = q.qx * -ry + q.qy *  rx + q.qz *  rw + q.qw * -rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @brief （Q×P×R：forward rotation, backward rotation in SIGVerse）return 3D vector rotated from 3D vector and rotation quaternion
* @param[in/out] XnPoint3D& input and output 3D vector
* @param[in]     Quaterion& input rotation quaternion(parent bone: initial line, child bone: radius vector)
* @param[in]     bool       whether only X axis is valid or not
* ------------------------------------------------------------------------------------------
*/
void RotVec_Xaxis_Reverse(XnPoint3D &v, Quaternion q, bool axis1) {

	//invalidate Y and Z axes when only X axis is valid
	if (axis1 == true) {
		q.qy = 0.0;
	}

	// （Q×P）calculate vector by rotation quaternion
	// ------------------------------------------------------------
	// In X axis(parallel to front of body) of human body, 
	// rotation directions of left hand and right hand must be opposite, 
	// invert only X value
	float rw = v.X *  q.qx + v.Y * -q.qy + v.Z * -q.qz;
	float rx = v.X *  q.qw + v.Y *  q.qz + v.Z * -q.qy;
	float ry = v.X * -q.qz + v.Y *  q.qw + v.Z * -q.qx;
	float rz = v.X *  q.qy + v.Y *  q.qx + v.Z *  q.qw;

	// （P×R）calculate cross product of vector and conjugate quaternion
	// ------------------------------------------------------------
	// In X axis(parallel to front of body) of human body, 
	// rotation directions of left hand and right hand must be opposite, 
	// invert only X value
	v.X = q.qx *  rw + q.qy * -rz + q.qz *  ry + q.qw *  rx;
	v.Y = q.qx *  rz + q.qy *  rw + q.qz *  rx + q.qw * -ry;
	v.Z = q.qx * -ry + q.qy * -rx + q.qz *  rw + q.qw * -rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @deprecated
* @brief return cross product of quaternions
* @param[in]  Quaternion input quaternion1
* @param[in]  Quaternion input quaternion2
* @param[out] Quaternion output cross product of quaternions
* ------------------------------------------------------------------------------------------
*/
Quaternion MultiQuaternion(Quaternion p, Quaternion q) {

	Quaternion r;
	r.qw = p.qw * q.qw - p.qx * q.qx - p.qy * q.qy - p.qz * q.qz;
	r.qx = p.qw * q.qx + p.qx * q.qw + p.qy * q.qz - p.qz * q.qy;
	r.qy = p.qw * q.qy - p.qx * q.qz + p.qy * q.qw + p.qz * q.qx;
	r.qz = p.qw * q.qz + p.qx * q.qy - p.qy * q.qx + p.qz * q.qw;

	return r;
}


