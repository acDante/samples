#ifndef _LINKAGE_
#define _LINKAGE_
/*!
* ==========================================================================================
*  @brief  get joint angles of left hand from Kinect to invert and transmit it to each part of right hand 
*  @file   linkage.h
*  @date   2013/4/18
*  @author National Institute of Informatics
*  @par    1.0.0
* ==========================================================================================
*/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "SIGService.h"
#include <tchar.h>
#include <XnCppWrapper.h>
/*!
* @brief define struct represents numbers of resources
*/
struct Quaternion {
	/*
	* @brief define real part
	*/
	float qw;

	/*!
	* @brief define imaginary x
	*/
	float qx;

	/*!
	* @brief define imaginary y
	*/
	float qy;

	/*!
	* @brief define imaginary z
	*/
	float qz;
};

/*!
* @brief define circumference ratio
*/
#define PI 3.14159265

/*!
* @brief convert degree to radian
*/
#define DEG2RAD(x) x * PI / 180.0;

/*!
* @brief play backup information from KINECT
*/
#ifdef _KINECT_PLAY
extern FILE *play;
#endif

/*!
* ------------------------------------------------------------------------------------------
* @brief calculate vector of child bone from 2 joint angle
* @param[out] XnPoint3D&              output calculated vector
* @param[in]  XnSkeltonJointPosition  input parent bone angle
* @param[in]  XnSkeltonJointPosition  input child bone angle
* ------------------------------------------------------------------------------------------
*/
bool DiffVec(XnPoint3D &rvec, XnSkeletonJointPosition jvec, XnSkeletonJointPosition kvec);

/*!
* ------------------------------------------------------------------------------------------
* @brief calculate rotation quaterion from 2 of 3D vectors
* @param[in] XnPoint3D input source vector
* @param[in] XnPoint3D input destination vector
* @return return calculeted rotation quaternion
* ------------------------------------------------------------------------------------------
*/
Quaternion CalcQuaternion(XnPoint3D kvec, XnPoint3D svec);

/*!
* ------------------------------------------------------------------------------------------
* @brief （R×P×Q：backward rotation,forward rotation in SIGVerse） return 3D vector calculated from 3D vector and rotation quaternion
* @param[in/out] XnPoint3D& input and output source 3D vector
* @param[in]     Quaterion& input rotation quaternon(parent bone: initial line, child bone: radius vector)
* @param[in]     bool       whether only X axis is valid or not
* ------------------------------------------------------------------------------------------
*/
void RotVec(XnPoint3D &v, Quaternion q, bool axis1 = false);

/*!
* ------------------------------------------------------------------------------------------
* @deprecated
* @brief （Q×P×R：forward rotation, backword rotation in SIGVerse）return 3D vector rotated from 3D vector according to rotation quaternion
* @param[in/out] XnPoint3D& input and output source 3D vector
* @param[in]     Quaterion& input rotation quaternion ( parent bone: initial line, child bone: radius vector)
* @param[in]     bool       whether only X axis is valid or not
* ------------------------------------------------------------------------------------------
*/
void RotVec_Reverse(XnPoint3D &v, Quaternion q, bool axis1 = false);

/*!
* ------------------------------------------------------------------------------------------
* @brief （Q×P×R：forward rotation, backward rotation in SIGVerse）return 3D vector rotated from 3D vector and rotation quaternion
* @param[in/out] XnPoint3D& input and output 3D vector
* @param[in]     Quaterion& input rotation quaternion ( parent bone: initial line, child bone: radius angle) 
* @param[in]     bool       whether only X axis is valid or not
* ------------------------------------------------------------------------------------------
*/
void RotVec_Xaxis_Reverse(XnPoint3D &v, Quaternion q, bool axis1 = false);

/*!
* ------------------------------------------------------------------------------------------
* @deprecated
* @brief return cross product of quaternions
* @param[in]  Quaternion input quaternion
* @param[in]  Quaternion input quaternion
* @param[out] Quaternion output calculated cross product of quaternions
* ------------------------------------------------------------------------------------------
*/
Quaternion MultiQuaternion(Quaternion p, Quaternion q);

#endif

