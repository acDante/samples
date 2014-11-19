#ifndef _LINKAGE_
#define _LINKAGE_
/*!
* ==========================================================================================
*  @brief  左手の関節角の情報をKinectより取得して、符号を逆とした角度を右手の部位に渡します。 get joint angles of left hand fromr Kinect to invert and transmit it to each part of right hand 
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
* @brief 資源数を表す構造体を定義します。define struct represents numbers of resources
*/
struct Quaternion {
	/*
	* @brief 実数部を定義します。define real
	*/
	float qw;

	/*!
	* @brief 虚数部(x軸)を定義します。define imaginary x
	*/
	float qx;

	/*!
	* @brief 虚数部(y軸)を定義します。define imaginary y
	*/
	float qy;

	/*!
	* @brief 虚数部(z軸)を定義します。define imaginary z
	*/
	float qz;
};

/*!
* @brief 円周率を定義します。define circumference ratio
*/
#define PI 3.14159265

/*!
* @brief 度数を弧度に変換します。convert degree to radian
*/
#define DEG2RAD(x) x * PI / 180.0;

/*!
* @brief KINECTより以前に取得して保管した情報を再生します。play backup information from KINECT
*/
#ifdef _KINECT_PLAY
extern FILE *play;
#endif

/*!
* ------------------------------------------------------------------------------------------
* @brief 2つの関節角より子ボーンのベクトルを求めます。
* @param[out] XnPoint3D&             求めたベクトルを返します。  output calculated vector
* @param[in]  XnSkeltonJointPosition 親ボーンの関節角を渡します。input parent bone angle
* @param[in]  XnSkeltonJointPosition 子ボーンの関節角を渡します。input child bone angle
* ------------------------------------------------------------------------------------------
*/
bool DiffVec(XnPoint3D &rvec, XnSkeletonJointPosition jvec, XnSkeletonJointPosition kvec);

/*!
* ------------------------------------------------------------------------------------------
* @brief 2つの3次元ベクトルより回転四元数を求めます。 calculate rotation quaterion
* @param[in] XnPoint3D 元のベクトルを渡します。 input source vector
* @param[in] XnPoint3D 先のベクトルを渡します。input destination vector
* @return 求めた回転四元数を返します。 return calculeted rotation quaternion
* ------------------------------------------------------------------------------------------
*/
Quaternion CalcQuaternion(XnPoint3D kvec, XnPoint3D svec);

/*!
* ------------------------------------------------------------------------------------------
* @brief （R×P×Q：逆回転,SIGVerseでは順回転）三次元ベクトルと回転四元数より回転した三次元ベクトルを返します。  return 3D vector calculated from 3D vector and rotation quaternion
* @param[in/out] XnPoint3D& 元の三次元ベクトルを渡します。 input and output source 3D vector
* @param[in]     Quaterion& 回転四元数（親ボーンを始線、子ボーンを動径とした回転角）を渡します。 input rotation quaternon
* @param[in]     bool       x軸のみの回転を有効とします。 whether validate only X axis or not
* ------------------------------------------------------------------------------------------
*/
void RotVec(XnPoint3D &v, Quaternion q, bool axis1 = false);

/*!
* ------------------------------------------------------------------------------------------
* @deprecated
* @brief （Q×P×R：順回転,SIGVerseでは逆回転）三次元ベクトルと回転四元数より回転した三次元ベクトルを返します。 return 3D vector rotated from 3D vector according to rotation quaternion
* @param[in/out] XnPoint3D& 元の三次元ベクトルを渡します。input and output source 3D vector
* @param[in]     Quaterion& 回転四元数（親ボーンを始線、子ボーンを動径とした回転角）を渡します。 input rotation quaternion
* @param[in]     bool       x軸のみの回転を有効とします。 whether validates only X axis or not
* ------------------------------------------------------------------------------------------
*/
void RotVec_Reverse(XnPoint3D &v, Quaternion q, bool axis1 = false);

/*!
* ------------------------------------------------------------------------------------------
* @brief （Q×P×R：順回転,SIGVerseでは逆回転）三次元ベクトルと回転四元数より回転した三次元ベクトルを返します。
* @param[in/out] XnPoint3D& 元の三次元ベクトルを渡します。
* @param[in]     Quaterion& 回転四元数（親ボーンを始線、子ボーンを動径とした回転角）を渡します。
* @param[in]     bool       x軸のみの回転を有効とします。
* ------------------------------------------------------------------------------------------
*/
void RotVec_Xaxis_Reverse(XnPoint3D &v, Quaternion q, bool axis1 = false);

/*!
* ------------------------------------------------------------------------------------------
* @deprecated
* @brief 四元数の積を求めて返します。 return cross product of quaternions
* @param[in]  Quaternion 四元数を渡します。 input quaternion
* @param[in]  Quaternion 四元数を渡します。 input quaternion
* @param[out] Quaternion 求めた四元数の積を返します。 output calculated cross product of quaternions
* ------------------------------------------------------------------------------------------
*/
Quaternion MultiQuaternion(Quaternion p, Quaternion q);

#endif

