/*!
* ==========================================================================================
*  @brief  左手の関節角の情報をKinectより取得して、符号を逆とした角度を右手の部位に渡します。
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
* @brief 2つの関節角より子ボーンのベクトルを求めます。
* @param[out] XnPoint3D&             求めたベクトルを返します。
* @param[in]  XnSkeltonJointPosition 親ボーンの関節角を渡します。
* @param[in]  XnSkeltonJointPosition 子ボーンの関節角を渡します。
* ------------------------------------------------------------------------------------------
*/
bool DiffVec(XnPoint3D &rvec, XnSkeletonJointPosition jvec, XnSkeletonJointPosition kvec) {

	if (jvec.fConfidence < 0.5 || kvec.fConfidence < 0.5) {
		return false;
	}
	rvec.X = -(kvec.position.X - jvec.position.X);
	rvec.Y = kvec.position.Y - jvec.position.Y;
	rvec.Z = -(kvec.position.Z - jvec.position.Z);

	// P=(o; x,y,z) (a^2+b^2+c^2=1)とするために正規化します
	float length = sqrt(rvec.X * rvec.X + rvec.Y * rvec.Y + rvec.Z * rvec.Z);
	rvec.X = rvec.X / length;
	rvec.Y = rvec.Y / length;
	rvec.Z = rvec.Z / length;

	return true;
}

/*!
* ------------------------------------------------------------------------------------------
* @brief 2つの3次元ベクトルより回転四元数を求めます。
* @param[in] XnPoint3D 元のベクトルを渡します。
* @param[in] XnPoint3D 先のベクトルを渡します。
* @return 求めた回転四元数を返します。
* ------------------------------------------------------------------------------------------
*/
Quaternion CalcQuaternion(XnPoint3D kvec, XnPoint3D svec) {
	Quaternion q;
	/* 同じ位置であれば解なしとして角度なし=0の回転四元数を返します */
	if (kvec.X == svec.X && kvec.Y == svec.Y && kvec.Z == svec.Z) {
		q.qw = 1; q.qx = 0; q.qy = 0; q.qz = 0;
		return q;
	}

	/* 各軸の量を取得します */
	float x = kvec.Y * svec.Z - kvec.Z * svec.Y;
	float y = kvec.Z * svec.X - kvec.X * svec.Z;
	float z = kvec.X * svec.Y - kvec.Y * svec.X;

	// P=(o; x,y,z) (a^2+b^2+c^2=1)とするために正規化します
	float sum = sqrt(x * x + y * y + z * z);
	x = x / sum;
	y = y / sum;
	z = z / sum;

	// 2つのベクトルより回転四元数を求めます
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
* @brief （R×P×Q：逆回転,SIGVerseでは順回転）三次元ベクトルと回転四元数より回転した三次元ベクトルを返します。
* @param[in/out] XnPoint3D& 元の三次元ベクトルを渡します。
* @param[in]     Quaterion& 回転四元数（親ボーンを始線、子ボーンを動径とした回転角）を渡します。
* @param[in]     bool       x軸のみの回転を有効とします。
* ------------------------------------------------------------------------------------------
*/
void RotVec(XnPoint3D &v, Quaternion q, bool axis1) {

	// x軸のみが有効な場合、y,z軸の回転を無効とします
	if (axis1 == true) {
		q.qy = 0.0;
	}

	// （R×P）ベクトルと共役四元数の積を求めます
	float rw = v.X *  q.qx + v.Y *  q.qy + v.Z *  q.qz;
	float rx = v.X *  q.qw + v.Y * -q.qz + v.Z *  q.qy;
	float ry = v.X *  q.qz + v.Y *  q.qw + v.Z * -q.qx;
	float rz = v.X * -q.qy + v.Y *  q.qx + v.Z *  q.qw;

	// （P×Q）回転四元数を掛けて回転した先のベクトルを求めます
	v.X = q.qx *  rw + q.qy *  rz + q.qz * -ry + q.qw * rx;
	v.Y = q.qx * -rz + q.qy *  rw + q.qz *  rx + q.qw * ry;
	v.Z = q.qx *  ry + q.qy * -rx + q.qz *  rw + q.qw * rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @deprecated
* @brief （Q×P×R：順回転,SIGVerseでは逆回転）三次元ベクトルと回転四元数より回転した三次元ベクトルを返します。
* @param[in/out] XnPoint3D& 元の三次元ベクトルを渡します。
* @param[in]     Quaterion& 回転四元数（親ボーンを始線、子ボーンを動径とした回転角）を渡します。
* @param[in]     bool       x軸のみの回転を有効とします。
* ------------------------------------------------------------------------------------------
*/
void RotVec_Reverse(XnPoint3D &v, Quaternion q, bool axis1) {

	// （Q×P）回転四元数を掛けて回転した先のベクトルを求めます
	// 通常の順回転を行います
	float rw = v.X * -q.qx + v.Y * -q.qy + v.Z * -q.qz;
	float rx = v.X *  q.qw + v.Y *  q.qz + v.Z * -q.qy;
	float ry = v.X * -q.qz + v.Y *  q.qw + v.Z *  q.qx;
	float rz = v.X *  q.qy + v.Y * -q.qx + v.Z *  q.qw;

	// （P×R）ベクトルと共役四元数の積を求めます
	// 通常の順回転を行いますs
	v.X = q.qx *  rw + q.qy * -rz + q.qz *  ry + q.qw * -rx;
	v.Y = q.qx *  rz + q.qy *  rw + q.qz * -rx + q.qw * -ry;
	v.Z = q.qx * -ry + q.qy *  rx + q.qz *  rw + q.qw * -rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @brief （Q×P×R：順回転,SIGVerseでは逆回転）三次元ベクトルと回転四元数より回転した三次元ベクトルを返します。
* @param[in/out] XnPoint3D& 元の三次元ベクトルを渡します。
* @param[in]     Quaterion& 回転四元数（親ボーンを始線、子ボーンを動径とした回転角）を渡します。
* @param[in]     bool       x軸のみの回転を有効とします。
* ------------------------------------------------------------------------------------------
*/
void RotVec_Xaxis_Reverse(XnPoint3D &v, Quaternion q, bool axis1) {

	// x軸のみが有効な場合、y,z軸の回転を無効とします
	if (axis1 == true) {
		q.qy = 0.0;
	}

	// （Q×P）回転四元数を掛けて回転した先のベクトルを求めます
	// ------------------------------------------------------------
	// 人体を対象とした場合、x軸（体の正面に平行な軸）について
	// 左手と右手を連動する必要があるために回転は逆である必要があり
	// x軸の符号のみを逆にしています
	float rw = v.X *  q.qx + v.Y * -q.qy + v.Z * -q.qz;
	float rx = v.X *  q.qw + v.Y *  q.qz + v.Z * -q.qy;
	float ry = v.X * -q.qz + v.Y *  q.qw + v.Z * -q.qx;
	float rz = v.X *  q.qy + v.Y *  q.qx + v.Z *  q.qw;

	// （P×R）ベクトルと共役四元数の積を求めます
	// ------------------------------------------------------------
	// 人体を対象とした場合、x軸（体の正面に平行な軸）について
	// 左手と右手を連動する必要があるために回転は逆である必要があり
	// x軸の符号のみを逆にしています
	v.X = q.qx *  rw + q.qy * -rz + q.qz *  ry + q.qw *  rx;
	v.Y = q.qx *  rz + q.qy *  rw + q.qz *  rx + q.qw * -ry;
	v.Z = q.qx * -ry + q.qy * -rx + q.qz *  rw + q.qw * -rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @deprecated
* @brief 四元数の積を求めて返します。
* @param[in]  Quaternion 四元数を渡します。
* @param[in]  Quaternion 四元数を渡します。
* @param[out] Quaternion 求めた四元数の積を返します。
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


