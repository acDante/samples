#pragma once
#include "SensorData.h"

#include <string>
#include <sstream>
#include <iomanip>

// Set precision of quaternion and position. Only for send information.
#define ORIENTATION_PRECISION 5
#define POSITION_PRECISION 5

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
#endif //_Vector4_

#ifndef _KinectV2JointType_
#define _KinectV2JointType_

enum KinectV2JointType
{
	SpineBase		= 0,
	SpineMid		= 1,
	Neck			= 2,
	Head			= 3,
	ShoulderLeft	= 4,
	ElbowLeft		= 5,
	WristLeft		= 6,
	HandLeft		= 7,
	ShoulderRight	= 8,
	ElbowRight		= 9,
	WristRight		= 10,
	HandRight		= 11,
	HipLeft			= 12,
	KneeLeft		= 13,
	AnkleLeft		= 14,
	FootLeft		= 15,
	HipRight		= 16,
	KneeRight		= 17,
	AnkleRight		= 18,
	FootRight		= 19,
	SpineShoulder	= 20,
	HandTipLeft		= 21,
	ThumbLeft		= 22,
	HandTipRight	= 23,
	ThumbRight		= 24,
	KinectV2JointType_Count = (ThumbRight + 1)
};
#endif // _KinectJointType_

#ifndef _KinectV2JointOrientation_
#define _KinectV2JointOrientation_
typedef struct _KinectV2JointOrientation
{
	KinectV2JointType jointType;
	Vector4 orientation;
} 	KinectV2JointOrientation;

#endif // _JointOrientation_


///@brief Posture class for using kinect v2 device.
class KinectV2SensorData :
	public SensorData
{
private:
	///@brief Root body position.
	Vector3 rootPosition;

	///@brief Whole body joint orientations.
	KinectV2JointOrientation jointOrientations[KinectV2JointType_Count];

	// 現状ではルートのポジションと各関節のオリエンテーション（クォータニオン）しか保持していませんが，
	// KinectV2JointPosition などの型を定義して，関節の位置を保持しても問題ないと思います．

public:

	///@brief Precision of quaternion when send message.
	static const int orientationPrecision = ORIENTATION_PRECISION;

	///@brief Precision of root position when send message.
	static const int rootPositionPrecision = POSITION_PRECISION;

	///@brief Generate message by posture.
	std::string encodeSensorData2Message(const std::string &pairsDelim = pairsDelimDefault, const std::string &keyValueDelim = keyValueDelimDefault, const std::string &vectorDelim = vectorDelimDefault);

	///@brief Decode message to sensor data.
	void decodeMessage2SensorData(const std::string &message, const std::string &pairsDelim = pairsDelimDefault, const std::string &keyValueDelim = keyValueDelimDefault, const std::string &vectorDelim = vectorDelimDefault);

	///@brief Set root position.
	void setRootPosition(const Vector3 &position) {
		this->rootPosition = position;
	}

	void setKinectV2JointOrientation(KinectV2JointOrientation *kinectV2JointOrientation);

	std::vector<KinectV2JointOrientation> getKinectV2JointOrientation();

	KinectV2JointType shortJointName2KinectV2JointType(const std::string &shortJointName);
};

///@brief Get string enum of JointType.
struct jointType2String : public std::string {
	jointType2String(KinectV2JointType e) {
		switch (e) {
			break; case SpineBase:		{ assign("SpineBase"); }
			break; case SpineMid:		{ assign("SpineMid"); }
			break; case Neck:			{ assign("Neck"); }
			break; case Head:			{ assign("Head"); }
			break; case ShoulderLeft:	{ assign("ShoulderLeft"); }
			break; case ElbowLeft:		{ assign("ElbowLeft"); }
			break; case WristLeft:		{ assign("WristLeft"); }
			break; case HandLeft:		{ assign("HandLeft"); }
			break; case ShoulderRight:	{ assign("ShoulderRight"); }
			break; case ElbowRight:		{ assign("ElbowRight"); }
			break; case WristRight:		{ assign("WristRight"); }
			break; case HandRight:		{ assign("HandRight"); }
			break; case HipLeft:		{ assign("HipLeft"); }
			break; case KneeLeft:		{ assign("KneeLeft"); }
			break; case AnkleLeft:		{ assign("AnkleLeft"); }
			break; case FootLeft:		{ assign("FootLeft"); }
			break; case HipRight:		{ assign("HipRight"); }
			break; case KneeRight:		{ assign("KneeRight"); }
			break; case AnkleRight:		{ assign("AnkleRight"); }
			break; case FootRight:		{ assign("FootRight"); }
			break; case SpineShoulder:	{ assign("SpineShoulder"); }
			break; case HandTipLeft:	{ assign("HandTipLeft"); }
			break; case ThumbLeft:		{ assign("ThumbLeft"); }
			break; case HandTipRight:	{ assign("HandTipRight"); }
			break; case ThumbRight:		{ assign("ThumbRight"); }
			break; default:				{ assign("illegal"); }
		}
	}
};

///@brief Get string enum of JointType to genrate message.
struct jointType2ShortJointName : public std::string {
	jointType2ShortJointName(KinectV2JointType e) {
		switch (e) {
			break; case SpineBase:		{ assign("SpBs_Q"); }
			break; case SpineMid:		{ assign("SpMd_Q"); }
			break; case Neck:			{ assign("Neck_Q"); }
			break; case Head:			{ assign("Head_Q"); }
			break; case ShoulderLeft:	{ assign("ShL_Q"); }
			break; case ElbowLeft:		{ assign("LbL_Q"); }
			break; case WristLeft:		{ assign("WrL_Q"); }
			break; case HandLeft:		{ assign("HndL_Q"); }
			break; case ShoulderRight:	{ assign("ShR_Q"); }
			break; case ElbowRight:		{ assign("LbR_Q"); }
			break; case WristRight:		{ assign("WrR_Q"); }
			break; case HandRight:		{ assign("HndR_Q"); }
			break; case HipLeft:		{ assign("HpL_Q"); }
			break; case KneeLeft:		{ assign("NeeL_Q"); }
			break; case AnkleLeft:		{ assign("AnkL_Q"); }
			break; case FootLeft:		{ assign("FtL_Q"); }
			break; case HipRight:		{ assign("HpR_Q"); }
			break; case KneeRight:		{ assign("NeeR_Q"); }
			break; case AnkleRight:		{ assign("AnkR_Q"); }
			break; case FootRight:		{ assign("FtR_Q"); }
			break; case SpineShoulder:	{ assign("SpSh_Q"); }
			break; case HandTipLeft:	{ assign("HTL_Q"); }
			break; case ThumbLeft:		{ assign("ThmL_Q"); }
			break; case HandTipRight:	{ assign("HTR_Q"); }
			break; case ThumbRight:		{ assign("ThmR_Q"); }
			break; default:				{ assign("illegal"); }
		}
	}
};
