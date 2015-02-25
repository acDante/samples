#pragma once
#include <Kinect.h>
#include <string>
#include <sstream>

class Posture
{
public:
	Posture(){};
	virtual ~Posture(){};

///@brief 実装すべき関数群など
//	virtual std::string convertPosture2Message() = 0;
//	virtual void convertMessage2Posture(const std::string &message) = 0;
//	タイムスタンプ

};

///@brief Posture class for using kinect device.
class KinectPosture : public Posture
{
public:
	KinectPosture(){};
	~KinectPosture(){};

	struct RootPositionType
	{
		double x;
		double y;
		double z;
	};

	RootPositionType rootPosition;
	JointOrientation orientations[JointType::JointType_Count];
	Joint positions[JointType::JointType_Count];

};

struct jointType2String : public std::string {
	jointType2String(JointType e) {
		switch (e) {
			break; case JointType_SpineBase: { assign("JointType_SpineBase"); }
			break; case JointType_SpineMid: { assign("JointType_SpineMid"); }
			break; case JointType_Neck: { assign("JointType_Neck"); }
			break; case JointType_Head: { assign("JointType_Head"); }
			break; case JointType_ShoulderLeft: { assign("JointType_ShoulderLeft"); }
			break; case JointType_ElbowLeft: { assign("JointType_ElbowLeft"); }
			break; case JointType_WristLeft: { assign("JointType_WristLeft"); }
			break; case JointType_HandLeft: { assign("JointType_HandLeft"); }
			break; case JointType_ShoulderRight: { assign("JointType_ShoulderRight"); }
			break; case JointType_ElbowRight: { assign("JointType_ElbowRight"); }
			break; case JointType_WristRight: { assign("JointType_WristRight"); }
			break; case JointType_HandRight: { assign("JointType_HandRight"); }
			break; case JointType_HipLeft: { assign("JointType_HipLeft"); }
			break; case JointType_KneeLeft: { assign("JointType_KneeLeft"); }
			break; case JointType_AnkleLeft: { assign("JointType_AnkleLeft"); }
			break; case JointType_FootLeft: { assign("JointType_FootLeft"); }
			break; case JointType_HipRight: { assign("JointType_HipRight"); }
			break; case JointType_KneeRight: { assign("JointType_KneeRight"); }
			break; case JointType_AnkleRight: { assign("JointType_AnkleRight"); }
			break; case JointType_FootRight: { assign("JointType_FootRight"); }
			break; case JointType_SpineShoulder: { assign("JointType_SpineShoulder"); }
			break; case JointType_HandTipLeft: { assign("JointType_HandTipLeft"); }
			break; case JointType_ThumbLeft: { assign("JointType_ThumbLeft"); }
			break; case JointType_HandTipRight: { assign("JointType_HandTipRight"); }
			break; case JointType_ThumbRight: { assign("JointType_ThumbRight"); }
			break; case JointType_Count: { assign("JointType_Count"); }
			break; default: { assign("illegal"); }
		}
	}
};

struct orientation2String : public std::string {
	orientation2String(const Vector4 &orientation)
	{
		std::stringstream ssOrientation;
		ssOrientation << "(" << orientation.w << "," << orientation.x << "," << orientation.y << "," << orientation.z << ")";
		assign(ssOrientation.str());
	}
};