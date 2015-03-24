#pragma once
#include <SIGService.h>
#include <Common/KinectV2SensorData.h>
#include <Plugin/PluginCommon/Device.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>

// Parameter file name. Must be defined.
// パラメータファイルのファイル名を記述する．
#define PARAM_FILE_NAME "KinectV2.conf"
static const std::string defaultParameterFileName = PARAM_FILE_NAME;

// Default service name of this program. Must be defined.
// SIGServerとの通信で必要になるサービス名の初期値を記述する．
// 上記のPARAM_FILE_NAMEが見つからない場合，ここで定義される初期値が使われる．
#define DEFAULT_SERVICE_NAME "SIGKINECT"
static const std::string defaultServiceName = DEFAULT_SERVICE_NAME;

class KinectV2Device : public Device
{
private:

	///@brief Variables to use kinect device.
	//KinectV2Connector kinectConnector;
	int colorFrameWidth;
	int colorFrameHeight;

public:
	KinectV2Device(){};
	KinectV2Device(int argc, char **argv);
	~KinectV2Device();

	///@brief Set service name.
	void setSigServiceName();

	///@brief Initialize kinect v2 device.
	int run();

	void handStateProcessing(const JointType &hand, const HandState &handState, ICoordinateMapper* &coordinateMapper, Joint* joint, cv::Mat &image);

	///@brief Set whole body joint orientations.
	void setJointOrientations(JointOrientation* orientations) {
		//for (int i = 0; i < KinectV2JointType_Count; i++) {
		//	this->jointOrientations[i].jointType = setKinectV2JointType(orientations[i].JointType);
		//	this->jointOrientations[i].orientation = orientations[i].Orientation;
		//}
	}

	///@brief Set whole body joint orientations.
	void convertJointOrientations2KinectV2JointOrientations(JointOrientation *orientations, KinectV2JointOrientation *kinectV2JointOrientations) {
	//void setJointOrientations(JointOrientation* orientations) {
		//kinectV2JointOrientations.resize(KinectV2JointType_Count);
		for (int i = 0; i < KinectV2JointType_Count; i++) {
			kinectV2JointOrientations[i].jointType = setKinectV2JointType(orientations[i].JointType);
			kinectV2JointOrientations[i].orientation = orientations[i].Orientation;
		}
	}

	///@brief Convert JointType to KinectV2JointType.
	KinectV2JointType setKinectV2JointType(const JointType jointType) {
		switch (jointType) {
			break; case JointType_SpineBase:	{ return SpineBase; }
			break; case JointType_SpineMid:		{ return SpineMid; }
			break; case JointType_Neck:			{ return Neck; }
			break; case JointType_Head:			{ return Head; }
			break; case JointType_ShoulderLeft:	{ return ShoulderLeft; }
			break; case JointType_ElbowLeft:	{ return ElbowLeft; }
			break; case JointType_WristLeft:	{ return WristLeft; }
			break; case JointType_HandLeft:		{ return HandLeft; }
			break; case JointType_ShoulderRight:{ return ShoulderRight; }
			break; case JointType_ElbowRight:	{ return ElbowRight; }
			break; case JointType_WristRight:	{ return WristRight; }
			break; case JointType_HandRight:	{ return HandRight; }
			break; case JointType_HipLeft:		{ return HipLeft; }
			break; case JointType_KneeLeft:		{ return KneeLeft; }
			break; case JointType_AnkleLeft:	{ return AnkleLeft; }
			break; case JointType_FootLeft:		{ return FootLeft; }
			break; case JointType_HipRight:		{ return HipRight; }
			break; case JointType_KneeRight:	{ return KneeRight; }
			break; case JointType_AnkleRight:	{ return AnkleRight; }
			break; case JointType_FootRight:	{ return FootRight; }
			break; case JointType_SpineShoulder:{ return SpineShoulder; }
			break; case JointType_HandTipLeft:	{ return HandTipLeft; }
			break; case JointType_ThumbLeft:	{ return ThumbLeft; }
			break; case JointType_HandTipRight:	{ return HandTipRight; }
			break; case JointType_ThumbRight:	{ return ThumbRight; }
			break; default: {}
		}
	}
};

