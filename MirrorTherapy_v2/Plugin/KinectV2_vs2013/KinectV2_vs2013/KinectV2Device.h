#pragma once
#include <SIGService.h>
#include <Common/KinectV2SensorData.h>
#include <Plugin/PluginCommon/Device.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>

// Parameter file name. Must be defined.
#define PARAM_FILE_NAME_KINECTV2_INI  "KinectV2.ini"


class KinectV2Device : public Device
{
private:
	///@brief Parameter file name.
	std::string parameterFileName;

	int colorFrameWidth;
	int colorFrameHeight;

	///@brief Read parameter file.
	void readIniFile();

public:
	KinectV2Device(){};
	KinectV2Device(int argc, char **argv);
	~KinectV2Device();

	///@brief Initialize kinect v2 device.
	int run();

	void handStateProcessing(const JointType &hand, const HandState &handState, ICoordinateMapper* &coordinateMapper, Joint* joint, cv::Mat &image);

	///@brief Set whole body joint orientations.
	void convertJointOrientations2KinectV2JointOrientations(JointOrientation *orientations, KinectV2SensorData::KinectV2JointOrientation *kinectV2JointOrientations) 
	{
		//void setJointOrientations(JointOrientation* orientations) {
		//kinectV2JointOrientations.resize(KinectV2JointType_Count);
		for (int i = 0; i < KinectV2SensorData::KinectV2JointType_Count; i++) 
		{
			kinectV2JointOrientations[i].jointType = getKinectV2JointType(orientations[i].JointType);
			kinectV2JointOrientations[i].orientation.w = orientations[i].Orientation.w;
			kinectV2JointOrientations[i].orientation.x = orientations[i].Orientation.x;
			kinectV2JointOrientations[i].orientation.y = orientations[i].Orientation.y;
			kinectV2JointOrientations[i].orientation.z = orientations[i].Orientation.z;
		}
	}

	///@brief Convert JointType to KinectV2JointType.
	KinectV2SensorData::KinectV2JointType getKinectV2JointType(const JointType jointType) 
	{
		switch (jointType) 
		{
			break; case JointType_SpineBase:	{ return KinectV2SensorData::SpineBase; }
			break; case JointType_SpineMid:		{ return KinectV2SensorData::SpineMid; }
			break; case JointType_Neck:			{ return KinectV2SensorData::Neck; }
			break; case JointType_Head:			{ return KinectV2SensorData::Head; }
			break; case JointType_ShoulderLeft:	{ return KinectV2SensorData::ShoulderLeft; }
			break; case JointType_ElbowLeft:	{ return KinectV2SensorData::ElbowLeft; }
			break; case JointType_WristLeft:	{ return KinectV2SensorData::WristLeft; }
			break; case JointType_HandLeft:		{ return KinectV2SensorData::HandLeft; }
			break; case JointType_ShoulderRight:{ return KinectV2SensorData::ShoulderRight; }
			break; case JointType_ElbowRight:	{ return KinectV2SensorData::ElbowRight; }
			break; case JointType_WristRight:	{ return KinectV2SensorData::WristRight; }
			break; case JointType_HandRight:	{ return KinectV2SensorData::HandRight; }
			break; case JointType_HipLeft:		{ return KinectV2SensorData::HipLeft; }
			break; case JointType_KneeLeft:		{ return KinectV2SensorData::KneeLeft; }
			break; case JointType_AnkleLeft:	{ return KinectV2SensorData::AnkleLeft; }
			break; case JointType_FootLeft:		{ return KinectV2SensorData::FootLeft; }
			break; case JointType_HipRight:		{ return KinectV2SensorData::HipRight; }
			break; case JointType_KneeRight:	{ return KinectV2SensorData::KneeRight; }
			break; case JointType_AnkleRight:	{ return KinectV2SensorData::AnkleRight; }
			break; case JointType_FootRight:	{ return KinectV2SensorData::FootRight; }
			break; case JointType_SpineShoulder:{ return KinectV2SensorData::SpineShoulder; }
			break; case JointType_HandTipLeft:	{ return KinectV2SensorData::HandTipLeft; }
			break; case JointType_ThumbLeft:	{ return KinectV2SensorData::ThumbLeft; }
			break; case JointType_HandTipRight:	{ return KinectV2SensorData::HandTipRight; }
			break; case JointType_ThumbRight:	{ return KinectV2SensorData::ThumbRight; }
			break; default: { throw std::string("This JointType is invalid. JointType enum number = "+jointType); }
		}
	}
};

