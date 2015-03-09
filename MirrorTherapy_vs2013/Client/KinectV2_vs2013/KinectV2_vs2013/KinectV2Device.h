#pragma once
#include "SIGService.h"
#include "Device.h"
#include "KinectV2SensorData.h"
#include "KinectV2Connector.h"
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

//// Default moving speed of avatar. 
//#define DEFAULT_MOVING_SPEED 1.0
//static const double defaultMovingSpeed = DEFAULT_MOVING_SPEED;

class KinectV2Device :
	public Device
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

};

