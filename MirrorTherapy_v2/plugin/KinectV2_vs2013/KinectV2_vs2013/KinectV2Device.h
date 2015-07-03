#pragma once
#include <SIGService.h>
#include "../../../common/device/KinectV2SensorData.h"
#include "../../../plugin/PluginCommon/Device.h"
#include <Kinect.h>
#include <opencv2/opencv.hpp>


template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

#define ENUMSTR(val) #val


class KinectV2Device : public Device
{
public:
	///@brief Parameter file name.
	static const std::string parameterFileName;

	//Parameter file information
	static const std::string paramFileKeyKinectV2SensorDataMode;
	static const std::string paramFileValKinectV2SensorDataModeDefault;

private:
	int colorFrameWidth;
	int colorFrameHeight;

	///@brief Read parameter file.
	void readIniFile();

public:
	KinectV2Device();
	KinectV2Device(int argc, char **argv);
	~KinectV2Device();

	///@brief Initialize kinect v2 device.
	int run();

	void handStateProcessing(const JointType &hand, const HandState &handState, ICoordinateMapper* &coordinateMapper, Joint* joint, cv::Mat &image);

	///@brief Set whole body joint positions.
	void convertJointPositions2KinectV2JointPositions(Joint *joints, KinectV2SensorData::KinectV2JointPosition *kinectV2JointPositions);

	///@brief Set whole body joint orientations.
	void convertJointOrientations2KinectV2JointOrientations(JointOrientation *orientations, KinectV2SensorData::KinectV2JointOrientation *kinectV2JointOrientations);

	///@brief Convert JointType to KinectV2JointType.
	KinectV2SensorData::KinectV2JointType getKinectV2JointType(const JointType jointType);

	KinectV2SensorData sensorData;


#ifdef DEBUG_PRINT
	typedef struct TIME_INFO 
	{
		SYSTEMTIME systemTime;
		float val;
	};

	void debugPrint();
	std::vector<TIME_INFO> timeInfoList;
#endif
};

