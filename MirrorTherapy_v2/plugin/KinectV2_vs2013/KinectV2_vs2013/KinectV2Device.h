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
	enum SmoothingType
	{
		NONE,
		SMA,
		WMA,
	};

	///@brief Parameter file name.
	static const std::string parameterFileName;

	//Parameter file info (key)
	static const std::string paramFileKeyKinectV2SensorDataMode;
	static const std::string paramFileKeyKinectV2SmoothingType;
	static const std::string paramFileKeyKinectV2SmoothingSMANum;
	static const std::string paramFileKeyKinectV2SmoothingWMAWeight;

	//Parameter file info (default value)
	static const std::string paramFileValKinectV2SensorDataModeDefault;
	static const std::string paramFileValKinectV2SmoothingTypeDefault;
	static const std::string paramFileValKinectV2SmoothingSMANumDefault;
	static const std::string paramFileValKinectV2SmoothingWMAWeightDefault;

private:
	SmoothingType smoothingType;
	int    smoothingSMANum;
	double smoothingWMAWeight;

	int colorFrameWidth;
	int colorFrameHeight;

	///@brief Read parameter file.
	void readIniFile();

	void handStateProcessing(const JointType &hand, const HandState &handState, ICoordinateMapper* &coordinateMapper, Joint* joint, cv::Mat &image);

	///@brief Set whole body joint positions.
	void convertJointPositions2KinectV2JointPositions(Joint *joints, KinectV2SensorData::KinectV2JointPosition *kinectV2JointPositions);

	///@brief Set whole body joint orientations.
	void convertJointOrientations2KinectV2JointOrientations(JointOrientation *orientations, KinectV2SensorData::KinectV2JointOrientation *kinectV2JointOrientations);

	///@brief Convert JointType to KinectV2JointType.
	KinectV2SensorData::KinectV2JointType getKinectV2JointType(const JointType jointType);

	///@brief Set smoothing information.
	void setSmoothingInfo(std::string typeStr, std::string smaNumStr, std::string wmaWeightStr);

	///@brief Only get latest sensor data (No Smoothing)
	KinectV2SensorData getLatestSensorData(const std::vector<KinectV2SensorData> &sensorDataList);

	///@brief Smoothing by Simple Moving Average.
	KinectV2SensorData smoothingBySMA(const std::vector<KinectV2SensorData> &sensorDataList);

	///@brief Smoothing by Weighted Moving Average.
	KinectV2SensorData smoothingByWMA(const std::vector<KinectV2SensorData> &sensorDataList);
public:
	KinectV2Device();
	KinectV2Device(int argc, char **argv);
	~KinectV2Device();

	///@brief Execute Kinect v2 device.
	int run();


	std::string sensorDataModeStr;

	std::vector<KinectV2SensorData> sensorDataList;
	unsigned int sensorDataListNum;


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

