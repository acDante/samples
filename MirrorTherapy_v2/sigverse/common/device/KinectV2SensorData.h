#ifndef __KINECT_V2_SENSOR_DATA_H__
#define __KINECT_V2_SENSOR_DATA_H__

#include "../Quaternion.h"
#include "../SigCmn.h"
#include "SensorData.h"

#include <string>
#include <sstream>
#include <iomanip>

// Set precision of quaternion and position. Only for send information.
#define ORIENTATION_PRECISION 5
#define POSITION_PRECISION 5

#define MSG_KEY_ROOT_P "Root_P"


///@brief Posture class for using kinect v2 device.
class KinectV2SensorData : public SensorData
{
public:
	enum SensorDataMode
	{
		POSITION,
		QUATERNION,
	};

	enum KinectV2JointType
	{
		SpineBase       = 0,
		SpineMid        = 1,
		Neck            = 2,
		Head            = 3,
		ShoulderLeft    = 4,
		ElbowLeft       = 5,
		WristLeft       = 6,
		HandLeft        = 7,
		ShoulderRight   = 8,
		ElbowRight      = 9,
		WristRight      = 10,
		HandRight       = 11,
		HipLeft         = 12,
		KneeLeft        = 13,
		AnkleLeft       = 14,
		FootLeft        = 15,
		HipRight        = 16,
		KneeRight       = 17,
		AnkleRight      = 18,
		FootRight       = 19,
		SpineShoulder   = 20,
		HandTipLeft     = 21,
		ThumbLeft       = 22,
		HandTipRight    = 23,
		ThumbRight      = 24,
		KinectV2JointType_Count = (ThumbRight + 1)
	};
	
	enum KinectV2TrackingState
	{
		NotTracked = 0,
		Inferred,
		Tracked,
	};


	typedef struct _KinectV2JointPosition
	{
		KinectV2JointType     jointType;
		SigCmn::Vector3       position;
		KinectV2TrackingState trackingState;
	} KinectV2JointPosition;

	typedef struct _KinectV2JointOrientation
	{
		KinectV2JointType jointType;
		Quaternion        orientation;
	} KinectV2JointOrientation;


private:
	///@brief root position (Root_P, x, y, z) to string.
	std::string rootPosition2Message(const SigCmn::Vector3 &position, const std::string &keyValueDelim, const std::string &valuesDelim) const;

	///@brief Three dimensional position (x, y, z) to string.
	std::string position2Message(const SigCmn::Vector3 &position, const std::string &valuesDelim) const;

	///@brief JointPosition (jointname, x, y, z) to string.
	std::string jointPosition2Message(const KinectV2JointPosition &jp, const std::string &keyValueDelim, const std::string &valuesDelim) const;

	std::string orientation2Message(const Quaternion &orientation, const std::string &valuesDelim) const;

	///@brief JointOrientation (jointname, w, x, y, z) to string.
	std::string jointOrientation2Message(const KinectV2JointOrientation &jo, const std::string &keyValueDelim, const std::string &valuesDelim) const;

	std::string jointType2String(KinectV2SensorData::KinectV2JointType e) const;

	std::string jointType2ShortJointName(KinectV2JointType e) const;

public:

	///@brief Precision of quaternion when send message.
	static const int orientationPrecision = ORIENTATION_PRECISION;

	///@brief Precision of root position when send message.
	static const int rootPositionPrecision = POSITION_PRECISION;

	//sensor data mode.("Quaternion" or "Position")
	static SensorDataMode sensorDataMode;

	///@brief Constructor
	static void setSensorDataMode(std::string sensorDataModeStr);

	///@brief Convert sensor information to string.
	///@param itemsDelim Symbol to split items.
	///@param keyValueDelim Symbol to split KEY and VALUE.
	///@param valuesDelim Symbol to split each componet of VALUE. When VALUE is multidimensional data, you use this symbol.
	std::string encodeSensorData(const std::string &itemsDelim = ITEMS_DELIMITER, const std::string &keyValueDelim = KEY_AND_VALUE_DELIMITER, const std::string &valuesDelim = VALUES_DELIMITER) const;

	///@brief Set to sensor data.
	///@param Map of sensor data;
	bool setSensorData(const std::map<std::string, std::vector<std::string> > &sensorDataMap);

	///@brief Set root position.
	void setRootPosition(const SigCmn::Vector3 &position);

	///@brief Set posture from kinect joint positions.
	void setKinectV2JointPosition(KinectV2JointPosition *kinectV2JointPositions);

	///@brief Get orientations from kinect joint positions.
	void getKinectV2JointPosition(KinectV2JointPosition *destination) const;

	///@brief Set posture from kinect joint orientations.
	void setKinectV2JointOrientation(KinectV2JointOrientation *kinectV2JointOrientation);

	///@brief Get orientations from kinect joint orientations.
	void getKinectV2JointOrientation(KinectV2JointOrientation *destination) const;

	///@brief Set joint quaternions for man-nii avatar.
	KinectV2JointType shortJointName2KinectV2JointType(const std::string &shortJointName) const;

	///@brief Root body position.
	SigCmn::Vector3 rootPosition;

	///@brief Whole body joint positions.
	KinectV2JointPosition jointPositions[KinectV2JointType_Count];

	///@brief Whole body joint orientations.
	KinectV2JointOrientation jointOrientations[KinectV2JointType_Count];
};

#endif //__KINECT_V2_SENSOR_DATA_H__


