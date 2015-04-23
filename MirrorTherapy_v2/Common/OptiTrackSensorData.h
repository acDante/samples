#pragma once
#include "SensorData.h"
//can't include NatNetTypes.h, because NatNetTypes.h is only for Windows.
//#include <NatNetTypes.h>

#include <string>


#define MSG_KEY_N_RIGID_BODIES  "N_RIGID_BODIES"
#define MSG_KEY_RIGID_STR       "RIGID"
#define MSG_KEY_RIGID(x)        MSG_KEY_RIGID_STR+x   //e.g. RIGID1, RIGID2, ... , RIGID100


//from NatNetTypes.h(version 2.7.0.0).
#define MAX_RIGIDBODIES         1000    // maximum number of RigidBodies


///@brief Posture class for using OptiTrack device.
class OptiTrackSensorData : public SensorData
{
public:
	typedef float MarkerData[3];                // posX, posY, posZ from NatNetTypes.h(version 2.7.0.0).

	// Rigid Body Data (single frame of one rigid body) from NatNetTypes.h(version 2.7.0.0).
	typedef struct sRigidBodyDataSgv
	{
		int ID;                                 // RigidBody identifier
		float x, y, z;                          // Position
		float qx, qy, qz, qw;                   // Orientation
		int nMarkers;                           // Number of markers associated with this rigid body
		MarkerData* Markers;                    // Array of marker data ( [nMarkers][3] )
		int* MarkerIDs;                         // Array of marker IDs
		float* MarkerSizes;                     // Array of marker sizes
		float MeanError;                        // Mean measure-to-solve deviation
		short params;                           // Host defined tracking flags
		sRigidBodyDataSgv()
		{
			Markers = 0; MarkerIDs = 0; MarkerSizes = 0; params=0;
		}
	} sRigidBodyDataSgv;

private:
	int nRigidBodies;                               // # of rigid bodies
	sRigidBodyDataSgv rigidBodies[MAX_RIGIDBODIES]; // Rigid body data

	///@brief convert RigidBody(ID, qw, qx, qy, qz) to string( "RIGIDx:(ID,qw,qx,qy,qz)" ).
//	std::string rigidBody2Str(const sRigidBodyDataSgv &rigidBody, const std::string &keyValueDelim, const std::string &vectorDelim) const;
	std::vector<std::string> rigidBody2Str(const sRigidBodyDataSgv &rigidBody) const;

	void addNRigidBodies(std::map<std::string, std::vector<std::string> > &messageMap) const;
	void addRigidBody(std::map<std::string, std::vector<std::string> > &messageMap, const sRigidBodyDataSgv &rigidBody) const;

public:

	OptiTrackSensorData(void);
	~OptiTrackSensorData(void);

	///@brief Convert sensor information to string.
	///@param itemsDelim Symbol to split items.
	///@param keyValueDelim Symbol to split KEY and VALUE.
	///@param valuesDelim Symbol to split each componet of VALUE. When VALUE is multidimensional data, you use this symbol.
	std::string encodeSensorData(const std::string &itemsDelim = ITEMS_DELIMITER, const std::string &keyValueDelim = KEY_AND_VALUE_DELIMITER, const std::string &valuesDelim = VALUES_DELIMITER) const;

	///@brief Set to sensor data.
	///@param Map of sensor data;
	bool setSensorData(const std::map<std::string, std::vector<std::string> > &sensorDataMap);

	///@brief set sensor data.
	void setSensorData(const int _nRigidBodies, const sRigidBodyDataSgv (&_rigidBodies)[MAX_RIGIDBODIES]);

	int getNRigidBodies(void) const;

	OptiTrackSensorData::sRigidBodyDataSgv getSRigidBodyDataSgv(const int i) const;
};
