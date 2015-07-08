#include <sigverse/common/device/OptiTrackSensorData.h>

#include <sstream>


OptiTrackSensorData::OptiTrackSensorData(void)
{
	this->nRigidBodies = 0;
}


OptiTrackSensorData::~OptiTrackSensorData(void)
{
}


void OptiTrackSensorData::addNRigidBodies(std::map<std::string, std::vector<std::string> > &messageMap) const
{
	std::vector<std::string> vec;

	std::stringstream ss;
	ss << this->nRigidBodies;

	vec.push_back(ss.str());

	messageMap.insert(make_pair(MSG_KEY_N_RIGID_BODIES, vec) );
}


///@brief RigidBody(ID, qw, qx, qy, qz) to string( "RIGIDx:(ID,qw,qx,qy,qz)" ).
void OptiTrackSensorData::addRigidBody(std::map<std::string, std::vector<std::string> > &messageMap, const sRigidBodyDataSgv &rigidBody) const
{
	std::stringstream ssKey;

	//e.g. RIGID1, RIGID2, ... , RIGID100
//	ssKey << MSG_KEY_RIGID(NUM_TO_STR(rigidBody.ID));
	ssKey << MSG_KEY_RIGID_STR << rigidBody.ID;

//	std::cout << "ssKey:" << ssKey.str() << std::endl;

	std::stringstream ssValue;

	std::vector<std::string> vec;

	ssValue << rigidBody.ID;     vec.push_back(ssValue.str()); ssValue.clear(); ssValue.str("");
	ssValue << rigidBody.qw;     vec.push_back(ssValue.str()); ssValue.clear(); ssValue.str("");
	ssValue << rigidBody.qx;     vec.push_back(ssValue.str()); ssValue.clear(); ssValue.str("");
	ssValue << rigidBody.qy;     vec.push_back(ssValue.str()); ssValue.clear(); ssValue.str("");
	ssValue << rigidBody.qz;     vec.push_back(ssValue.str()); ssValue.clear(); ssValue.str("");
	ssValue << rigidBody.params; vec.push_back(ssValue.str());

	messageMap.insert(make_pair(ssKey.str(), vec));
};


///@brief generate message by posture.
std::string OptiTrackSensorData::encodeSensorData(const std::string &itemsDelim, const std::string &keyValueDelim, const std::string &valuesDelim) const
{
	if(this->nRigidBodies == 0){ return ""; }

	std::map<std::string, std::vector<std::string> > messageMap;

	//number of rigid bodies info.
	this->addNRigidBodies(messageMap);

	//rigid bodies info.
	for(int i=0; i<this->nRigidBodies; i++)
	{
		this->addRigidBody(messageMap, this->rigidBodies[i]);
	}
	
	return this->convertMap2Message(messageMap, itemsDelim, keyValueDelim, valuesDelim);
}


bool OptiTrackSensorData::setSensorData(const std::map<std::string, std::vector<std::string> > &sensorDataMap)
{
	std::map<std::string, std::vector<std::string> >::const_iterator it;

	int cntRigidBodies = 0;

	for (it = sensorDataMap.begin(); it != sensorDataMap.end(); it++)
	{
		if((*it).first==MSG_KEY_N_RIGID_BODIES)
		{
			this->nRigidBodies = atoi((*it).second[0].c_str());
			continue;
		}
		//RIGIDx
		if((*it).first.substr(0, 5)==MSG_KEY_RIGID_STR)
		{
			if((*it).second.size()!=6)
			{
				std::cout << "Number of OptiTrack sensor data is not 6." << std::endl;
				return false;
			}

			sRigidBodyDataSgv tmpData;
			tmpData.ID     =        atoi((*it).second[0].c_str());
			tmpData.qw     = (float)atof((*it).second[1].c_str());
			tmpData.qx     = (float)atof((*it).second[2].c_str());
			tmpData.qy     = (float)atof((*it).second[3].c_str());
			tmpData.qz     = (float)atof((*it).second[4].c_str());
			tmpData.params = (short)atoi((*it).second[5].c_str());

	//			std::cout << "dec3:" << this->nRigidBodies << ",ID=" << tmpData.ID << "," << tmpData.qw << "," << tmpData.qx << "," << tmpData.qy << "," << tmpData.qz << ", "<< tmpData.params << std::endl;

			if(tmpData.ID >= MAX_RIGIDBODIES){ continue; }

			this->rigidBodies[cntRigidBodies] = tmpData;

			cntRigidBodies++;
			continue;
		}
	}

	return true;
}


///@brief set sensor data.
void OptiTrackSensorData::setSensorData(const int _nRigidBodies, const sRigidBodyDataSgv (&_rigidBodies)[MAX_RIGIDBODIES])
{
	this->nRigidBodies = _nRigidBodies;
	
	memcpy(this->rigidBodies, _rigidBodies, sizeof(_rigidBodies));
}


int OptiTrackSensorData::getNRigidBodies(void) const
{
	return this->nRigidBodies;
}


OptiTrackSensorData::sRigidBodyDataSgv OptiTrackSensorData::getSRigidBodyDataSgv(const int i) const
{
	return this->rigidBodies[i];
}
