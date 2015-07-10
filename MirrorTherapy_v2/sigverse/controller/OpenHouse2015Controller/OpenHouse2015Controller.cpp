/*
 * OpenHouse2015Controller.cpp
 *
 *  Created on: 2015/06/08
 *      Author: Yamada@tome
 */

#include <sigverse/common/device/SensorData.h>
#include <sigverse/controller/OpenHouse2015Controller/OpenHouse2015Controller.h>


#include <sys/time.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <math.h>

///@brief Parameter file name.
const std::string ManNiiAvatarControllerByKinectV2::parameterFileName = "OpenHouse2015.ini";


///@brief Message heard by the robot.
void OpenHouse2015Controller::onRecvMsg(RecvMsgEvent &evt)
{
	try
	{
		std::string allMsg = evt.getMsg();
//		std::cout << allMsg << std::endl;

		/*
		 * special messages
		 */
		if(std::string(evt.getSender()) == "SIGViewer" && std::string(myname()) != "man_nii_head")
		{
			sendMsg("man_nii_head", allMsg);
			return;
		}

		if(std::string(myname()) == "man_nii_head")
		{
			sendMsg("right_arm_robot_man", allMsg);
			sendMsg("right_arm_long_man",  allMsg);
			sendMsg("right_arm_short_man", allMsg);
			sendMsg("mirror_therapy_man",  allMsg);
		}


		if(allMsg=="0" || allMsg=="1" || allMsg=="2" || allMsg=="3" || allMsg=="4")
		{
			if(std::string(myname()) != "man_nii_head")
			{
				this->kinectV2DeviceManager.iniPos.z = m_iniPos_z - 1000.0 * atoi(evt.getMsg());

				SimObj *my = getObj(myname());
				my->setPosition(m_posx,m_posy,m_posz);
				this->kinectV2DeviceManager.setRootPosition(obj, this->kinectV2DeviceManager.rootPos);
			}
			return;
		}


		char *all_msg = (char*)evt.getMsg();

		char *msg = strtok(all_msg," ");

		/*
		 * Kinect sensor data
		 */
		if (strcmp(msg,"KINECT_DATA") == 0)
		{
			//自分自身の取得
			SimObj *my = getObj(myname());

			int i = 0;

			// Store current posture and time stamp.
			TimeAndPostureType current;
			current.timeStamp = this->generateCurrentTimeStamp();

			while (true)
			{
				i++;

				char *type = strtok(NULL,":");

				{
					double w = atof(strtok(NULL,","));
					double x = atof(strtok(NULL,","));
					double y = atof(strtok(NULL,","));
					double z = atof(strtok(NULL," "));

					if (this->usingOculus && (strcmp(type,"HEAD_JOINT0")==0 || strcmp(type,"HEAD_JOINT1")==0)){ continue; }

					if(std::string(myname()) != "mirror_therapy_man" ||
						(strcmp(type,"RARM_JOINT2")!=0 && strcmp(type,"RARM_JOINT3")!=0 &&
						 strcmp(type,"LARM_JOINT2")!=0 && strcmp(type,"LARM_JOINT3")!=0)
					){
						this->setJointQuaternionWithCheckingOutOfRange(my, type, this->getQuaternion(w, x, y, z));
					}
					else
					{
						if(strcmp(type,"RARM_JOINT2")==0){ current.qRARM_JOINT2= this->getQuaternion(w, x, y, z); }
						if(strcmp(type,"RARM_JOINT3")==0){ current.qRARM_JOINT3= this->getQuaternion(w, x, y, z); }
						if(strcmp(type,"LARM_JOINT2")==0){ current.qLARM_JOINT2= this->getQuaternion(w, x, y, z); }
						if(strcmp(type,"LARM_JOINT3")==0){ current.qLARM_JOINT3= this->getQuaternion(w, x, y, z); }

						//doesn't setJointQuaternion here.
					}

					continue;
				}
			}

			if(std::string(myname()) == "mirror_therapy_man")
			{
				// Decode message to sensor data of kinect v2.
				KinectV2SensorData sensorData;
				sensorData.setSensorData(sensorDataMap);

				// Convert kinect v2 quaternions(orientations) to man-nii posture(sigverse quaternion format).
				ManNiiPosture posture = KinectV2DeviceManager::convertSensorData2ManNiiPosture(sensorData);


				// Store current posture and time stamp.
				int currentIndex = this->storeCurrentPosture(posture);

				// Search nearest neighbor of target delay time in stored time stamps.
				//   last 500 postures is in this->pastPostures.
				//   find nearest index.
				ManNiiPosture delayedPosture = this->getDelayedPosture(currentIndex);

				this->invertPosture(posture, delayedPosture);


				// Set SIGVerse quaternions and positions
				SimObj *obj = getObj(myname());
				this->kinectV2DeviceManager.setRootPosition(obj, sensorData.rootPosition);
				KinectV2DeviceManager::setJointQuaternions2ManNii(obj, posture, sensorData);

				this->frameNumber++;
			}
		}
		/*
		 * Oculus sensor data or another message
		 */
		else
		{
//			// Receive message.
//			const std::string allMsg = evt.getMsg();

			// Get device type from message.
			SensorData sensorData;
			std::map<std::string, std::vector<std::string> > sensorDataMap = sensorData.decodeSensorData(allMsg);

			// Device
			if (sensorDataMap.find(MSG_KEY_DEV_TYPE) != sensorDataMap.end())
			{
				std::string deviceTypeValue = sensorDataMap[MSG_KEY_DEV_TYPE][0];
				std::string deviceUniqueId  = sensorDataMap[MSG_KEY_DEV_UNIQUE_ID][0];

				// Oculus DK1
				if (deviceTypeValue == this->oculusDK1DeviceType && deviceUniqueId ==  this->oculusDK1DeviceUniqueID)
				{
					if(!this->usingOculus)
					{
						this->usingOculus = true;
					}

					OculusRiftDK1SensorData sensorData;
					sensorData.setSensorData(sensorDataMap);

					ManNiiPosture posture = OculusDK1DeviceManager::convertEulerAngle2ManNiiPosture(sensorData.getEulerAngle());

					SimObj *obj = getObj(myname());
					OculusDK1DeviceManager::setJointQuaternions2ManNii(obj, posture);
				}
			}
			// Not a device.
			else
			{
				if(std::string(myname()) != "mirror_therapy_man")
				{
					this->setReverseModeAndDelayTime(sensorDataMap);
				}
			}
		}
	}
	catch(SimObj::NoAttributeException &err)
	{
		LOG_MSG(("NoAttributeException: %s", err.msg()));
	}
	catch(SimObj::AttributeReadOnlyException &err)
	{
		LOG_MSG(("AttributeReadOnlyException: %s", err.msg()));
	}
	catch(SimObj::Exception &err)
	{
		LOG_MSG(("Exception: %s", err.msg()));
	}
}

Quaternion OpenHouse2015Controller::getQuaternion(const double w, const double x, const double y, const double z)
{
	Quaternion quaternion;

	quaternion.w = w;
	quaternion.x = x;
	quaternion.y = y;
	quaternion.z = z;

	return quaternion;
}







extern "C" Controller * createController()
{
	return new OpenHouse2015Controller;
}

