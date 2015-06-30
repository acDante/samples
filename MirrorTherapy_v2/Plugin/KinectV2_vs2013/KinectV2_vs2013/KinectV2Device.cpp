#include "KinectV2Device.h"
#include "../../PluginCommon/CheckRecvSIGServiceData.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>


///@brief Parameter file name.
const std::string KinectV2Device::parameterFileName = "KinectV2.ini";

//Parameter file information
const std::string KinectV2Device::paramFileKeyKinectV2SensorDataMode        = "KinectV2.sensor_data_mode";
const std::string KinectV2Device::paramFileValKinectV2SensorDataModeDefault = "Quaternion";


///@brief Constructor
KinectV2Device::KinectV2Device(int argc, char **argv)
{
	if (argc == 1) 
	{
		// When it works without SIGVerse.
		fprintf(stdout, "Work stand alone.\n");
		this->sendMessageFlag = false;
		this->serverAddress = "";
		this->portNumber = -1;
	}
	else if (argc == 3) 
	{
		// Work with SIGVerse.
		fprintf(stdout, "SIGServer IP address: %s\n", argv[1]);
		fprintf(stdout, "Port number: %s\n", argv[2]);
		this->sendMessageFlag = true;
		this->serverAddress = argv[1];
		this->portNumber = atoi(argv[2]);
	}
}


KinectV2Device::~KinectV2Device()
{
}


#ifdef DEBUG_PRINT
void KinectV2Device::debugPrint()
{
	// init output file stream
	std::ofstream ofs("C:\\tmp\\plugin.log");

	std::vector<TIME_INFO>::iterator it;

	it = timeInfoList.begin();

	//printf("%d/%d/%d %d:%d:%d'%d ElbowRight.w,%f\n", systime.wYear, systime.wMonth, systime.wDay, systime.wHour, systime.wMinute, systime.wSecond, systime.wMilliseconds, tmpOrientations[KinectV2SensorData::KinectV2JointType::ElbowRight].Orientation.w);

	while( it != timeInfoList.end() ) 
	{
		ofs << (*it).systemTime.wYear << "/" 
			<< (*it).systemTime.wMonth << "/"
			<< (*it).systemTime.wDay  << " " 
			<< (*it).systemTime.wHour << ":"
			<< (*it).systemTime.wMinute << ":" 
			<< (*it).systemTime.wSecond << "'"
			<< (*it).systemTime.wMilliseconds
			<< ", value," << std::setprecision(4) << (*it).val  << std::endl;
		
		it++;
	}
}
#endif

///@brief Initialize kinect v2 device.
int KinectV2Device::run()
{
	// Refer to https://github.com/UnaNancyOwen/Kinect2Sample/blob/master/Sample/Body/Body.cpp

	cv::setUseOptimized(true);

	try 
	{
		//-- Prepare to use SIGService.
		this->readIniFile();
//		this->setSigServiceName();

		sigverse::SIGService sigService(this->serviceName);
		this->initializeSigService(sigService);

		// check receive SIGService data by another thread
		CheckRecvSIGServiceData checkRecvSIGServiceData;
		boost::thread thCheckRecvData(&CheckRecvSIGServiceData::run, &checkRecvSIGServiceData, &sigService);

		//-- Initialize kinect.

		IKinectSensor* pSensor;
		HRESULT hResult = S_OK;
		hResult = GetDefaultKinectSensor(&pSensor);
		if (FAILED(hResult))
		{
			std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
			return -1;
		}

		hResult = pSensor->Open();
		if (FAILED(hResult))
		{
			std::cerr << "Error : IKinectSensor::Open()" << std::endl;
			return -1;
		}

		// Source
		IColorFrameSource* pColorSource;
		hResult = pSensor->get_ColorFrameSource(&pColorSource);
		if (FAILED(hResult))
		{
			std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
			return -1;
		}

		IBodyFrameSource* pBodySource;
		hResult = pSensor->get_BodyFrameSource(&pBodySource);
		if (FAILED(hResult))
		{
			std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
			return -1;
		}

		// Reader
		IColorFrameReader* pColorReader;
		hResult = pColorSource->OpenReader(&pColorReader);
		if (FAILED(hResult))
		{
			std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
			return -1;
		}

		IBodyFrameReader* pBodyReader;
		hResult = pBodySource->OpenReader(&pBodyReader);
		if (FAILED(hResult))
		{
			std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
			return -1;
		}

		// Description
		IFrameDescription* pDescription;
		hResult = pColorSource->get_FrameDescription(&pDescription);
		if (FAILED(hResult))
		{
			std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
			return -1;
		}

		int width = 0;
		int height = 0;
		pDescription->get_Width(&width); // 1920
		pDescription->get_Height(&height); // 1080
		pDescription->get_Width(&this->colorFrameWidth); // 1920
		pDescription->get_Height(&this->colorFrameHeight); // 1080

		unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);

		cv::Mat bufferMat(height, width, CV_8UC4);
		cv::Mat bodyMat(height / 2, width / 2, CV_8UC4);
		cv::namedWindow("Body");

		// Color Table
		cv::Vec3b color[BODY_COUNT];
		color[0] = cv::Vec3b(255, 0, 0);
		color[1] = cv::Vec3b(0, 255, 0);
		color[2] = cv::Vec3b(0, 0, 255);
		color[3] = cv::Vec3b(255, 255, 0);
		color[4] = cv::Vec3b(255, 0, 255);
		color[5] = cv::Vec3b(0, 255, 255);

		// Coordinate Mapper
		ICoordinateMapper* pCoordinateMapper;
		hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
		if (FAILED(hResult))
		{
			std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
			return -1;
		}

		//-- Start measuring.

		while (1)
		{
			// Frame
			IColorFrame* pColorFrame = nullptr;
			hResult = pColorReader->AcquireLatestFrame(&pColorFrame);

			if (SUCCEEDED(hResult))
			{
				hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
				if (SUCCEEDED(hResult))
				{
					cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
				}
			}
			//SafeRelease( pColorFrame );

			IBodyFrame* pBodyFrame = nullptr;
			hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);

			if (SUCCEEDED(hResult))
			{
				IBody* pBody[BODY_COUNT] = { 0 };
				hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);

				if (SUCCEEDED(hResult))
				{
					for (int count = 0; count < BODY_COUNT; count++)
					{
						BOOLEAN bTracked = false;
						hResult = pBody[count]->get_IsTracked(&bTracked);

						if (SUCCEEDED(hResult) && bTracked)
						{
							Joint joint[KinectV2SensorData::KinectV2JointType_Count];
							hResult = pBody[count]->GetJoints(KinectV2SensorData::KinectV2JointType_Count, joint);

							if (SUCCEEDED(hResult))
							{
								// Left Hand State
								HandState leftHandState = HandState::HandState_Unknown;
								hResult = pBody[count]->get_HandLeftState(&leftHandState);

								if (SUCCEEDED(hResult))
								{
									this->handStateProcessing(JointType::JointType_HandLeft, leftHandState, pCoordinateMapper, joint, bufferMat);
								}

								// Right Hand State
								HandState rightHandState = HandState::HandState_Unknown;
								hResult = pBody[count]->get_HandRightState(&rightHandState);

								if (SUCCEEDED(hResult))
								{
									this->handStateProcessing(JointType::JointType_HandRight, rightHandState, pCoordinateMapper, joint, bufferMat);
								}

								// Joint
								bool detectBodyFlag = false;

								for (int type = 0; type < JointType::JointType_Count; type++)
								{
									ColorSpacePoint colorSpacePoint = { 0 };
									pCoordinateMapper->MapCameraPointToColorSpace(joint[type].Position, &colorSpacePoint);
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);

									if ((x >= 0) && (x < width) && (y >= 0) && (y < height))
									{
										cv::circle(bufferMat, cv::Point(x, y), 5, static_cast<cv::Scalar>(color[count]), -1, CV_AA);

										detectBodyFlag = true;
									}
								}

								if (detectBodyFlag) 
								{
									// Get spine base position.
									SensorData::Vector3 spPos = { 0 };
									spPos.x = joint[JointType::JointType_SpineBase].Position.X;
									spPos.y = joint[JointType::JointType_SpineBase].Position.Y;
									spPos.z = joint[JointType::JointType_SpineBase].Position.Z;

									// Set spine base position.
									((KinectV2SensorData*)this->sensorData)->setRootPosition(spPos);

									switch (((KinectV2SensorData *)this->sensorData)->sensorDataMode)
									{
										case KinectV2SensorData::Position:
										{
											// Get Joint Position.
											Joint tmpPositions[_JointType::JointType_Count];
											hResult = pBody[count]->GetJoints(_JointType::JointType_Count, tmpPositions);

											KinectV2SensorData::KinectV2JointPosition tmpKinectV2JointPositions[KinectV2SensorData::KinectV2JointType_Count];
											this->convertJointPositions2KinectV2JointPositions(tmpPositions, tmpKinectV2JointPositions);

											((KinectV2SensorData*)this->sensorData)->setKinectV2JointPosition(tmpKinectV2JointPositions);
#ifdef DEBUG_PRINT
											if (SUCCEEDED(hResult))
											{
												SYSTEMTIME systime;
												GetLocalTime(&systime);

												TIME_INFO tmpTimeInfo;
												tmpTimeInfo.systemTime = systime;
												//test print elbow right
												tmpTimeInfo.val = tmpPositions[KinectV2SensorData::KinectV2JointType::ElbowRight].Position.X;
												timeInfoList.push_back(tmpTimeInfo);
											}
#endif
											break;
										}
										case KinectV2SensorData::Quaternion:
										{
											// Get Joint Orientation.
											JointOrientation tmpOrientations[_JointType::JointType_Count];
											hResult = pBody[count]->GetJointOrientations(_JointType::JointType_Count, tmpOrientations);

											KinectV2SensorData::KinectV2JointOrientation tmpKinectV2JointOrientations[KinectV2SensorData::KinectV2JointType_Count];
											this->convertJointOrientations2KinectV2JointOrientations(tmpOrientations, tmpKinectV2JointOrientations);

											((KinectV2SensorData*)this->sensorData)->setKinectV2JointOrientation(tmpKinectV2JointOrientations);
#ifdef DEBUG_PRINT
											if (SUCCEEDED(hResult))
											{
												SYSTEMTIME systime;
												GetLocalTime(&systime);

												TIME_INFO tmpTimeInfo;
												tmpTimeInfo.systemTime = systime;
												//test print elbow right
												tmpTimeInfo.val = tmpOrientations[KinectV2SensorData::KinectV2JointType::ElbowRight].Orientation.w;
												timeInfoList.push_back(tmpTimeInfo);
											}
#endif
											break;
										}

										if (SUCCEEDED(hResult) && this->sendMessageFlag) 
										{
											// Send message to SigServer.
											const std::string sensorDataMessage = this->sensorData->encodeSensorData();
											const std::string messageHeader = this->generateMessageHeader();
											const std::string message = messageHeader + sensorDataMessage;
											this->sendMessage(sigService, message);
											// std::cout << "SEND: " << message << std::endl;
										}
									}
								}
							}

							// Lean
							//PointF amount;
							//hResult = pBody[count]->get_Lean(&amount);
							//if (SUCCEEDED(hResult)){
							//	std::cout << "amount : " << amount.X << ", " << amount.Y << std::endl;
							//}
						}
					}
					cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
				}
				for (int count = 0; count < BODY_COUNT; count++){
					SafeRelease(pBody[count]);
				}
			}
			//SafeRelease( pBodyFrame );

			SafeRelease(pColorFrame);
			SafeRelease(pBodyFrame);

			cv::imshow("Body", bodyMat);

			if (cv::waitKey(10) == VK_ESCAPE)
			{
				break;
			}
		}

		SafeRelease(pColorSource);
		SafeRelease(pBodySource);
		SafeRelease(pColorReader);
		SafeRelease(pBodyReader);
		SafeRelease(pDescription);
		SafeRelease(pCoordinateMapper);

		if (pSensor)
		{
			pSensor->Close();
		}
		SafeRelease(pSensor);
		cv::destroyAllWindows();
	}
	catch (std::exception &ex) 
	{
		std::cout << "Initialize sigservice ERR :" << ex.what() << std::endl;
	}

#ifdef DEBUG_PRINT
	debugPrint();
#endif

	return 0;
}


void KinectV2Device::handStateProcessing(const JointType &hand, const HandState &handState, ICoordinateMapper* &coordinateMapper, Joint* joint, cv::Mat &image)
{
	// Left Hand State
	ColorSpacePoint colorSpacePoint = { 0 };
	HRESULT hResult = coordinateMapper->MapCameraPointToColorSpace(joint[hand].Position, &colorSpacePoint);

	if (SUCCEEDED(hResult))
	{
		int x = static_cast<int>(colorSpacePoint.X);
		int y = static_cast<int>(colorSpacePoint.Y);

		if ((x >= 0) && (x < this->colorFrameWidth) && (y >= 0) && (y < this->colorFrameHeight))
		{
			if (handState == HandState::HandState_Open)
			{
				cv::circle(image, cv::Point(x, y), 75, cv::Scalar(0, 128, 0), 5, CV_AA);
			}
			else if (handState == HandState::HandState_Closed)
			{
				cv::circle(image, cv::Point(x, y), 75, cv::Scalar(0, 0, 128), 5, CV_AA);
			}
			else if (handState == HandState::HandState_Lasso)
			{
				cv::circle(image, cv::Point(x, y), 75, cv::Scalar(128, 128, 0), 5, CV_AA);
			}
		}
	}
}


///@brief Set whole body joint positions.
void KinectV2Device::convertJointPositions2KinectV2JointPositions(Joint *joints, KinectV2SensorData::KinectV2JointPosition *kinectV2JointPositions)
{
	for (int i = 0; i < KinectV2SensorData::KinectV2JointType_Count; i++) 
	{
		kinectV2JointPositions[i].jointType     = getKinectV2JointType(joints[i].JointType);
		kinectV2JointPositions[i].position.x    = joints[i].Position.X;
		kinectV2JointPositions[i].position.y    = joints[i].Position.Y;
		kinectV2JointPositions[i].position.z    = joints[i].Position.Z;
		kinectV2JointPositions[i].trackingState = (KinectV2SensorData::KinectV2TrackingState)(joints[i].TrackingState);
	}
}

///@brief Set whole body joint orientations.
void KinectV2Device::convertJointOrientations2KinectV2JointOrientations(JointOrientation *orientations, KinectV2SensorData::KinectV2JointOrientation *kinectV2JointOrientations) 
{
	for (int i = 0; i < KinectV2SensorData::KinectV2JointType_Count; i++) 
	{
		kinectV2JointOrientations[i].jointType     = getKinectV2JointType(orientations[i].JointType);
		kinectV2JointOrientations[i].orientation.w = orientations[i].Orientation.w;
		kinectV2JointOrientations[i].orientation.x = orientations[i].Orientation.x;
		kinectV2JointOrientations[i].orientation.y = orientations[i].Orientation.y;
		kinectV2JointOrientations[i].orientation.z = orientations[i].Orientation.z;
	}
}

///@brief Convert JointType to KinectV2JointType.
KinectV2SensorData::KinectV2JointType KinectV2Device::getKinectV2JointType(const JointType jointType) 
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

///@brief Read parameter file.
void KinectV2Device::readIniFile()
{
	std::ifstream ifs(this->parameterFileName.c_str());

	std::string sensorDataModeStr = paramFileValKinectV2SensorDataModeDefault;

	// Parameter file is "not" exists.
	if (ifs.fail()) 
	{
		std::cout << "Not exist : " << this->parameterFileName << std::endl;
		std::cout << "Use default parameter." << std::endl;

		this->serviceName    = SERVICE_NAME_KINECT_V2;
		this->deviceType     = DEV_TYPE_KINECT_V2;
		this->deviceUniqueID = DEV_UNIQUE_ID_0;
	}
	// Parameter file is exists.
	else 
	{
		try 
		{
			std::cout << "Read " << this->parameterFileName << std::endl;
			boost::property_tree::ptree pt;
			boost::property_tree::read_ini(this->parameterFileName, pt);


			this->serviceName    = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME);
			this->deviceType     = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE);
			this->deviceUniqueID = pt.get<std::string>(PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID);

			sensorDataModeStr    = pt.get<std::string>(paramFileKeyKinectV2SensorDataMode);
		}
		catch (std::string &ex)
		{
			std::cout << ex << std::endl;
			exit(EXIT_FAILURE);
		}
		catch (boost::exception &ex) 
		{
			std::cout << this->parameterFileName << " ERR :" << *boost::diagnostic_information_what(ex) << std::endl;
			exit(EXIT_FAILURE);
		}
	}

	std::cout << PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME     << ":" << this->serviceName    << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE      << ":" << this->deviceType     << std::endl;
	std::cout << PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID << ":" << this->deviceUniqueID << std::endl;
	std::cout << paramFileKeyKinectV2SensorDataMode          << ":" << ((KinectV2SensorData *)this->sensorData)->getSensorDataModeStr() << std::endl;

	
	KinectV2SensorData kinectV2SensorData(sensorDataModeStr);
	this->sensorData = &kinectV2SensorData;
}

