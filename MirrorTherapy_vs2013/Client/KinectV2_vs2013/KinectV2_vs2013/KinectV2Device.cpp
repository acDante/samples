#include "KinectV2Device.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

//KinectV2Device::KinectV2Device(int argc, char **argv)
//{
//	KinectV2Device::Device(argc, argv);
//}

KinectV2Device::~KinectV2Device()
{
}


///@brief Constructor
KinectV2Device::KinectV2Device(int argc, char **argv)
{
	if (argc == 1) {
		// When it works without SIGVerse.
		fprintf(stdout, "Work stand alone.\n");
		this->sendMessageFlag = false;
		this->serverAddress = "";
		this->portNumber = -1;
	}
	else if (argc == 3) {
		// Work with SIGVerse.
		fprintf(stdout, "SIGServer IP address: %s\n", argv[1]);
		fprintf(stdout, "Port number: %s\n", argv[2]);
		this->sendMessageFlag = true;
		this->serverAddress = argv[1];
		this->portNumber = atoi(argv[2]);
	}
	this->parameterFileName = defaultParameterFileName;
	//this->kinectConnector = KinectV2Connector();
	//this->readIniFile();

}

void KinectV2Device::setSigServiceName()
{
	// パラメータファイルを読み込んでサービスネームを設定します．
	// パラメータファイルが見つからなかった場合，ヘッダファイルに記述したサービスネームを設定します．
	if (!this->Device::readIniFile()) {
		this->serviceName = defaultServiceName;
		std::cout << "Set default service name : " << this->serviceName << std::endl;
	}
}

///@brief Initialize kinect v2 device.
int KinectV2Device::run()
{
	// Refer to https://github.com/UnaNancyOwen/Kinect2Sample/blob/master/Sample/Body/Body.cpp

	cv::setUseOptimized(true);

	try {

		//-- Prepare to use SIGService.
		this->setSigServiceName();
		sigverse::SIGService sigService(this->serviceName);
		this->initializeSigService(sigService);

		//-- Initialize kinect.

		IKinectSensor* pSensor;
		HRESULT hResult = S_OK;
		hResult = GetDefaultKinectSensor(&pSensor);
		if (FAILED(hResult)){
			std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
			return -1;
		}

		hResult = pSensor->Open();
		if (FAILED(hResult)){
			std::cerr << "Error : IKinectSensor::Open()" << std::endl;
			return -1;
		}

		// Source
		IColorFrameSource* pColorSource;
		hResult = pSensor->get_ColorFrameSource(&pColorSource);
		if (FAILED(hResult)){
			std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
			return -1;
		}

		IBodyFrameSource* pBodySource;
		hResult = pSensor->get_BodyFrameSource(&pBodySource);
		if (FAILED(hResult)){
			std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
			return -1;
		}

		// Reader
		IColorFrameReader* pColorReader;
		hResult = pColorSource->OpenReader(&pColorReader);
		if (FAILED(hResult)){
			std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
			return -1;
		}

		IBodyFrameReader* pBodyReader;
		hResult = pBodySource->OpenReader(&pBodyReader);
		if (FAILED(hResult)){
			std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
			return -1;
		}

		// Description
		IFrameDescription* pDescription;
		hResult = pColorSource->get_FrameDescription(&pDescription);
		if (FAILED(hResult)){
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
		if (FAILED(hResult)){
			std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
			return -1;
		}

		//-- Start measuring.

		while (1){
		
			// Frame
			IColorFrame* pColorFrame = nullptr;
			hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
			if (SUCCEEDED(hResult)){
				hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
				if (SUCCEEDED(hResult)){
					cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
				}
			}
			//SafeRelease( pColorFrame );

			IBodyFrame* pBodyFrame = nullptr;
			hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
			if (SUCCEEDED(hResult)){
				IBody* pBody[BODY_COUNT] = { 0 };
				hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
				if (SUCCEEDED(hResult)){
					for (int count = 0; count < BODY_COUNT; count++){
						BOOLEAN bTracked = false;
						hResult = pBody[count]->get_IsTracked(&bTracked);
						if (SUCCEEDED(hResult) && bTracked){
							Joint joint[KinectV2JointType_Count];
							hResult = pBody[count]->GetJoints(KinectV2JointType_Count, joint);
							if (SUCCEEDED(hResult)){
								// Left Hand State
								HandState leftHandState = HandState::HandState_Unknown;
								hResult = pBody[count]->get_HandLeftState(&leftHandState);
								if (SUCCEEDED(hResult)){
									this->handStateProcessing(JointType::JointType_HandLeft, leftHandState, pCoordinateMapper, joint, bufferMat);
								}

								// Right Hand State
								HandState rightHandState = HandState::HandState_Unknown;
								hResult = pBody[count]->get_HandRightState(&rightHandState);
								if (SUCCEEDED(hResult)){
									this->handStateProcessing(JointType::JointType_HandRight, rightHandState, pCoordinateMapper, joint, bufferMat);
								}

								// Joint
								bool detectBodyFlag = false;
								for (int type = 0; type < JointType::JointType_Count; type++){
									ColorSpacePoint colorSpacePoint = { 0 };
									pCoordinateMapper->MapCameraPointToColorSpace(joint[type].Position, &colorSpacePoint);
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);

									if ((x >= 0) && (x < width) && (y >= 0) && (y < height)){
										// 検出したJointの位置が画面内に収まるときに，Bodyを検出したとする
										cv::circle(bufferMat, cv::Point(x, y), 5, static_cast<cv::Scalar>(color[count]), -1, CV_AA);
										
										// この時，このフレームのこのBodyで人体を検出したフラグをたてる
										detectBodyFlag = true;
									}
								}

								// この if のブロックが，主に野崎が編集したところです．
								if (detectBodyFlag) {
									// Get spine base position.
									Vector3 spPos = { 0 };
									spPos.x = joint[JointType::JointType_SpineBase].Position.X;
									spPos.y = joint[JointType::JointType_SpineBase].Position.Y;
									spPos.z = joint[JointType::JointType_SpineBase].Position.Z;
									
									// Set spine base position.
									KinectV2SensorData kinectV2SensorData;
									kinectV2SensorData.setRootPosition(spPos);

									// Get Joint Orientation.
									JointOrientation tmpOrientations[KinectV2JointType_Count];
									hResult = pBody[count]->GetJointOrientations(KinectV2JointType_Count, tmpOrientations);
									
									// Set Joint Orientation.
									kinectV2SensorData.setJointOrientations(tmpOrientations);
									
									// Set sensor data to member variable.
									this->sensorData = &kinectV2SensorData;
									
									if (SUCCEEDED(hResult) && this->sendMessageFlag) {
										// Send message to SigServer.
										const std::string sensorDataMessage = this->sensorData->convertSensorData2Message(keyDelimDefault, recordDelimDefault, valueDelimDefault);
										const std::string messageHeader = "DEV_TYPE:KINECTV2;DEV_ID:0;";
										const std::string message = messageHeader + sensorDataMessage;
										this->sendMessage(sigService, message);
										//std::cout << "SEND: " << message << std::endl;
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

			if (cv::waitKey(10) == VK_ESCAPE){
				break;
			}
		}

		SafeRelease(pColorSource);
		SafeRelease(pBodySource);
		SafeRelease(pColorReader);
		SafeRelease(pBodyReader);
		SafeRelease(pDescription);
		SafeRelease(pCoordinateMapper);
		if (pSensor){
			pSensor->Close();
		}
		SafeRelease(pSensor);
		cv::destroyAllWindows();
	}
	catch (std::exception &ex) {
		std::cout << "Initialize sigservice ERR :" << ex.what() << std::endl;
	}

	return 0;
}


///@brief Left hand state processing
//HRESULT KinectV2Device::leftHandStateProcessing(const HandState &leftHandState, ICoordinateMapper* &coordinateMapper, const Joint* &joint, cv::Mat &image)
//{
//
//}

void KinectV2Device::handStateProcessing(const JointType &hand, const HandState &handState, ICoordinateMapper* &coordinateMapper, Joint* joint, cv::Mat &image)
{
	// Left Hand State
	ColorSpacePoint colorSpacePoint = { 0 };
	HRESULT hResult = coordinateMapper->MapCameraPointToColorSpace(joint[hand].Position, &colorSpacePoint);
	if (SUCCEEDED(hResult)){
		int x = static_cast<int>(colorSpacePoint.X);
		int y = static_cast<int>(colorSpacePoint.Y);
		if ((x >= 0) && (x < this->colorFrameWidth) && (y >= 0) && (y < this->colorFrameHeight)){
			if (handState == HandState::HandState_Open){
				cv::circle(image, cv::Point(x, y), 75, cv::Scalar(0, 128, 0), 5, CV_AA);
			}
			else if (handState == HandState::HandState_Closed){
				cv::circle(image, cv::Point(x, y), 75, cv::Scalar(0, 0, 128), 5, CV_AA);
			}
			else if (handState == HandState::HandState_Lasso){
				cv::circle(image, cv::Point(x, y), 75, cv::Scalar(128, 128, 0), 5, CV_AA);
			}
		}
	}
}