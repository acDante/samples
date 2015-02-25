#include "KinectDevice.h"
#include <ComPtr.h>
#include <d2d1.h>
#include <boost/thread.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>

class BoostException : public boost::exception, public std::exception {};

///@brief Constructor.
KinectDevice::KinectDevice(int argc, char **argv)
{
	if (argc == 1)
	{
		// When it works without SIGViewer.
		fprintf(stdout, "Work stand alone.\n");
		this->sendMessageFlag = false;
		this->serverAddress = "";
		this->portNumber = -1;
	}
	else if (argc == 3)
	{
		// Work with SIGViewer.
		fprintf(stdout, "SIGServer IP address: %s\n", argv[1]);
		fprintf(stdout, "Port number: %s\n", argv[2]);
		this->sendMessageFlag = true;
		this->serverAddress = argv[1];
		this->portNumber = atoi(argv[2]);
	}
	this->finishMeasuringFlag = false;

	// Read and set parameters.
	this->readIniFile();
}

KinectDevice::~KinectDevice()
{
}

///@brief Read the parameter file.
void KinectDevice::readIniFile()
{
	try
	{
		boost::property_tree::ptree pt;
		boost::property_tree::read_ini(this->PARAM_FILE_NAME, pt);
		this->movingSpeed = pt.get<double>("General.move_speed");
		std::cout << "move_speed:" << this->movingSpeed << std::endl;
		this->serviceName = pt.get<std::string>("General.service_name");
		std::cout << "service_name:" << this->serviceName << std::endl;
	}
	catch (boost::exception &ex)
	{
		std::cout << this->PARAM_FILE_NAME << " ERR :" << *boost::diagnostic_information_what(ex) << std::endl;
	}
}


///@brief Initialize kinect device.
void KinectDevice::initializeKinect()
{
	// Get kinect sensor.
	ERROR_CHECK(::GetDefaultKinectSensor(&(this->kinectConnector.kinect)));

	// Open kinect.
	ERROR_CHECK(this->kinectConnector.kinect->Open());

	BOOLEAN isOpen = false;

	ERROR_CHECK(this->kinectConnector.kinect->get_IsOpen(&isOpen));
	if (!isOpen)
	{
		throw std::runtime_error("Can't open Kinect Device.");
	}

	// Get body reader for body information.
	ComPtr<IBodyFrameSource> bodyFrameSource;
	ERROR_CHECK(this->kinectConnector.kinect->get_BodyFrameSource(&bodyFrameSource));
	ERROR_CHECK(bodyFrameSource->OpenReader(&this->kinectConnector.bodyFrameReader));

	// Get color reader for scene image.
	ComPtr<IColorFrameSource> colorFrameSource;
	ERROR_CHECK(this->kinectConnector.kinect->get_ColorFrameSource(&colorFrameSource));
	ERROR_CHECK(colorFrameSource->OpenReader(&this->kinectConnector.colorFrameReader));

	// Get image size.
	ComPtr<IFrameDescription> colorFrameDescription;
	ERROR_CHECK(colorFrameSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription));
	ERROR_CHECK(colorFrameDescription->get_Width(&this->kinectConnector.colorFrameWidth));
	ERROR_CHECK(colorFrameDescription->get_Height(&this->kinectConnector.colorFrameHeight));
	ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&this->kinectConnector.colorBytesPerPixel));

	// Generate color image buffer.
	this->kinectConnector.colorBuffer.resize(this->kinectConnector.colorFrameWidth * this->kinectConnector.colorFrameHeight * this->kinectConnector.colorBytesPerPixel);

	//this->startMeasureFlag = false;
	//this->finishMeasureFlag = false;
}




///@brief Update body frame.
void KinectDevice::updateBodyFrame()
{
	// Get frame.
	ComPtr<IBodyFrame> bodyFrame;
	auto ret = this->kinectConnector.bodyFrameReader->AcquireLatestFrame(&bodyFrame);

	if (ret == S_OK)
	{
		// Recognize only one person.
		ERROR_CHECK(bodyFrame->GetAndRefreshBodyData(6, &this->kinectConnector.bodies[0]));
	}
}

///@brief Draw body.
void KinectDevice::draw()
{
	try
	{
		this->drawScene();
		this->drawBodyIndexFrame();
	}
	catch (std::exception& ex)
	{
		std::cout << "draw ERR :" << ex.what() << std::endl;
	}
}

///@brief Draw body index.
void KinectDevice::drawBodyIndexFrame()
{
	try
	{
		// Display joints.
		this->kinectConnector.bodyImage = cv::Mat::zeros(cv::Size(this->kinectConnector.depthImageWidth, this->kinectConnector.depthImageHeight), CV_8UC4);

		for (auto body : this->kinectConnector.bodies)
		{
			// No body detected, do nothing.
			if (body == nullptr) { continue; }

			// Check tracking the body successful or not.
			BOOLEAN isTracked = false;
			ERROR_CHECK(body->get_IsTracked(&isTracked));
			if (!isTracked) { continue; }

			// Get joint positions.
			//Joint joints[JointType::JointType_Count];
			body->GetJoints(JointType::JointType_Count, this->posture.positions);

			// Display lines between joints.
			this->drawLines(this->kinectConnector.bodyImage, this->posture.positions);

			// Display ellipses at all joints.
			for (auto joint : this->posture.positions)
			{
				if (joint.TrackingState == TrackingState::TrackingState_Tracked)
				{
					// Tracking state.
					drawEllipse(this->kinectConnector.bodyImage, joint, this->kinectConnector.ellipseRadius, this->kinectConnector.BLUE);
				}
				else if (joint.TrackingState == TrackingState::TrackingState_Inferred)
				{
					// Estimating state.
					drawEllipse(this->kinectConnector.bodyImage, joint, this->kinectConnector.ellipseRadius, this->kinectConnector.CYAN);
				}
			}
		}
		cv::imshow("Body", this->kinectConnector.bodyImage);
	}
	catch (std::exception &ex)
	{
		std::cout << "drawBodyIndexFrame ERR :" << ex.what() << std::endl;
	}
}

///@brief Draw scene(color image from camera).
void KinectDevice::drawScene()
{
	try
	{
		// Get color frame.
		ComPtr<IColorFrame> colorFrame;
		int ret = this->kinectConnector.colorFrameReader->AcquireLatestFrame(&colorFrame); // auto -> int
		if (ret == S_OK)
		{
			// Get color image as BGRA format.
			ERROR_CHECK(
				colorFrame->CopyConvertedFrameDataToArray(
				this->kinectConnector.colorBuffer.size(),
				&this->kinectConnector.colorBuffer[0],
				ColorImageFormat::ColorImageFormat_Bgra));

			// Display color image.
			cv::Mat colorImage(this->kinectConnector.colorFrameHeight, this->kinectConnector.colorFrameWidth, CV_8UC4, &this->kinectConnector.colorBuffer[0]);
			//cv::resize(colorImage, kinectConnector.sceneImage, cv::Size(drawVar.imgCol, drawVar.imgRow), cv::INTER_LINEAR);
			cv::resize(colorImage, this->kinectConnector.sceneImage, cv::Size(), 0.25, 0.25, cv::INTER_LINEAR); // Resize to smaller size.
			cv::imshow("Scene", this->kinectConnector.sceneImage);
		}
	}
	catch (std::exception &ex)
	{
		std::cout << "drawScene ERR :" << ex.what() << std::endl;
	}
}

///@brief Get body position to color scene.
void KinectDevice::getPositionToColorFrame(const CameraSpacePoint &bodyPoint, cv::Point &point, const int width, const int height)
{
	ComPtr<ICoordinateMapper> mapper;
	ERROR_CHECK(this->kinectConnector.kinect->get_CoordinateMapper(&mapper));
	DepthSpacePoint depthPoint = { 0 };
	mapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);
	point.x = (int)((depthPoint.X * width) / this->kinectConnector.depthImageWidth);
	point.y = (int)((depthPoint.Y * height) / this->kinectConnector.depthImageHeight);
}

///@brief Draw ellipses at each joint.
void KinectDevice::drawEllipse(cv::Mat &bodyImage, const Joint &joint, const int r, const cv::Scalar &color)
{
	// Convert camera space coordinate to depth space coordinate. 
	ComPtr<ICoordinateMapper> mapper;
	ERROR_CHECK(this->kinectConnector.kinect->get_CoordinateMapper(&mapper));

	DepthSpacePoint point;
	mapper->MapCameraPointToDepthSpace(joint.Position, &point);

	cv::circle(bodyImage, cv::Point((int)point.X, (int)point.Y), r, color, -1);
}

///@brief Draw connecting all joints lines.
void KinectDevice::drawLines(cv::Mat &bodyImage, const Joint *joints)
{
	this->drawLine(this->kinectConnector.bodyImage, joints[3], joints[2], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[2], joints[20], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[20], joints[8], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[20], joints[4], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[20], joints[1], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[8], joints[9], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[4], joints[5], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[9], joints[10], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[5], joints[6], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[10], joints[11], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[6], joints[7], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[11], joints[23], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[7], joints[21], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[10], joints[24], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[6], joints[22], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[1], joints[0], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[0], joints[16], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[0], joints[12], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[16], joints[17], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[12], joints[13], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[17], joints[18], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[13], joints[14], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[18], joints[19], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
	this->drawLine(this->kinectConnector.bodyImage, joints[14], joints[15], this->kinectConnector.YELLOW, this->kinectConnector.lineThickness);
}


///@brief Draw connecting line at each joint.
void KinectDevice::drawLine(cv::Mat &bodyImage, const Joint &joint1, const Joint &joint2, const cv::Scalar &lineColor, const int thickness)
{
	ComPtr<ICoordinateMapper> mapper;
	ERROR_CHECK(this->kinectConnector.kinect->get_CoordinateMapper(&mapper));

	DepthSpacePoint point1;
	mapper->MapCameraPointToDepthSpace(joint1.Position, &point1);

	DepthSpacePoint point2;
	mapper->MapCameraPointToDepthSpace(joint2.Position, &point2);
	
	cv::line(bodyImage, cv::Point((int)point1.X, (int)point1.Y), cv::Point((int)point2.X, (int)point2.Y), lineColor, thickness);
}


///@brief Run draw methods.
void KinectDevice::runDrawingMethods()
{
	// Initialize kinect device.
	this->initializeKinect();

	//std::cout << "DrawBody::run -- START --" << std::endl;

	while (true)
	{
		// Update body frame.
		this->updateBodyFrame();

		// Draw the image.
		this->draw();

		// Update the image.
		int key = cv::waitKey(1);

		// Push 'q', stop drawing.
		if (key == 'q')
		{
			this->finishMeasuringFlag = true;
			break;
		}
	}

	//std::cout << "DrawBody::run -- END --" << std::endl;

}

///@brief Run send methods.
void KinectDevice::runSendingMethods()
{
	//const std::string sname = "SIGKINECT";
	sigverse::SIGService sigservice(this->serviceName);

	if (this->sendMessageFlag)
	{
		bool connectServerFlag = false; bool connectViewerFlag = false;

		this->connectToServerAndViewer(sigservice, connectServerFlag, connectViewerFlag);

		if (!connectServerFlag || !connectViewerFlag)
		{
			return;
		}
		else
		{
			std::cout << "Ready to send kinect data." << std::endl;
			this->waitUntilDetectBody();
		}
	}
	else
	{
		this->waitUntilDetectBody();
	}

	this->calcAndSendJointQuaternions(sigservice);
}

///@brief Try connecting to SIGServer and SIGViewer.
void KinectDevice::connectToServerAndViewer(sigverse::SIGService &sigservice, bool &connectServerFlag, bool &connectViewerFlag)
{
	connectServerFlag = false;
	connectViewerFlag = false;

	if (sigservice.connect(this->serverAddress, this->portNumber))
	{
		fprintf(stdout, "Success to connect to SIGServer.\n");
		fflush(stdout);
		connectServerFlag = true;

		if (sigservice.connectToViewer())
		{
			fprintf(stdout, "Success to connect to SIGViewer.\n");
			fflush(stdout);
			connectViewerFlag = true;
		}
	}
}

///@brief Wait until it detects the body.
void KinectDevice::waitUntilDetectBody()
{
	while (true)
	{
		try
		{
			for (auto body : this->kinectConnector.bodies)
			{
				if (body == nullptr)
				{
					fprintf(stdout, "There are no bodies.\r");
					fflush(stdout);
					continue;
				}

				BOOLEAN isTracked = false;
				ERROR_CHECK(body->get_IsTracked(&isTracked));
				if (!isTracked) { continue; }

				// Get joint positions.
				body->GetJoints(JointType::JointType_Count, this->posture.positions);

				XnSkeletonJointPosition SpineBase;
				this->getSkeletonJointPosition(XN_SKEL_SPINEBASE, SpineBase);

				fprintf(stdout, "\rSpineBase Z position: %lf", (double)SpineBase.position.Z);
				fflush(stdout);

				// SpineBaseのz座標に値が入るとループを抜ける
				if (SpineBase.position.Z != 0.0)
				{
					this->readyToSend = true;
					this->startPosition = SpineBase;
					fprintf(stdout, "\n Start Measuring.\n");
					return;
				}
			}
		}
		catch (std::exception &ex)
		{
			std::cout << "drawBodyIndexFrame ERR :" << ex.what() << std::endl;
		}
	}
}

std::string KinectDevice::convertQuaternion2Message(JointNameAndQuaternionMap &mq)
{
	const double pCoef = 100.0;
	std::ostringstream ssPos;
	ssPos << "POSITION" << ":" << (pCoef * this->rootPosition.X * this->movingSpeed) << "," << (pCoef * this->rootPosition.Y * this->movingSpeed) << "," << (pCoef * this->rootPosition.Z * this->movingSpeed);

	std::ostringstream ssQuaternions;
	KinectDevice::JointNameAndQuaternionMap::iterator it = mq.begin();
	for (; it != mq.end(); it++)
	{
		ssQuaternions << " " << Util::getJointNameAndQuaternion((*it).first.c_str(), (*it).second);
	}

	std::ostringstream ssFinal;
	ssFinal << "KINECT_DATA" << " " << ssPos.str() << ssQuaternions.str() << " " << "END:";

	return ssFinal.str();
}
void KinectDevice::calcAndSendJointQuaternions(sigverse::SIGService &sigservice)
{
	while (true)
	{
		if (this->finishMeasuringFlag)
		{
			return;
		}

		JointNameAndQuaternionMap mq;
		try
		{
			this->calcWholeBodyJointQuaternions(mq);
		}
		catch (std::exception &ex)
		{
			std::cout << "drawBodyIndexFrame ERR :" << ex.what() << std::endl;
		}

		const std::string message = this->convertQuaternion2Message(mq);

		if (this->sendMessageFlag)
		{
			if (!sigservice.checkRecvData(0))
			{
				std::cout << "!sigService.checkRecvData(0)" << std::endl;
				//break;
			}

			std::vector<std::string> connectedNames = sigservice.getAllConnectedEntitiesName();
			for (int i = 0; i < (int)connectedNames.size(); i++)
			{
				sigservice.sendMsgToCtr(connectedNames[i].c_str(), message);
			}
			fprintf(stdout, "%s\r", message.c_str());
			fflush(stdout);
		}
		else
		{
			fprintf(stdout, "%s\r", message.c_str());
			fflush(stdout);
		}

		boost::this_thread::sleep(boost::posix_time::milliseconds(33));
	}

}

void KinectDevice::calcWholeBodyJointQuaternions(JointNameAndQuaternionMap &mq)
{
	XnSkeletonJointPosition SpineBase, torso, lhip, rhip, neck, lshoul,
		rshoul, lelb, relb, lknee, rknee, lhand, rhand, lfingertip, rfingertip, lankle, rankle, lfoot, rfoot, head;

	this->getSkeletonJointPosition(XN_SKEL_SPINEBASE, SpineBase);
	this->getSkeletonJointPosition(XN_SKEL_TORSO, torso);
	this->getSkeletonJointPosition(XN_SKEL_LEFT_HIP, lhip);
	this->getSkeletonJointPosition(XN_SKEL_RIGHT_HIP, rhip);
	this->getSkeletonJointPosition(XN_SKEL_NECK, neck);
	this->getSkeletonJointPosition(XN_SKEL_HEAD, head);
	this->getSkeletonJointPosition(XN_SKEL_LEFT_SHOULDER, lshoul);
	this->getSkeletonJointPosition(XN_SKEL_RIGHT_SHOULDER, rshoul);
	this->getSkeletonJointPosition(XN_SKEL_LEFT_ELBOW, lelb);
	this->getSkeletonJointPosition(XN_SKEL_RIGHT_ELBOW, relb);
	this->getSkeletonJointPosition(XN_SKEL_LEFT_HAND, lhand);
	this->getSkeletonJointPosition(XN_SKEL_RIGHT_HAND, rhand);
	this->getSkeletonJointPosition(XN_SKEL_LEFT_KNEE, lknee);
	this->getSkeletonJointPosition(XN_SKEL_RIGHT_KNEE, rknee);
	this->getSkeletonJointPosition(XN_SKEL_LEFT_FOOT, lfoot);
	this->getSkeletonJointPosition(XN_SKEL_RIGHT_FOOT, rfoot);

	this->getSkeletonJointPosition(XN_SKEL_LEFT_FINGERTIP, lfingertip);
	this->getSkeletonJointPosition(XN_SKEL_RIGHT_FINGERTIP, rfingertip);

	this->getSkeletonJointPosition(XN_SKEL_LEFT_KNEE, lknee);
	this->getSkeletonJointPosition(XN_SKEL_RIGHT_KNEE, rknee);
	this->getSkeletonJointPosition(XN_SKEL_LEFT_ANKLE, lankle);
	this->getSkeletonJointPosition(XN_SKEL_RIGHT_ANKLE, rankle);
	this->getSkeletonJointPosition(XN_SKEL_LEFT_FOOT, lfoot);
	this->getSkeletonJointPosition(XN_SKEL_RIGHT_FOOT, rfoot);

	this->rootPosition.X = -(SpineBase.position.X - this->startPosition.position.X);
	this->rootPosition.Y = SpineBase.position.Y - this->startPosition.position.Y;
	this->rootPosition.Z = -(SpineBase.position.Z - this->startPosition.position.Z);

	//calculate the rotation of entire body from left hip position and right hip position.
	XnPoint3D shoulder_vec;
	XnPoint3D khip_vec;
	if (Util::diffVec(khip_vec, rhip, lhip))
	{
		//					r sh val l sh val
		Util::diffVec(shoulder_vec, rshoul, lshoul);
		//                                                  p.X = 1;
		//                                                  p.Y = 0;
		//                                                  p.Z = 0; 
		//rshoulder means the quaternion calculated from (1,0,0) and rshouder vector.
		Quaternion rshoulder = Util::calcQuaternion(shoulder_vec, Util::getSigVec(Util::HIP));


		//	Util::rotateVec(khip_vec, rshoulder);
		//set body direction
		//                                                  p.X = 1;
		//                                                  p.Y = 0;
		//                                                  p.Z = 0;         
		//waist quaternion is calculated from (1,0,0) and hip vector
		mq.insert(JointNameAndQuaternionMap::value_type("WAIST", Util::calcQuaternion(Util::getSigVec(Util::HIP), khip_vec)));

		//                                               p.X = 1;
		//                                               p.Y = 0;
		//                                               p.Z = 0;  
		//rrootq quaternion is calculated from hip vector and  (1,0,0)
		Quaternion rrootq = Util::calcQuaternion(khip_vec, Util::getSigVec(Util::HIP));

		//waist
		XnPoint3D kneck_vec;
		//kneck: vector from torso to  neck
		if (Util::diffVec(kneck_vec, torso, neck))
		{
			//whole body rotation
			Util::rotateVec(kneck_vec, rrootq);
			//Util::rotateVec(kneck_vec, rroot);
			//tlanslation by whole body rotation 

			//Util::rotateVec(kneck_vec, rshoulder);
			//                       p.X = 0;
			//                       p.Y = 1;
			//                       p.Z = 0;
			XnPoint3D sneck_vec = Util::getSigVec(Util::WAIST);
			//showXnPoint3D("shoulder_vec", shoulder_vec);

			//                                                        p.X = 0;
			//                                                        p.Y = 1;
			//                                                        p.Z = 0;   kneck: vector from torso to  neck
			mq.insert(JointNameAndQuaternionMap::value_type("WAIST_JOINT1", Util::calcQuaternion(sneck_vec, kneck_vec)));

			//mq.insert(JointNameAndQuaternionMap::value_type("WAIST_JOINT1", Util::calcQuaternion(sneck_vec, kneck_vec)));
			//                                              p.X = 0;
			//                                              p.Y = 1;
			//           kneck: vector from torso to  neck  p.Z = 0;   
			Quaternion rwaist = Util::calcQuaternion(kneck_vec, sneck_vec);
			XnPoint3D shoulder_vec1;
			if (Util::diffVec(shoulder_vec1, rshoul, lshoul))
			{
				//Util::rotateVec(shoulder_vec1, rrootq);
				//	Util::rotateVec(shoulder_vec1, rwaist);
				//                                                          p.X = 1;
				//                                                          p.Y = 0;
				//                                                          p.Z = 0;       rsh to lsh
				mq.insert(JointNameAndQuaternionMap::value_type("WAIST_JOINT0", Util::calcQuaternion(Util::getSigVec(Util::HIP), shoulder_vec1)));
			}
			XnPoint3D khead_vec;
			if (Util::diffVec(khead_vec, neck, head))
			{
				Util::rotateVec(kneck_vec, rrootq);
				Util::rotateVec(kneck_vec, rwaist);

				mq.insert(JointNameAndQuaternionMap::value_type("HEAD_JOINT1", Util::calcQuaternion(sneck_vec, khead_vec)));
			}


			XnPoint3D krsh_vec;
			if (Util::diffVec(krsh_vec, rshoul, relb))
			{
				XnPoint3D srsh_vec = Util::getSigVec(Util::RSHOULDER);

				Util::rotateVec(krsh_vec, rrootq);
				Util::rotateVec(krsh_vec, rwaist);

				mq.insert(JointNameAndQuaternionMap::value_type("RARM_JOINT2", Util::calcQuaternion(srsh_vec, krsh_vec)));

				Quaternion rrsh = Util::calcQuaternion(krsh_vec, srsh_vec);

				XnPoint3D krel_vec;
				if (Util::diffVec(krel_vec, relb, rhand))
				{
					//Coordinate system tlanslation by entire body and waist.
					Util::rotateVec(krel_vec, rrootq);
					Util::rotateVec(krel_vec, rwaist);
					Util::rotateVec(krel_vec, rrsh);

					//Calculate quaternion
					mq.insert(JointNameAndQuaternionMap::value_type("RARM_JOINT3", Util::calcQuaternion(srsh_vec, krel_vec)));
					Quaternion rrel = Util::calcQuaternion(krel_vec, srsh_vec);

					//right wrist
					XnPoint3D krwrist_vec;
					if (Util::diffVec(krwrist_vec, rhand, rfingertip))
					{
						//Coordinate system tlanslation by entire body and waist.
						Util::rotateVec(krwrist_vec, rrootq);
						Util::rotateVec(krwrist_vec, rwaist);
						Util::rotateVec(krwrist_vec, rrsh);
						Util::rotateVec(krwrist_vec, rrel);

						//Calculate quaternion
						mq.insert(JointNameAndQuaternionMap::value_type("RARM_JOINT5", Util::calcQuaternion(srsh_vec, krwrist_vec)));
					}
				}
			}

			XnPoint3D klsh_vec;
			if (Util::diffVec(klsh_vec, lshoul, lelb))
			{
				XnPoint3D slsh_vec = Util::getSigVec(Util::LSHOULDER);

				//Coordinate system tlanslation by entire body and waist.
				Util::rotateVec(klsh_vec, rrootq);
				Util::rotateVec(klsh_vec, rwaist);

				mq.insert(JointNameAndQuaternionMap::value_type("LARM_JOINT2", Util::calcQuaternion(slsh_vec, klsh_vec)));
				Quaternion rlsh = Util::calcQuaternion(klsh_vec, slsh_vec); //‹t‰ñ“]

				XnPoint3D klel_vec;
				if (Util::diffVec(klel_vec, lelb, lhand))
				{
					//Coordinate system tlanslation by entire body and waist.
					Util::rotateVec(klel_vec, rrootq);
					Util::rotateVec(klel_vec, rwaist);
					Util::rotateVec(klel_vec, rlsh);

					mq.insert(JointNameAndQuaternionMap::value_type("LARM_JOINT3", Util::calcQuaternion(slsh_vec, klel_vec)));
					Quaternion rlel = Util::calcQuaternion(klel_vec, slsh_vec);

					//left wrist
					XnPoint3D klwrist_vec;
					if (Util::diffVec(klwrist_vec, lhand, lfingertip))
					{
						//Coordinate system tlanslation by entire body and waist.
						Util::rotateVec(klwrist_vec, rrootq);
						Util::rotateVec(klwrist_vec, rwaist);
						Util::rotateVec(klwrist_vec, rlsh);
						Util::rotateVec(klwrist_vec, rlel);

						//calculate quaternion
						mq.insert(JointNameAndQuaternionMap::value_type("LARM_JOINT5", Util::calcQuaternion(slsh_vec, klwrist_vec)));
					}
				}
			}
		}

		XnPoint3D sleg_vec = Util::getSigVec(Util::LEG);
		XnPoint3D sfoot_vec = Util::getSigVec(Util::FOOT);

		//root of right leg
		XnPoint3D krhip_vec;
		if (Util::diffVec(krhip_vec, rhip, rknee))
		{
			//Coordinate system tlanslation of vector in Kinect accoding to parent joint 
			Util::rotateVec(krhip_vec, rrootq);

			//calculate quaternion
			mq.insert(JointNameAndQuaternionMap::value_type("RLEG_JOINT2", Util::calcQuaternion(sleg_vec, krhip_vec)));
			Quaternion rrhp = Util::calcQuaternion(krhip_vec, sleg_vec);

			//right knee
			XnPoint3D krknee_vec;
			if (Util::diffVec(krknee_vec, rknee, rankle))
			{
				//Coordinate system tlanslation of vector in Kinect accoding to parent joint 
				Util::rotateVec(krknee_vec, rrootq);
				Util::rotateVec(krknee_vec, rrhp);

				//calculate quaternion
				mq.insert(JointNameAndQuaternionMap::value_type("RLEG_JOINT4", Util::calcQuaternion(sleg_vec, krknee_vec)));
				Quaternion rrknee = Util::calcQuaternion(krknee_vec, sleg_vec);

				//right ankle
				XnPoint3D krankle_vec;
				if (Util::diffVec(krankle_vec, rankle, rfoot))
				{
					//Coordinate system tlanslation by entire body and waist.
					Util::rotateVec(krankle_vec, rrootq);
					Util::rotateVec(krankle_vec, rrhp);
					Util::rotateVec(krankle_vec, rrknee);

					//calculate quaternion
					mq.insert(JointNameAndQuaternionMap::value_type("RLEG_JOINT6", Util::calcQuaternion(sfoot_vec, krankle_vec)));
				}
			}
		}

		//root of left leg
		XnPoint3D klhip_vec;
		if (Util::diffVec(klhip_vec, lhip, lknee))
		{
			//Coordinate system tlanslation by entire body and waist.
			Util::rotateVec(klhip_vec, rrootq);

			//Calculate quaternoin
			mq.insert(JointNameAndQuaternionMap::value_type("LLEG_JOINT2", Util::calcQuaternion(sleg_vec, klhip_vec)));
			Quaternion rlhp = Util::calcQuaternion(klhip_vec, sleg_vec);

			//left knee
			XnPoint3D klknee_vec;
			if (Util::diffVec(klknee_vec, lknee, lankle))
			{
				Util::rotateVec(klknee_vec, rrootq);
				Util::rotateVec(klknee_vec, rlhp);

				//calculate quaternion
				mq.insert(JointNameAndQuaternionMap::value_type("LLEG_JOINT4", Util::calcQuaternion(sleg_vec, klknee_vec)));
				Quaternion rlknee = Util::calcQuaternion(klknee_vec, sleg_vec);

				//left ankle
				XnPoint3D klankle_vec;
				if (Util::diffVec(klankle_vec, lankle, lfoot))
				{
					//Coordinate system tlanslation by entire body and waist.
					Util::rotateVec(klankle_vec, rrootq);
					Util::rotateVec(klankle_vec, rlhp);
					Util::rotateVec(klankle_vec, rlknee);

					//calculate quaternion
					mq.insert(JointNameAndQuaternionMap::value_type("LLEG_JOINT6", Util::calcQuaternion(sfoot_vec, klankle_vec)));
				}
			}
		}
	}

}

///@brief Get skeleton joint position.
void KinectDevice::getSkeletonJointPosition(XnSkeletonJoint eJoint, XnSkeletonJointPosition& Joint)
{
	switch (eJoint)
	{
	case XN_SKEL_HEAD:
		if (this->posture.positions[3].TrackingState == 0) { Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[3].Position.X;
		Joint.position.Y = this->posture.positions[3].Position.Y;
		Joint.position.Z = this->posture.positions[3].Position.Z;
		break;
	case XN_SKEL_NECK:
		if (this->posture.positions[2].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[2].Position.X;
		Joint.position.Y = this->posture.positions[2].Position.Y;
		Joint.position.Z = this->posture.positions[2].Position.Z;
		break;
	case XN_SKEL_SPINEBASE:
		if (this->posture.positions[0].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[0].Position.X;
		Joint.position.Y = this->posture.positions[0].Position.Y;
		Joint.position.Z = this->posture.positions[0].Position.Z;
		break;
	case XN_SKEL_TORSO:
		if (this->posture.positions[1].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[1].Position.X;
		Joint.position.Y = this->posture.positions[1].Position.Y;
		Joint.position.Z = this->posture.positions[1].Position.Z;
		break;
	case XN_SKEL_WAIST:
		if (this->posture.positions[0].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[0].Position.X;
		Joint.position.Y = this->posture.positions[0].Position.Y;
		Joint.position.Z = this->posture.positions[0].Position.Z;
		break;
	case XN_SKEL_LEFT_COLLAR:
		if (this->posture.positions[20].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[20].Position.X;
		Joint.position.Y = this->posture.positions[20].Position.Y;
		Joint.position.Z = this->posture.positions[20].Position.Z;
		break;
	case XN_SKEL_LEFT_SHOULDER:
		if (this->posture.positions[4].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[4].Position.X;
		Joint.position.Y = this->posture.positions[4].Position.Y;
		Joint.position.Z = this->posture.positions[4].Position.Z;
		break;
	case XN_SKEL_LEFT_ELBOW:
		if (this->posture.positions[5].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[5].Position.X;
		Joint.position.Y = this->posture.positions[5].Position.Y;
		Joint.position.Z = this->posture.positions[5].Position.Z;
		break;
	case XN_SKEL_LEFT_WRIST:
		if (this->posture.positions[6].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[6].Position.X;
		Joint.position.Y = this->posture.positions[6].Position.Y;
		Joint.position.Z = this->posture.positions[6].Position.Z;
		break;
	case XN_SKEL_LEFT_HAND:
		if (this->posture.positions[7].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[7].Position.X;
		Joint.position.Y = this->posture.positions[7].Position.Y;
		Joint.position.Z = this->posture.positions[7].Position.Z;
		break;
	case XN_SKEL_LEFT_FINGERTIP:
		if (this->posture.positions[21].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[21].Position.X;
		Joint.position.Y = this->posture.positions[21].Position.Y;
		Joint.position.Z = this->posture.positions[21].Position.Z;
		break;
	case XN_SKEL_RIGHT_COLLAR:
		if (this->posture.positions[20].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[20].Position.X;
		Joint.position.Y = this->posture.positions[20].Position.Y;
		Joint.position.Z = this->posture.positions[20].Position.Z;
		break;
	case XN_SKEL_RIGHT_SHOULDER:
		if (this->posture.positions[8].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[8].Position.X;
		Joint.position.Y = this->posture.positions[8].Position.Y;
		Joint.position.Z = this->posture.positions[8].Position.Z;
		break;
	case XN_SKEL_RIGHT_ELBOW:
		if (this->posture.positions[9].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[9].Position.X;
		Joint.position.Y = this->posture.positions[9].Position.Y;
		Joint.position.Z = this->posture.positions[9].Position.Z;
		break;
	case XN_SKEL_RIGHT_WRIST:
		if (this->posture.positions[10].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[10].Position.X;
		Joint.position.Y = this->posture.positions[10].Position.Y;
		Joint.position.Z = this->posture.positions[10].Position.Z;
		break;
	case XN_SKEL_RIGHT_HAND:
		if (this->posture.positions[11].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[11].Position.X;
		Joint.position.Y = this->posture.positions[11].Position.Y;
		Joint.position.Z = this->posture.positions[11].Position.Z;
		break;
	case XN_SKEL_RIGHT_FINGERTIP:
		if (this->posture.positions[23].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[23].Position.X;
		Joint.position.Y = this->posture.positions[23].Position.Y;
		Joint.position.Z = this->posture.positions[23].Position.Z;
		break;
	case XN_SKEL_LEFT_HIP:
		if (this->posture.positions[12].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[12].Position.X;
		Joint.position.Y = this->posture.positions[12].Position.Y;
		Joint.position.Z = this->posture.positions[12].Position.Z;
		break;
	case XN_SKEL_LEFT_KNEE:
		if (this->posture.positions[13].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[13].Position.X;
		Joint.position.Y = this->posture.positions[13].Position.Y;
		Joint.position.Z = this->posture.positions[13].Position.Z;
		break;
	case XN_SKEL_LEFT_ANKLE:
		if (this->posture.positions[14].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[14].Position.X;
		Joint.position.Y = this->posture.positions[14].Position.Y;
		Joint.position.Z = this->posture.positions[14].Position.Z;
		break;
	case XN_SKEL_LEFT_FOOT:
		if (this->posture.positions[15].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[15].Position.X;
		Joint.position.Y = this->posture.positions[15].Position.Y;
		Joint.position.Z = this->posture.positions[15].Position.Z;
		break;
	case XN_SKEL_RIGHT_HIP:
		if (this->posture.positions[16].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[16].Position.X;
		Joint.position.Y = this->posture.positions[16].Position.Y;
		Joint.position.Z = this->posture.positions[16].Position.Z;
		break;
	case XN_SKEL_RIGHT_KNEE:
		if (this->posture.positions[17].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[17].Position.X;
		Joint.position.Y = this->posture.positions[17].Position.Y;
		Joint.position.Z = this->posture.positions[17].Position.Z;
		break;
	case XN_SKEL_RIGHT_ANKLE:
		if (this->posture.positions[18].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[18].Position.X;
		Joint.position.Y = this->posture.positions[18].Position.Y;
		Joint.position.Z = this->posture.positions[18].Position.Z;
		break;
	case XN_SKEL_RIGHT_FOOT:
		if (this->posture.positions[19].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = this->posture.positions[19].Position.X;
		Joint.position.Y = this->posture.positions[19].Position.Y;
		Joint.position.Z = this->posture.positions[19].Position.Z;
		break;
	default:
		break;
	}

}


void KinectDevice::calcWholeBodyJointQuaternionsFromKinectQuaternions(JointNameAndQuaternionMap &mq)
{
	const double coef = 1.0 / sqrt(2.0);

	Quaternion q0 = Quaternion(this->posture.orientations[0].Orientation);
	Quaternion q1 = Quaternion(this->posture.orientations[1].Orientation);
	Quaternion q1_h = Quaternion(0.0, 0.0, 1.0, 0.0);
	Quaternion q1_rot = Util::calcCrossProduct(q1_h, q1);
	Quaternion q2 = Quaternion(this->posture.orientations[20].Orientation);
	Quaternion q2_h = Quaternion(0.0, 0.0, -1.0, 0.0);
	Quaternion q2_rot = Util::calcCrossProduct(q2, q2_h);

	Quaternion q3 = Quaternion(this->posture.orientations[8].Orientation);
	Quaternion q4 = Quaternion(this->posture.orientations[9].Orientation);
	Quaternion q7 = Quaternion(this->posture.orientations[5].Orientation);
	Quaternion q9 = Quaternion(this->posture.orientations[10].Orientation);
	Quaternion q13 = Quaternion(this->posture.orientations[17].Orientation);
	Quaternion q15 = Quaternion(this->posture.orientations[13].Orientation);
	Quaternion q17 = Quaternion(this->posture.orientations[18].Orientation);
	Quaternion q19 = Quaternion(this->posture.orientations[14].Orientation);
	Quaternion q20 = Quaternion(this->posture.orientations[20].Orientation);
	Quaternion q20_rot = Util::calcCrossProduct(q1_h, q20);
	Quaternion q21 = Quaternion(this->posture.orientations[6].Orientation);
	Quaternion q25 = Quaternion(this->posture.orientations[11].Orientation);
	Quaternion q26 = Quaternion(this->posture.orientations[7].Orientation);
	Quaternion q1_con = Quaternion(q20_rot.w, -q20_rot.x, -q20_rot.y, -q20_rot.z);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////  Right Arm quaternion //////////////////////
	Quaternion q5;
	Quaternion q5_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q5_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion q5_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion q5_rotx = Util::calcCrossProduct(q5_x, q4);
	q5.w = q5_rotx.w;
	q5.x = q5_rotx.y;
	q5.y = q5_rotx.z;
	q5.z = q5_rotx.x;
	q5 = Util::calcCrossProduct(q2_h, q5);

	q5 = Util::calcCrossProduct(q5_y, q5);
	q5.w = q5.w;
	q5.x = -q5.x;
	q5.y = -q5.y;
	q5.z = q5.z;
	q5 = Util::calcCrossProduct(q1_con, q5);
	//////////////////////////////////////////////////////////////////////

	////////////// Left Arm quaternion ///////////////////////////////////
	Quaternion q6;
	Quaternion q6_x = Quaternion(coef, coef, 0.0, 0.0);
	Quaternion q6_z = Quaternion(coef, 0.0, 0.0, coef);
	Quaternion q6_y = Quaternion(coef, 0.0, coef, 0.0);
	Quaternion q6_rotx = Util::calcCrossProduct(q6_x, q7);
	q6.w = q6_rotx.w;
	q6.x = q6_rotx.y;
	q6.y = q6_rotx.z;
	q6.z = q6_rotx.x;
	q6 = Util::calcCrossProduct(q2_h, q6);
	q6 = Util::calcCrossProduct(q6_y, q6);
	q6.w = q6.w;
	q6.x = q6.x;
	q6.y = q6.y;
	q6.z = q6.z;
	q6 = Util::calcCrossProduct(q1_con, q6);

	//////////////////////////////////////////////////////////////////////////

	///////////////////////////  Right hipe quaternion //////////////////////
	Quaternion q12;
	Quaternion q12_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q12_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion q12_y = Quaternion(coef, 0.0, coef, 0.0);
	Quaternion q12_rotx = Util::calcCrossProduct(q12_x, q13);
	Quaternion q3_h = Quaternion(0.0, 0.0, 1.0, 0.0);
	Quaternion q12_h = Quaternion(0.0, 0.0, 0.0, -1.0);
	q12 = Util::calcCrossProduct(q12_h, q13);
	Quaternion q12_b = Util::calcCrossProduct(q12_y, q12);
	q12.w = q12_b.w;
	q12.x = -q12_b.z;
	q12.y = -q12_b.y;
	q12.z = -q12_b.x;
	//////////////////////////////////////////////////////////////////////

	///////////////////////////  Right leg quaternion //////////////////////
	Quaternion q16;
	Quaternion q16_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q16_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion q16_y = Quaternion(coef, 0.0, coef, 0.0);
	Quaternion q16_rotx = Util::calcCrossProduct(q16_x, q17);
	Quaternion q16_h = Quaternion(0.0, 0.0, 0.0, -1.0);
	q16 = Util::calcCrossProduct(q16_h, q17);
	Quaternion q16_b = Util::calcCrossProduct(q16_y, q16);
	q16.w = q16_b.w;
	q16.x = -q16_b.z;
	q16.y = -q16_b.y;
	q16.z = -q16_b.x;

	Quaternion  q12_con;
	q12_con.w = q12.w;
	q12_con.x = -q12.x;
	q12_con.y = -q12.y;
	q12_con.z = -q12.z;
	q16 = Util::calcCrossProduct(q12_con, q16);
	//////////////////////////////////////////////////////////////////////


	///////////////////////////  left hipe quaternion //////////////////////
	Quaternion q14;
	Quaternion q14_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q14_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion q14_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion q14_rotx = Util::calcCrossProduct(q14_x, q15);

	Quaternion q14_h = Quaternion(0.0, 0.0, 0.0, 1.0);
	q14 = Util::calcCrossProduct(q14_h, q15);
	Quaternion q14_b = Util::calcCrossProduct(q14_y, q14);
	q14.w = q14_b.w;
	q14.x = q14_b.z;
	q14.y = -q14_b.y;
	q14.z = q14_b.x;
	//q14 = Util::calcCrossProduct(q2_h, q14);
	//////////////////////////////////////////////////////////////////////


	///////////////////////////  left leg quaternion //////////////////////
	Quaternion q18;
	Quaternion q18_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q18_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion q18_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion q18_rotx = Util::calcCrossProduct(q18_x, q19);
	Quaternion q18_h = Quaternion(0.0, 0.0, 0.0, 1.0);
	q18 = Util::calcCrossProduct(q18_h, q19);
	Quaternion q18_b = Util::calcCrossProduct(q18_y, q18);
	q18.w = q18_b.w;
	q18.x = q18_b.z;
	q18.y = -q18_b.y;
	q18.z = q18_b.x;

	Quaternion  q14_con;
	q14_con.w = q14.w;
	q14_con.x = -q14.x;
	q14_con.y = -q14.y;
	q14_con.z = -q14.z;
	q18 = Util::calcCrossProduct(q14_con, q18);
	//////////////////////////////////////////////////////////////////////


	////////////// Right for Arm  quaternion /////////////////////////////
	Quaternion q8;
	Quaternion q8_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q8_z = Quaternion(coef, 0.0, 0.0, coef);
	Quaternion q8_y = Quaternion(coef, 0.0, coef, 0.0);
	Quaternion q8_rotx = Util::calcCrossProduct(q8_x, q9);

	q8.w = q8_rotx.w;
	q8.x = q8_rotx.y;
	q8.y = q8_rotx.z;
	q8.z = q8_rotx.x;
	q8 = Util::calcCrossProduct(q8_y, q8);
	Quaternion q8_xx = Quaternion(0.0, 1.0, 0.0, 0.0);
	Quaternion q8_zz = Quaternion(0.0, 0.0, 0.0, 1.0);
	Quaternion  q8_va = Quaternion(q8.w, q8.x, q8.y, q8.z);

	q8 = Util::calcCrossProduct(q2_h, q8);
	q8.w = q8_va.w;
	q8.x = -q8_va.x;
	q8.y = -q8_va.y;
	q8.z = q8_va.z;

	Quaternion q5_con;
	q5_con.w = q5.w;
	q5_con.x = -q5.x;
	q5_con.y = -q5.y;
	q5_con.z = -q5.z;
	q8 = Util::calcCrossProduct(q5_con, q8);
	////////////////////////////////////////////////////////////////////////

	////////////// Left for Arm  quaternion ////////////////////////////////
	Quaternion q22;
	Quaternion q22_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q22_z = Quaternion(coef, 0.0, 0.0, coef);
	Quaternion q22_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion q22_rotx = Util::calcCrossProduct(q22_x, q21);

	q22.w = q22_rotx.w;
	q22.x = q22_rotx.y;
	q22.y = q22_rotx.z;
	q22.z = q22_rotx.x;
	q22 = Util::calcCrossProduct(q22_y, q22);

	Quaternion q22_xx = Quaternion(0.0, 1.0, 0.0, 0.0);
	Quaternion q22_zz = Quaternion(0.0, 0.0, 0.0, 1.0);
	Quaternion  q22_va = Quaternion(q22.w, q22.x, q22.y, q22.z);

	q22 = Util::calcCrossProduct(q2_h, q8);
	q22.w = q22_va.w;
	q22.x = q22_va.x;
	q22.y = -q22_va.y;
	q22.z = -q22_va.z;

	Quaternion q6_con;
	q6_con.w = q6.w;
	q6_con.x = -q6.x;
	q6_con.y = -q6.y;
	q6_con.z = -q6.z;
	q22 = Util::calcCrossProduct(q6_con, q22);

	//////////////////////////////////////////////////////////////////////////



	////////////// Right hand  quaternion ////////////////////////////////////
	Quaternion q27;
	Quaternion q27_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q27_z = Quaternion(coef, 0.0, 0.0, coef);
	Quaternion q27_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion q27_rotx = Util::calcCrossProduct(q27_x, q25);
	q27.w = q27_rotx.w;
	q27.x = q27_rotx.y;
	q27.y = q27_rotx.z;
	q27.z = q27_rotx.x;
	Quaternion q27_xx = Quaternion(0.0, 1.0, 0.0, 0.0);
	Quaternion q27_zz = Quaternion(0.0, 0.0, 0.0, 1.0);
	Quaternion  q27_va = Quaternion(q27.w, q27.x, q27.y, q27.z);
	q27 = Util::calcCrossProduct(q2_h, q27);
	q27.w = q27_va.w;
	q27.x = q27_va.x;
	q27.y = q27_va.y;
	q27.z = q27_va.z;
	Quaternion q8_con;
	q8_con.w = q8.w;
	q8_con.x = -q8.x;
	q8_con.y = -q8.y;
	q8_con.z = -q8.z;
	q27 = Util::calcCrossProduct(q8_con, q27);
	//////////////////////////////////////////////////////////////////////////


	////////////// Left Hand  quaternion ///////////////////////////////////
	Quaternion q28;
	Quaternion q28_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q28_z = Quaternion(coef, 0.0, 0.0, coef);
	Quaternion q28_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion q28_rotx = Util::calcCrossProduct(q28_x, q26);

	q28.w = q28_rotx.w;
	q28.x = q28_rotx.y;
	q28.y = q28_rotx.z;
	q28.z = q28_rotx.x;
	q28 = Util::calcCrossProduct(q28_y, q28);
	Quaternion q28_xx = Quaternion(0.0, 1.0, 0.0, 0.0);
	Quaternion q28_zz = Quaternion(0.0, 0.0, 0.0, 1.0);
	Quaternion  q28_va = Quaternion(q28.w, q28.x, q28.y, q28.z);
	q28 = Util::calcCrossProduct(q2_h, q28);
	q28.w = q28_va.w;
	q28.x = q28_va.x;
	q28.y = -q28_va.y;
	q28.z = -q28_va.z;
	Quaternion q22_con;
	q22_con.w = q22.w;
	q22_con.x = -q22.x;
	q22_con.y = -q22.y;
	q22_con.z = -q22.z;
	q28 = Util::calcCrossProduct(q22_con, q28);

	mq.insert(KinectDevice::JointNameAndQuaternionMap::value_type("WAIST_JOINT1", q1_rot));
	mq.insert(KinectDevice::JointNameAndQuaternionMap::value_type("RARM_JOINT2", q5));
	mq.insert(KinectDevice::JointNameAndQuaternionMap::value_type("LARM_JOINT2", q6));
	mq.insert(KinectDevice::JointNameAndQuaternionMap::value_type("RLEG_JOINT2", q12));
	mq.insert(KinectDevice::JointNameAndQuaternionMap::value_type("LLEG_JOINT2", q14));
	mq.insert(KinectDevice::JointNameAndQuaternionMap::value_type("RLEG_JOINT4", q16));
	mq.insert(KinectDevice::JointNameAndQuaternionMap::value_type("LLEG_JOINT4", q18));
	mq.insert(KinectDevice::JointNameAndQuaternionMap::value_type("RARM_JOINT3", q8));
	mq.insert(KinectDevice::JointNameAndQuaternionMap::value_type("LARM_JOINT3", q22));
	mq.insert(KinectDevice::JointNameAndQuaternionMap::value_type("RARM_JOINT5", q27));
	mq.insert(KinectDevice::JointNameAndQuaternionMap::value_type("LARM_JOINT5", q28));

}