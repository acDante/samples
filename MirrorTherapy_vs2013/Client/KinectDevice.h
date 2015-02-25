#pragma once
#include <SIGService.h>
#include "KinectPosture.h"
#include <opencv2/opencv.hpp>
#include <map>
#include "Util.h"
#include <sstream>


class Device
{
public:
	Device(){};
	virtual ~Device(){};
};



///@brief Variables for using kinect.
///@brief �L�l�N�g�p�̕ϐ��Q���܂Ƃ߂������̃N���X�D�\���̂ł������D
class KinectConnector
{
public:
	//-- Varibles to connect to kinect. --//
	IKinectSensor *kinect = nullptr;
	IBodyFrameReader *bodyFrameReader = nullptr;
	IBody *bodies[6];
	IColorFrameReader *colorFrameReader = nullptr;
	std::vector<BYTE> colorBuffer;

	//-- Meta information of color image. --//
	int colorFrameWidth;
	int colorFrameHeight;
	unsigned int colorBytesPerPixel;

	//-- Meta information of depth image. --//
	const int depthImageHeight = 424;
	const int depthImageWidth = 512;

	//-- Variables to draw posture in opencv window. --//
	const cv::Scalar RED = cv::Scalar(0, 0, 255);
	const cv::Scalar GREEN = cv::Scalar(0, 255, 0);
	const cv::Scalar BLUE = cv::Scalar(255, 0, 0);
	const cv::Scalar CYAN = cv::Scalar(255, 255, 0);
	const cv::Scalar YELLOW = cv::Scalar(0, 255, 255);
	const int ellipseRadius = 8;
	const int lineThickness = 5;

	//-- Image to draw. --//
	cv::Mat sceneImage;
	cv::Mat bodyImage;
};

///@brief Class for using kinect device.
class KinectDevice : public Device
{
private:

	///@brief Human posture of one frame measured by kinect.
	KinectPosture posture;

	///@brief Variables to use kinect device.
	KinectConnector kinectConnector;

	///@brief Parameter file name.
	const std::string PARAM_FILE_NAME = "config.ini";

	///@brief Command line parameter. IP address.
	std::string serverAddress;

	///@brief Command line parameter. Port number.
	int portNumber;

	///@brief Service name on SIGService.
	std::string serviceName;

	///@brief Moving speed of avatar.
	double movingSpeed;

	///@brief Typedef for std::map of joint name and quaternion.
	typedef std::map<std::string, Quaternion> JointNameAndQuaternionMap;

	///@brief std::map for joint name and quaternion.
	//JointNameAndQuaternionMap qmap;

	///@brief Body root(spine base) position.
	XnPoint3D rootPosition;

	///@brief Flag of send message to server.
	bool sendMessageFlag;

	///@brief Flag of to be ready to send information.
	bool readyToSend;

	///@brief Body root(spine base) postion when it starts measuring.
	XnSkeletonJointPosition startPosition;

	///@brief Flag of finish measuring.
	bool finishMeasuringFlag;

	// std::string deviceType;
	// std::string deviceUniqueID;


public:
	KinectDevice(int argc, char **argv);
	~KinectDevice();


	///@brief Read the parameter file.
	void readIniFile();

	///@brief Initialize kinect device.
	void initializeKinect();

	///@brief Run draw methods.
	void runDrawingMethods();

	///@brief Run send methods.
	void runSendingMethods();

	//-- Draw methods --//

	///@brief Update body frame.
	void updateBodyFrame();

	///@brief Draw body.
	void draw();

	///@brief Draw color scene.
	void drawScene();

	///@brief Draw body index.
	void drawBodyIndexFrame();

	///@brief Get body position to color scene.
	void getPositionToColorFrame(const CameraSpacePoint &bodyPoint, cv::Point &point, const int width, const int height);

	///@brief Draw connecting all joints lines.
	void drawLines(cv::Mat &bodyImage, const Joint *joints);

	///@brief Draw connecting line at each joint.
	void drawLine(cv::Mat &bodyImage, const Joint &joint1, const Joint &joint2, const cv::Scalar &lineColor, const int thickness);

	///@brief Draw ellipses at each joint.
	void drawEllipse(cv::Mat &bodyImage, const Joint &joint, const int r, const cv::Scalar &color);

	//-- Methods to send information to SIGServer. --//
	
	///@brief Try connecting to SIGServer and SIGViewer.
	void connectToServerAndViewer(sigverse::SIGService &sigservice, bool &connectServerFlag, bool &connectViewerFlag);
	
	///@brief Get skeleton joint position.
	void getSkeletonJointPosition(XnSkeletonJoint eJoint, XnSkeletonJointPosition& Joint);

	///@brief Wait until it detects the body.
	void waitUntilDetectBody();

	///@brief Measure body after detects the body.
	void calcAndSendJointQuaternions(sigverse::SIGService &sigservice);

	///@brief Calculate whole body joint quaternions.
	// �|�W�V��������SIGVerse�p�̃N�H�[�^�j�I���֕ϊ�����
	void calcWholeBodyJointQuaternions(JointNameAndQuaternionMap &mq);

	///@brief Calculate whole body joint quaternions from kinect quaternions.
	// �L�l�N�g�̃N�H�[�^�j�I������SIGVerse�p�̃N�H�[�^�j�I���֕ϊ�����
	void calcWholeBodyJointQuaternionsFromKinectQuaternions(JointNameAndQuaternionMap &mq);

	///@brief Generate quaternion message for sigverse Man-nii joints.
	// �{����Device���ł͂Ȃ��āCPosture���ɋL�q����ׂ��D
	// ����ł́CDevice���ŃL�l�N�g�̊֐ߏ�񂩂�SIGVerse�̊֐ߏ��ւ̕ϊ��𐿂������Ă���̂ŁC
	// Device���Ƀ��b�Z�[�W�����̋@�\������D
	std::string convertQuaternion2Message(JointNameAndQuaternionMap &mq);

	///@brief Generate quaternions for sigverse Man-nii joints.
	// sigverse��Man-nii�p�̃N�H�[�^�j�I���𐶐�����D
	// �����炭�́Ckinect�ŎZ�o����kinectPosture�N���X��JointOrientation�����̂܂�sigverse�֑����������ǂ��D
	// sigverse�p�̃N�H�[�^�j�I���֕ϊ�����̂�sigverse(sigserver)���ōs���ׂ��D
	//void generateSigverseQuaternionFromSkeletonJointPosition(JointNameAndQuaternionMap &mq, double &posx, double &posy, double &posz);

};


///@brief Kinect�g�p���ɕK�v�ȃ}�N��
// ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) ); // �g�p��
#define ERROR_CHECK( ret )  \
	if ( (ret) != S_OK )  \
				{  \
		std::stringstream ss;  \
		ss << "failed " #ret " " << std::hex << ret << std::endl;  \
		throw std::runtime_error( ss.str().c_str() );  \
				}