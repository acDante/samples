#ifndef _KINECTAPP_
#define _KINECTAPP_

#include "MyService.h"
#include "Logger.h"
#include <XnTypes.h>
#include <Kinect.h>
#include <opencv2\opencv.hpp>
#include <d2d1.h>
#include <string>





class KinectApp
{

// 次のように使います
// ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );
// 書籍での解説のためにマクロにしています。実際には展開した形で使うことを検討してください。
#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
	    }

	struct Quaternion {
		float qw;
		float qx;
		float qy;
		float qz;
	};

	//sigverse::SIGService *m_srv;
	typedef std::map<std::string, Quaternion> QMap;
	//typedef std::map<std::string, Vector4> QMap;

	enum SigVec
	{
		HIP = 0,
		HTOTOR,
		WAIST,
		RSHOULDER,
		LSHOULDER,
		RELBOW,
		LELBOW,
		LEG,
		FOOT
	};




private:
	IKinectSensor* kinect = nullptr;
	IBodyFrameReader* bodyFrameReader = nullptr;
	IBody* bodies[6];
	MyService* srv;
	JointOrientation jointorientations[JointType_Count];
	Joint joints[JointType_Count];
	Logger* log;

	//IInfraredFrameReader* infraredFrameReader = nullptr;
	//std::vector<UINT16> infraredBuffer;
	int width = 512;
	int height = 424;
	//int colorWidth = 424;
	//int colorHeight = 512;
	int colorWidth;
	int colorHeight;
	IColorFrameReader* colorFrameReader = nullptr;
	std::vector<BYTE> colorBuffer;
	unsigned int colorBytesPerPixel;

	CvCapture *capture = 0;
	IplImage *frame = 0;
	int c;
	cv::VideoWriter* writer;

	bool roc_button = false;
	bool roc_mode = false;
	std::string message = "";

public:


	std::string strMsg = "";

	// 初期化
	void initialize(MyService* p_srv);

	void run();

	// データの更新処理
	void update();

	// ボディフレームの更新
	void updateBodyFrame();

	void draw();

	void drawBodyIndexFrame();
	void KinectApp::sendMessage();
	std::string FloatToString(float x);
	std::string GetStringFromQuaternion(std::string jname, Quaternion q);
	std::string KinectApp::GetStringFromQuaternion_Vectror4(std::string jname, Vector4 q);
	void drawEllipse(cv::Mat& bodyImage, const Joint& joint, int r, const cv::Scalar& color);

	void drawHandState(cv::Mat& bodyImage, Joint joint, TrackingConfidence handConfidence, HandState handState);

	XnPoint3D GetSigVec(int sigvec);

	void RotVec(XnPoint3D &v, Quaternion q);

	Quaternion CalcQuaternion(XnPoint3D kvec, XnPoint3D svec);
	Quaternion CalcQuaternion(XnPoint3D kvec, XnPoint3D svec, double* angle);

	Quaternion convertVector42Quaternion(Vector4 vector4);
	Vector4 convertQuaternion2Vector4(KinectApp::Quaternion quaternion);

	bool DiffVec(XnPoint3D &rvec, XnSkeletonJointPosition jvec, XnSkeletonJointPosition kvec);
	void showVector4(std::string str, Vector4 vector4);
	void showQuaternion(std::string str, KinectApp::Quaternion quaternion);

	Quaternion Kakezan(KinectApp::Quaternion left, KinectApp::Quaternion right);
	Quaternion MakeRotationalQuaternion(double   radian, double AxisX, double AxisY, double AxisZ);
	Quaternion makeNewQuaternion(KinectApp::Quaternion old_quaternion, KinectApp::Quaternion rotation_quaternion);

	void GetSkeletonJointPosition2(XnSkeletonJoint eJoint, XnSkeletonJointPosition& Joint);
	void showXnPoint3D(std::string name, XnPoint3D vector);
	void showXnSkeletonJointPosition(std::string str, XnSkeletonJointPosition point);
	void DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints);
	void DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1);
	std::string KinectApp::strQuaternion(std::string str, KinectApp::Quaternion quaternion);
	void showPosition(std::string strMsg, CameraSpacePoint& point);
	void transQuaternion(KinectApp::Quaternion q, int i, int j, int k);
	KinectApp::Quaternion transQuaternion2(KinectApp::Quaternion q, int i, int j, int k);
	KinectApp::Quaternion MultiQuaternion(KinectApp::Quaternion p, KinectApp::Quaternion q);
	void drawLine(cv::Mat bodyImage, Joint joint1, Joint joint2);
	XnPoint3D convertCameraSpacePoint2Point2XnPoint3D(CameraSpacePoint csp);
	void updateColorFrame();
	void  KinectApp::showJointOrientations(std::string str, JointOrientation jointOrientation);
	void updateInfrared();
	void KinectApp::drawLine(cv::Mat bodyImage, Joint joint1, Joint joint2, cv::Scalar color);
	void KinectApp::drawString(cv::Mat& bodyImage, const Joint& joint, int r, const cv::Scalar& color);
	void KinectApp::BodyToScreen(const CameraSpacePoint& bodyPoint, cv::Point& point, int  width, int height);
};

#endif

