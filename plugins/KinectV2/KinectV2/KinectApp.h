#ifndef _KINECTAPP_
#define _KINECTAPP_
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "SIGService.h"  
#include "Logger.h"
#include "linkage_service.h"
#include <Kinect.h>
#include <opencv2\opencv.hpp>
#include <d2d1.h>

class KinectApp
{
	/*!
	* @brief ERROR_CHECK
	*/
	#define ERROR_CHECK( ret )  \
		if ( (ret) != S_OK ) {    \
			std::stringstream ss;	\
			ss << "failed " #ret " " << std::hex << ret << std::endl;			\
			throw std::runtime_error( ss.str().c_str() );			\
		}

	/*!
	* @brief define order of left and right hand to invert main hand easily later.
	*/
	enum TARGET_HANDS {
		LEFT,
		RIGHT
	};



	/*!
	* @brief map of joint names and their quaternions
	*/
	typedef std::map<std::string, Quaternion> QMap;
private:
	IKinectSensor* kinect = nullptr;
	IBodyFrameReader* bodyFrameReader = nullptr;
	IBody* bodies[6];
	//SIGKINECT_Linkage* srv = nullptr;
	JointOrientation jointorientations[JointType_Count];
	Joint joints[JointType_Count];
	Logger* log;
	//IInfraredFrameReader* infraredFrameReader = nullptr;
	//std::vector<UINT16> infraredBuffer;
	int width = 512;// fixed in Kinect Machine?
	int height = 424;// fixed in Kinect Machine?
	int colorWidth;// fixed in Kinect Machine?
	int colorHeight;// fixed in Kinect Machine?
	IColorFrameReader* colorFrameReader = nullptr;

	std::vector<BYTE> colorBuffer;
	unsigned int colorBytesPerPixel;
	CvCapture* capture = nullptr;
	IplImage* frame = nullptr;
	int c;
	cv::VideoWriter* writer = nullptr;
	bool roc_button = false;
	bool roc_mode = false;
	std::string message = "";
	std::string strMsg = "";
	XnSkeletonJointPosition sfoot, sknee;

	/*!
	* @brief handle the left shoulder and left elbow in grasping
	*/
	XnSkeletonJointPosition ssld, selb, shand;
public:
	void initialize();//initalize
	void run();
	void update();//update Data
	void updateBodyFrame();//update BodyFrame
	void draw();
	void drawBodyIndexFrame();
	//void sendMessage(std::string& agent_name);
	void sendMessage2(sigverse::SIGService *srv, std::string& agent_name);
	void sendMessage(sigverse::SIGService *srv, std::string& agent_name);
	std::string FloatToString(float x);
	std::string GetStringFromQuaternion(std::string jname, Quaternion q);
	std::string GetStringFromQuaternion_Vectror4(std::string jname, Vector4 q);
	void drawEllipse(cv::Mat& bodyImage, const Joint& joint, int r, const cv::Scalar& color);
	void drawHandState(cv::Mat& bodyImage, Joint joint, TrackingConfidence handConfidence, HandState handState);
	XnPoint3D GetSigVec(int sigvec);
	void RotVec(XnPoint3D &v, Quaternion q);
	void RotVec(XnPoint3D &v, Quaternion q, bool axis1);
	Quaternion convertVector42Quaternion(Vector4 vector4);
	Vector4 convertQuaternion2Vector4(Quaternion quaternion);
	bool DiffVec(XnPoint3D &rvec, XnSkeletonJointPosition jvec, XnSkeletonJointPosition kvec);
	void showVector4(std::string str, Vector4 vector4);
	void showQuaternion(std::string str, Quaternion quaternion);
	Quaternion MakeRotationalQuaternion(double   radian, double AxisX, double AxisY, double AxisZ);
	void GetSkeletonJointPosition2(XnSkeletonJoint eJoint, XnSkeletonJointPosition& Joint);
	void showXnPoint3D(std::string name, XnPoint3D vector);
	void showXnSkeletonJointPosition(std::string str, XnSkeletonJointPosition point);
	void DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints);
	void DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1);
	std::string strQuaternion(std::string str, Quaternion quaternion);
	void showPosition(std::string strMsg, CameraSpacePoint& point);
	void transQuaternion(Quaternion q, int i, int j, int k);
	Quaternion transQuaternion2(Quaternion q, int i, int j, int k);
	Quaternion MultiQuaternion(Quaternion p, Quaternion q);
	void drawLine(cv::Mat bodyImage, Joint joint1, Joint joint2);
	XnPoint3D convertCameraSpacePoint2Point2XnPoint3D(CameraSpacePoint csp);
	void updateColorFrame();
	void  showJointOrientations(std::string str, JointOrientation jointOrientation);
	void updateInfrared();
	void drawLine(cv::Mat bodyImage, Joint joint1, Joint joint2, cv::Scalar color);
	void drawString(cv::Mat& bodyImage, const Joint& joint, int r, const cv::Scalar& color);
	void BodyToScreen(const CameraSpacePoint& bodyPoint, cv::Point& point, int  width, int height);
	void copy_joint_position(XnSkeletonJointPosition& s, XnSkeletonJointPosition& t);
	void RotVec_Zaxis(XnPoint3D &v, Quaternion q, bool axis1);
	void RotVec_Xaxis(XnPoint3D &v, Quaternion q, bool axis1);
	void RotVec_Yaxis(XnPoint3D &v, Quaternion q, bool axis1);
	void RotVec_Reverse(XnPoint3D &v, Quaternion q, bool axis1);
	void RotVec_Xaxis_Reverse(XnPoint3D &v, Quaternion q, bool axis1);
	void RotVec_Zaxis_Reverse(XnPoint3D &v, Quaternion q, bool axis1);
	void RotVec_Yaxis_Reverse(XnPoint3D &v, Quaternion q, bool axis1);
	void RotVec_YZaxis_Reverse(XnPoint3D &v, Quaternion q, bool axis1);
	bool DiffVec_Xaxis(XnPoint3D &rvec, XnSkeletonJointPosition jvec, XnSkeletonJointPosition kvec);
	void sendMessage3(sigverse::SIGService *srv, std::string& agent_name);
	void sendMessage4(sigverse::SIGService *srv, std::string& agent_name);

};

#endif

