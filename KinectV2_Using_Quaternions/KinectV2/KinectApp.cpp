/*!
* ==========================================================================================
*  @brief  transmit quatenions of each joints of Kinect_V2　SDK to SIGServer.
*  @file   KinectApp.cpps
*  @date   2015/1/15
*  @author National Institute of Informatics
*  @par    1.0.0
* ==========================================================================================
*/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "KinectApp.h"
#include "ComPtr.h"

xn::Context g_Context;
xn::ScriptNode g_scriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;
xn::Player g_Player;

XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";
XnBool g_bDrawBackground = TRUE;
XnBool g_bDrawPixels = TRUE;
XnBool g_bDrawSkeleton = TRUE;
XnBool g_bPrintID = TRUE;
XnBool g_bPrintState = TRUE;

#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
		    }


#define LOG( message ) { Logger::Write( message ); }


/*!
* @brief positions of left shoulder and left elbow in grasping
*/
XnSkeletonJointPosition ssld, selb, shand;

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


/*!
* @brief define prefixes of left and right hands
*/
char strLabel[50] = "";
XnUserID aUsers[15];
XnUInt16 nUsers = 15;
#ifndef _REVERSE_HAND
const char *GRASP_ARM[2] = { "LARM", "RARM" };
const char *GRASP_LEG[2] = { "LLEG", "RLEG" };
#else
const char *GRASP_ARM[2] = { "RARM", "LARM" };
const char *GRASP_LEG[2] = { "RLEG", "LLEG" };
#endif


//values of joint angles in SIGVerse
XnPoint3D KinectApp::GetSigVec(int sigvec)
{
	XnPoint3D p;
	if (sigvec == WAIST || sigvec == HTOTOR)
	{
		p.X = 0;
		p.Y = 1;
		p.Z = 0;
	}
	else if (sigvec == RSHOULDER || sigvec == RELBOW)
	{
		p.X = -1;
		p.Y = 0;
		p.Z = 0;
	}
	else if (sigvec == HIP || sigvec == LSHOULDER || sigvec == LELBOW)
	{
		p.X = 1;
		p.Y = 0;
		p.Z = 0;
	}
	else if (sigvec == LEG)
	{
		p.X = 0;
		p.Y = -1;
		p.Z = 0;
	}
	else if (sigvec == FOOT)
	{
		p.X = 0;
		//p.Y = -0.7071f;
		//p.Z = +0.7071f;
		p.Y = -0.5;        // 1/2
		p.Z = +0.8660254f; // sqrt(3)/2
	}
	return p;
}

void KinectApp::initialize()
{
	// get default Kinect
	ERROR_CHECK(::GetDefaultKinectSensor(&kinect));

	// open the Kinect
	ERROR_CHECK(kinect->Open());

	BOOLEAN isOpen = false;
	ERROR_CHECK(kinect->get_IsOpen(&isOpen));
	if (!isOpen){
		throw std::runtime_error("Kinect not opened");
	}

	//get color reader
	ComPtr<IColorFrameSource> colorFrameSource;
	ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
	ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

	//get  color image size
	ComPtr<IFrameDescription> colorFrameDescription;
	ERROR_CHECK(colorFrameSource->CreateFrameDescription(
	ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription));
	ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth));
	ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight));
	ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));

	//create buffer
	colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);

	//get body reader
	ComPtr<IBodyFrameSource> bodyFrameSource;
	ERROR_CHECK(kinect->get_BodyFrameSource(&bodyFrameSource));
	ERROR_CHECK(bodyFrameSource->OpenReader(&bodyFrameReader));

	log->Initialize("log.txt");
}

void KinectApp::run()
{
	while (true) {
		update();
		draw();

		auto key = cv::waitKey(10);
		if (key == 'q'){
			break;
		}
	}
}

//update data
void KinectApp::update()
{
	updateColorFrame();
	//updateInfrared();
	updateBodyFrame();
}

//udpate body frame
void KinectApp::updateBodyFrame()
{
	//get frame
	ComPtr<IBodyFrame> bodyFrame;
	auto ret = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
	if (ret == S_OK){
		//get data
		ERROR_CHECK(bodyFrame->GetAndRefreshBodyData(6, &bodies[0]));
		//release frames if you do not use smart pointer.     
		//bodyFrame->Release();
	}
}



void KinectApp::draw()
{
	drawBodyIndexFrame();
}

/*!
* ------------------------------------------------------------------------------------------
* @brief （R×P×Q：backward rotation(forward in SIGVerse)）return 3D vector calculated from rotation quaternion and 3D vector
* @param[in/out] XnPoint3D& input and output source 3D vector
* @param[in]     Quaterion& input rotation quaternion(parent bone:initial line, child bone:radius vector)
* ------------------------------------------------------------------------------------------
*/
void KinectApp::RotVec(XnPoint3D &v, Quaternion q)
{
	double rx = (double)(v.X *  q.qw + v.Y * -q.qz + v.Z *  q.qy);
	double ry = (double)(v.X *  q.qz + v.Y *  q.qw + v.Z * -q.qx);
	double rz = (double)(v.X * -q.qy + v.Y *  q.qx + v.Z *  q.qw);
	double rw = (double)(v.X *  q.qx + v.Y *  q.qy + v.Z *  q.qz);

	v.X = (XnFloat)(q.qx *  rw + q.qy *  rz + q.qz * -ry + q.qw * rx);
	v.Y = (XnFloat)(q.qx * -rz + q.qy *  rw + q.qz *  rx + q.qw * ry);
	v.Z = (XnFloat)(q.qx *  ry + q.qy * -rx + q.qz *  rw + q.qw * rz);
}

/*!
* ------------------------------------------------------------------------------------------
* @brief (R×P×Q：backward rotation(forward in SIGVerse)）return 3D vector calculated from rotation quaternion and 3D vector
* @param[in/out] XnPoint3D& input and output source 3D vector
* @param[in]     Quaterion& input rotation quaternion (parent bone:initial line, child bone:radius vector)
* @param[in]     bool       whether only X axis is valid or not
* ------------------------------------------------------------------------------------------
*/
void KinectApp::RotVec(XnPoint3D &v, Quaternion q, bool axis1) 
{
	//invalidate Y and Z axis if only X axis is valid 
	if (axis1 == true) {
		q.qy = 0.0;
	}

	// （R×P） calculate cross product of vector and quaternion
	float rw = v.X *  q.qx + v.Y *  q.qy + v.Z *  q.qz;
	float rx = v.X *  q.qw + v.Y * -q.qz + v.Z *  q.qy;
	float ry = v.X *  q.qz + v.Y *  q.qw + v.Z * -q.qx;
	float rz = v.X * -q.qy + v.Y *  q.qx + v.Z *  q.qw;

	// （P×Q）calculate the vector rotated by the quaternion
 	v.X = q.qx *  rw + q.qy *  rz + q.qz * -ry + q.qw * rx;
	v.Y = q.qx * -rz + q.qy *  rw + q.qz *  rx + q.qw * ry;
	v.Z = q.qx *  ry + q.qy * -rx + q.qz *  rw + q.qw * rz;

}


//update color frame
void KinectApp::updateColorFrame()
{
	//printf("updateColorFrame\n");
	//get frame
	ComPtr<IColorFrame> colorFrame;
	auto ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
	if (ret == S_OK){
		//get data as BGRA format
		ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(
			colorBuffer.size(), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra));
	}
}

/**
void KinectApp::updateInfrared()
{
printf("updateInfrared\n");
ComPtr<IInfraredFrame> infraredFrame;
auto ret = infraredFrameReader->AcquireLatestFrame(&infraredFrame);
if (ret == S_OK){
ERROR_CHECK(infraredFrame->CopyFrameDataToArray(infraredBuffer.size(), &infraredBuffer[0]));
// infraredFrame->Release();
}
}
*/

void KinectApp::drawBodyIndexFrame()
{
	//display joint coordinates in Depth coordinate system.
	//take care of inversion between height and width
	//1920,1080 > 512,484(1024,968)
	cv::Mat colorImage(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
	cv::Mat bodyImage(width, height, CV_8UC4);
	cv::Mat cut_img(colorImage, cv::Rect(448, 56, 1472, 912));
	//cv::Mat cut_img(colorImage, cv::Rect(0, 0, 1024, 968));
	//cv::Mat cut_img(colorImage, cv::Rect(896, 112, 1920, 1080));
	cv::resize(colorImage, bodyImage, cv::Size(width, height), 0, 0, 1);

	for (auto body : bodies){
		if (body == nullptr){
			continue;
		}

		BOOLEAN isTracked = false;
		ERROR_CHECK(body->get_IsTracked(&isTracked));
		if (!isTracked) {
			continue;
		}

		system("cls");
		body->GetJointOrientations(JointType_Count, jointorientations);
		body->GetJoints(JointType::JointType_Count, joints);
		drawLine(bodyImage, joints[3], joints[2], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[2], joints[20], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[20], joints[8], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[20], joints[4], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[20], joints[1], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[8], joints[9], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[4], joints[5], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[9], joints[10], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[5], joints[6], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[10], joints[11], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[6], joints[7], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[11], joints[23], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[7], joints[21], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[10], joints[24], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[6], joints[22], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[1], joints[0], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[0], joints[16], cv::Scalar(0, 200, 0));//
		drawLine(bodyImage, joints[0], joints[12], cv::Scalar(0, 200, 0));//
		drawLine(bodyImage, joints[16], joints[17], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[12], joints[13], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[17], joints[18], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[13], joints[14], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[18], joints[19], cv::Scalar(0, 0, 200));
		drawLine(bodyImage, joints[14], joints[15], cv::Scalar(0, 0, 200));
		//showAllQuaternions("joints",joints);
		for (auto joint : joints)
		{
			//tracking joint
			if (joint.TrackingState == TrackingState::TrackingState_Tracked) {
				if (joint.JointType == JointType_HipRight)
				{
					drawEllipse(bodyImage, joint, 10, cv::Scalar(0, 255, 0));
				}
				else if (joint.JointType == JointType_HipLeft)
				{
					drawEllipse(bodyImage, joint, 10, cv::Scalar(255, 0, 0));
				}
				else
				{
					drawEllipse(bodyImage, joint, 10, cv::Scalar(255, 0, 0));
				}

				//when tracking left hand
				if (joint.JointType == JointType::JointType_HandLeft) {
					HandState handState;
					TrackingConfidence handConfidence;
					body->get_HandLeftState(&handState);
					body->get_HandLeftConfidence(&handConfidence);

					drawHandState(bodyImage, joint, handConfidence, handState);
				}
				//when tracking right hand
				else if (joint.JointType == JointType::JointType_HandRight) {
					HandState handState;
					TrackingConfidence handConfidence;
					body->get_HandRightState(&handState);
					body->get_HandRightConfidence(&handConfidence);

					drawHandState(bodyImage, joint, handConfidence, handState);
				}
			}
			//guess status
			else if (joint.TrackingState == TrackingState::TrackingState_Inferred) {
				drawEllipse(bodyImage, joint, 10, cv::Scalar(255, 255, 0));
			}
		}

	}
	cv::flip(bodyImage, bodyImage, 1); //flip horizontal
	cv::putText(bodyImage, message, cv::Point((int)50, (int)50), CV_FONT_HERSHEY_SIMPLEX, 0.2, cv::Scalar(0, 0, 200), 1, CV_AA);
	drawString(bodyImage, joints[0], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[1], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[2], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[3], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[4], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[5], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[6], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[7], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[8], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[9], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[10], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[11], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[12], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[13], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[14], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[15], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[16], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[17], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[18], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[19], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[20], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[21], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[22], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[23], 10, cv::Scalar(255, 255, 0));
	drawString(bodyImage, joints[24], 10, cv::Scalar(255, 255, 0));
	cv::imshow("Body Image", bodyImage);
}


void KinectApp::sendMessage4(sigverse::SIGService *srv, std::string& agent_name)
{
	static bool bInitialized = false;
	//static GLuint depthTexID;
	//static unsigned char* pDepthTexBuf;
	//static int texWidth, texHeight;

	//added for sigverse
	static char saddr[128];
	static int port;
	static int speed;
	static double move_speed;
	static int cnt;
	static bool start;
	static XnPoint3D startpos;

	//float topLeftX;
	//float topLeftY;
	//float bottomRightY;
	//float bottomRightX;
	//float texXpos;
	//float texYpos;

	if (!bInitialized)
	{
		start = false;

		TCHAR spd[8];
		GetPrivateProfileString(_T("SETTING"), _T("SEND_SPEED"), '\0', spd, 256, _T("./SIGNiUserTracker.ini"));
		if (spd[0] == '\0') {
			speed = 1;
		}
		else {
			speed = atoi((char*)spd);
		}

		GetPrivateProfileString(_T("SETTING"), _T("MOVE_SPEED"), '\0', spd, 256, _T("./SIGNiUserTracker.ini"));
		if (spd[0] == '\0') {
			move_speed = 1.0;
		}
		else {
			move_speed = atof((char*)spd);
		}

		bInitialized = true;
	}




	//i : humanbody index 
	int i = 0;
	if (i == 0){
		if (cnt < 0) cnt = speed;
		if (cnt == 0){
			XnSkeletonJointPosition SpineBase, torso, lhip, rhip, neck, lshoul,
				rshoul, lelb, relb, lknee, rknee, lhand, rhand, lfingertip, rfingertip, lankle, rankle, lfoot, rfoot, head;
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_TORSO, torso);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_LEFT_HIP, lhip);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_RIGHT_HIP, rhip);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_NECK, neck);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_HEAD, head);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_LEFT_SHOULDER, lshoul);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_RIGHT_SHOULDER, rshoul);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_LEFT_ELBOW, lelb);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_RIGHT_ELBOW, relb);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_LEFT_HAND, lhand);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_RIGHT_HAND, rhand);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_LEFT_KNEE, lknee);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_RIGHT_KNEE, rknee);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_LEFT_FOOT, lfoot);
			//g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_RIGHT_FOOT, rfoot);

			GetSkeletonJointPosition2(XN_SKEL_SPINEBASE, SpineBase);
			GetSkeletonJointPosition2(XN_SKEL_TORSO, torso);
			GetSkeletonJointPosition2(XN_SKEL_LEFT_HIP, lhip);
			GetSkeletonJointPosition2(XN_SKEL_RIGHT_HIP, rhip);
			GetSkeletonJointPosition2(XN_SKEL_NECK, neck);
			GetSkeletonJointPosition2(XN_SKEL_HEAD, head);
			GetSkeletonJointPosition2(XN_SKEL_LEFT_SHOULDER, lshoul);
			GetSkeletonJointPosition2(XN_SKEL_RIGHT_SHOULDER, rshoul);
			GetSkeletonJointPosition2(XN_SKEL_LEFT_ELBOW, lelb);
			GetSkeletonJointPosition2(XN_SKEL_RIGHT_ELBOW, relb);
			GetSkeletonJointPosition2(XN_SKEL_LEFT_HAND, lhand);
			GetSkeletonJointPosition2(XN_SKEL_RIGHT_HAND, rhand);
			GetSkeletonJointPosition2(XN_SKEL_LEFT_KNEE, lknee);
			GetSkeletonJointPosition2(XN_SKEL_RIGHT_KNEE, rknee);
			GetSkeletonJointPosition2(XN_SKEL_LEFT_FOOT, lfoot);
			GetSkeletonJointPosition2(XN_SKEL_RIGHT_FOOT, rfoot);

			GetSkeletonJointPosition2(XN_SKEL_LEFT_FINGERTIP, lfingertip);
			GetSkeletonJointPosition2(XN_SKEL_RIGHT_FINGERTIP, rfingertip);

			GetSkeletonJointPosition2(XN_SKEL_LEFT_KNEE, lknee);
			GetSkeletonJointPosition2(XN_SKEL_RIGHT_KNEE, rknee);
			GetSkeletonJointPosition2(XN_SKEL_LEFT_ANKLE, lankle);
			GetSkeletonJointPosition2(XN_SKEL_RIGHT_ANKLE, rankle);
			GetSkeletonJointPosition2(XN_SKEL_LEFT_FOOT, lfoot);
			GetSkeletonJointPosition2(XN_SKEL_RIGHT_FOOT, rfoot);

			if (start == false && SpineBase.position.Z != 0)
			{
				startpos.X = SpineBase.position.X;
				startpos.Y = SpineBase.position.Y;
				startpos.Z = SpineBase.position.Z;
				start = true;
			}

			XnPoint3D pos;
			pos.X = -(SpineBase.position.X - startpos.X);
			pos.Y = SpineBase.position.Y - startpos.Y;
			pos.Z = -(SpineBase.position.Z - startpos.Z);

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			QMap mq; //Map of joint names and its quaternions 
			 
			Quaternion q0 = convertVector42Quaternion(jointorientations[0].Orientation);
			Quaternion q1 = convertVector42Quaternion(jointorientations[1].Orientation);
			Quaternion q1_h = Q_val(0, 0, 1, 0);
			Quaternion q1_rot = MultiQuaternion(q1_h, q1);
			Quaternion q2 = convertVector42Quaternion(jointorientations[20].Orientation);
			Quaternion q2_h = Q_val(0, 0, -1, 0);
			Quaternion q2_rot = MultiQuaternion(q2, q2_h);
			
				
			Quaternion q3 = convertVector42Quaternion(jointorientations[8].Orientation);
			Quaternion q4 = convertVector42Quaternion(jointorientations[9].Orientation);
			Quaternion q7 = convertVector42Quaternion(jointorientations[5].Orientation);
			Quaternion q9 = convertVector42Quaternion(jointorientations[10].Orientation);
			Quaternion q13 = convertVector42Quaternion(jointorientations[17].Orientation);
			Quaternion q15 = convertVector42Quaternion(jointorientations[13].Orientation);
			Quaternion q17 = convertVector42Quaternion(jointorientations[18].Orientation);
			Quaternion q19 = convertVector42Quaternion(jointorientations[14].Orientation);
			Quaternion q20 = convertVector42Quaternion(jointorientations[20].Orientation);
			Quaternion q20_rot = MultiQuaternion(q1_h, q20);
			Quaternion q21 = convertVector42Quaternion(jointorientations[6].Orientation);
			Quaternion q25 = convertVector42Quaternion(jointorientations[11].Orientation);
			Quaternion q26 = convertVector42Quaternion(jointorientations[7].Orientation);
			Quaternion q1_con = Q_val(q20_rot.qw, -q20_rot.qx, -q20_rot.qy, -q20_rot.qz);
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////

			///////////////////////////  Right Arm quaternion //////////////////////
			Quaternion q5;
			Quaternion q5_x = Q_val(0.7071,-0.7071, 0, 0);
			Quaternion q5_z = Q_val(0.7071,0, 0, -0.7071);
			Quaternion q5_y = Q_val(0.7071, 0, -0.7071,0 );
			Quaternion q5_rotx = MultiQuaternion(q5_x, q4);
			q5.qw = q5_rotx.qw;
			q5.qx = q5_rotx.qy;
			q5.qy = q5_rotx.qz;
			q5.qz = q5_rotx.qx;
			q5 = MultiQuaternion(q2_h, q5);
	
			q5 = MultiQuaternion(q5_y, q5);
			q5.qw = q5.qw;
			q5.qx = -q5.qx;
			q5.qy = -q5.qy;
			q5.qz = q5.qz;
			q5 = MultiQuaternion(q1_con, q5);
			//////////////////////////////////////////////////////////////////////

			////////////// Left Arm quaternion ///////////////////////////////////
			Quaternion q6;
			Quaternion q6_x = Q_val(0.7071, 0.7071, 0, 0);
			Quaternion q6_z = Q_val(0.7071, 0, 0, 0.7071);
			Quaternion q6_y = Q_val(0.7071, 0, 0.7071, 0);
			Quaternion q6_rotx = MultiQuaternion(q6_x, q7);
			q6.qw = q6_rotx.qw;
			q6.qx = q6_rotx.qy;
			q6.qy = q6_rotx.qz;
			q6.qz = q6_rotx.qx;
			q6= MultiQuaternion(q2_h, q6);
			q6 = MultiQuaternion(q6_y, q6);
			q6.qw = q6.qw;
			q6.qx = q6.qx;
			q6.qy = q6.qy;
			q6.qz = q6.qz;
			q6 = MultiQuaternion(q1_con, q6);
			
           //////////////////////////////////////////////////////////////////////////

	    	///////////////////////////  Right hipe quaternion //////////////////////
			Quaternion q12;
			Quaternion q12_x = Q_val(0.7071, -0.7071, 0, 0);
			Quaternion q12_z = Q_val(0.7071, 0, 0, -0.7071);
			Quaternion q12_y = Q_val(0.7071, 0, 0.7071, 0);
			Quaternion q12_rotx = MultiQuaternion(q12_x, q13);
			Quaternion q3_h = Q_val(0, 0, 1, 0);
			Quaternion q12_h = Q_val(0, 0, 0, -1);
			q12 = MultiQuaternion(q12_h, q13);
			Quaternion q12_b = MultiQuaternion(q12_y, q12);
			q12.qw = q12_b.qw;
			q12.qx = -q12_b.qz;
			q12.qy = -q12_b.qy;
			q12.qz =- q12_b.qx;
			//////////////////////////////////////////////////////////////////////

			///////////////////////////  Right leg quaternion //////////////////////
			Quaternion q16;
			Quaternion q16_x = Q_val(0.7071, -0.7071, 0, 0);
			Quaternion q16_z = Q_val(0.7071, 0, 0, -0.7071);
			Quaternion q16_y = Q_val(0.7071, 0, 0.7071, 0);
			Quaternion q16_rotx = MultiQuaternion(q16_x, q17);
			Quaternion q16_h = Q_val(0, 0, 0, -1);
		 	q16 = MultiQuaternion(q16_h, q17);
			Quaternion q16_b = MultiQuaternion(q16_y, q16);
			q16.qw = q16_b.qw;
			q16.qx = -q16_b.qz;
			q16.qy = -q16_b.qy;
			q16.qz = -q16_b.qx;

			Quaternion  q12_con;
			q12_con.qw = q12.qw;
			q12_con.qx = -q12.qx;
			q12_con.qy = -q12.qy;
			q12_con.qz = -q12.qz;
			q16 = MultiQuaternion(q12_con, q16);
			//////////////////////////////////////////////////////////////////////


			///////////////////////////  left hipe quaternion //////////////////////
			Quaternion q14;
			Quaternion q14_x = Q_val(0.7071, -0.7071, 0, 0);
			Quaternion q14_z = Q_val(0.7071, 0, 0, -0.7071);
			Quaternion q14_y = Q_val(0.7071, 0, -0.7071, 0);
			Quaternion q14_rotx = MultiQuaternion(q14_x, q15);

			Quaternion q14_h = Q_val(0, 0, 0, 1);
			q14 = MultiQuaternion(q14_h, q15);
			Quaternion q14_b = MultiQuaternion(q14_y, q14);
			q14.qw = q14_b.qw;
			q14.qx = q14_b.qz;
			q14.qy = -q14_b.qy;
			q14.qz = q14_b.qx;
			//q14 = MultiQuaternion(q2_h, q14);
			//////////////////////////////////////////////////////////////////////


			///////////////////////////  left leg quaternion //////////////////////
			Quaternion q18;
			Quaternion q18_x = Q_val(0.7071, -0.7071, 0, 0);
			Quaternion q18_z = Q_val(0.7071, 0, 0, -0.7071);
			Quaternion q18_y = Q_val(0.7071, 0, -0.7071, 0);
			Quaternion q18_rotx = MultiQuaternion(q18_x, q19);
			Quaternion q18_h = Q_val(0, 0, 0, 1);
			q18 = MultiQuaternion(q18_h, q19);
			Quaternion q18_b = MultiQuaternion(q18_y, q18);
			q18.qw = q18_b.qw;
			q18.qx = q18_b.qz;
			q18.qy = -q18_b.qy;
			q18.qz = q18_b.qx;

			Quaternion  q14_con;
			q14_con.qw = q14.qw;
			q14_con.qx = -q14.qx;
			q14_con.qy = -q14.qy;
			q14_con.qz = -q14.qz;
			q18 = MultiQuaternion(q14_con, q18);
			//////////////////////////////////////////////////////////////////////


			////////////// Right for Arm  quaternion /////////////////////////////
			Quaternion q8;
			Quaternion q8_x = Q_val(0.7071, -0.7071, 0, 0);
			Quaternion q8_z = Q_val(0.7071, 0, 0, 0.7071);
			Quaternion q8_y = Q_val(0.7071, 0, 0.7071, 0);
			Quaternion q8_rotx = MultiQuaternion(q8_x, q9);

			q8.qw = q8_rotx.qw;
			q8.qx = q8_rotx.qy;
			q8.qy = q8_rotx.qz;
			q8.qz = q8_rotx.qx;
			q8 = MultiQuaternion(q8_y, q8);
			Quaternion q8_xx = Q_val(0, 1, 0, 0);
			Quaternion q8_zz = Q_val(0, 0, 0, 1);
			Quaternion  q8_va = Q_val(q8.qw, q8.qx, q8.qy, q8.qz);
			
			q8 = MultiQuaternion(q2_h, q8);
			q8.qw = q8_va.qw;
			q8.qx = -q8_va.qx;
			q8.qy = -q8_va.qy;
			q8.qz = q8_va.qz;

			Quaternion q5_con;
			q5_con.qw = q5.qw;
			q5_con.qx = -q5.qx;
			q5_con.qy = -q5.qy;
			q5_con.qz = -q5.qz;
			q8 = MultiQuaternion(q5_con, q8);
			////////////////////////////////////////////////////////////////////////

			////////////// Left for Arm  quaternion ////////////////////////////////
			Quaternion q22;
			Quaternion q22_x = Q_val(0.7071, -0.7071, 0, 0);
			Quaternion q22_z = Q_val(0.7071, 0, 0, 0.7071);
			Quaternion q22_y = Q_val(0.7071, 0, -0.7071, 0);
			Quaternion q22_rotx = MultiQuaternion(q22_x, q21);

			q22.qw = q22_rotx.qw;
			q22.qx = q22_rotx.qy;
			q22.qy = q22_rotx.qz;
			q22.qz = q22_rotx.qx;
			q22 = MultiQuaternion(q22_y, q22);

			Quaternion q22_xx = Q_val(0, 1, 0, 0);
			Quaternion q22_zz = Q_val(0, 0, 0, 1);
			Quaternion  q22_va = Q_val(q22.qw, q22.qx, q22.qy, q22.qz);

			q22 = MultiQuaternion(q2_h, q8);
			q22.qw = q22_va.qw;
			q22.qx = q22_va.qx;
			q22.qy = -q22_va.qy;
			q22.qz = -q22_va.qz;

			Quaternion q6_con;
			q6_con.qw = q6.qw;
			q6_con.qx = -q6.qx;
			q6_con.qy = -q6.qy;
			q6_con.qz = -q6.qz;
			q22 = MultiQuaternion(q6_con, q22);

			//////////////////////////////////////////////////////////////////////////


			
			////////////// Right hand  quaternion ////////////////////////////////////
			Quaternion q27;
			Quaternion q27_x = Q_val(0.7071, -0.7071, 0, 0);
			Quaternion q27_z = Q_val(0.7071, 0, 0, 0.7071);
			Quaternion q27_y = Q_val(0.7071, 0, -0.7071, 0);
			Quaternion q27_rotx = MultiQuaternion(q27_x,q25);
			q27.qw = q27_rotx.qw;
			q27.qx = q27_rotx.qy;
			q27.qy = q27_rotx.qz;
			q27.qz = q27_rotx.qx;
			Quaternion q27_xx = Q_val(0, 1, 0, 0);
			Quaternion q27_zz = Q_val(0, 0, 0, 1);
			Quaternion  q27_va = Q_val(q27.qw, q27.qx, q27.qy, q27.qz);
			q27 = MultiQuaternion(q2_h, q27);
			q27.qw = q27_va.qw;
			q27.qx = q27_va.qx;
			q27.qy = q27_va.qy;
			q27.qz = q27_va.qz;
			Quaternion q8_con;
			q8_con.qw = q8.qw;
			q8_con.qx = -q8.qx;
			q8_con.qy = -q8.qy;
			q8_con.qz = -q8.qz;
			q27 = MultiQuaternion(q8_con, q27);
			//////////////////////////////////////////////////////////////////////////


			////////////// Left Hand  quaternion ///////////////////////////////////
			Quaternion q28;
			Quaternion q28_x = Q_val(0.7071, -0.7071, 0, 0);
			Quaternion q28_z = Q_val(0.7071, 0, 0, 0.7071);
			Quaternion q28_y = Q_val(0.7071, 0, -0.7071, 0);
			Quaternion q28_rotx = MultiQuaternion(q28_x, q26);

			q28.qw = q28_rotx.qw;
			q28.qx = q28_rotx.qy;
			q28.qy = q28_rotx.qz;
			q28.qz = q28_rotx.qx;
			q28 = MultiQuaternion(q28_y, q28);
			Quaternion q28_xx = Q_val(0, 1, 0, 0);
			Quaternion q28_zz = Q_val(0, 0, 0, 1);
			Quaternion  q28_va = Q_val(q28.qw, q28.qx, q28.qy, q28.qz);
			q28 = MultiQuaternion(q2_h, q28);
			q28.qw = q28_va.qw;
			q28.qx = q28_va.qx;
			q28.qy = -q28_va.qy;
			q28.qz = -q28_va.qz;
			Quaternion q22_con;
			q22_con.qw = q22.qw;
			q22_con.qx = -q22.qx;
			q22_con.qy = -q22.qy;
			q22_con.qz = -q22.qz;
			q28 = MultiQuaternion(q22_con, q28);

			//////////////////////////////////////////////////////////////////////////




			           // showQuaternion("Q_8", q8);
					    mq.insert(QMap::value_type("WAIST_JOINT1", q1_rot));
						mq.insert(QMap::value_type("RARM_JOINT2", q5));
						mq.insert(QMap::value_type("LARM_JOINT2", q6));
						mq.insert(QMap::value_type("RLEG_JOINT2", q12));
						mq.insert(QMap::value_type("LLEG_JOINT2", q14));
						mq.insert(QMap::value_type("RLEG_JOINT4", q16));
						mq.insert(QMap::value_type("LLEG_JOINT4", q18));
						mq.insert(QMap::value_type("RARM_JOINT3", q8));
						mq.insert(QMap::value_type("LARM_JOINT3", q22));
						mq.insert(QMap::value_type("RARM_JOINT5", q27));
						mq.insert(QMap::value_type("LARM_JOINT5", q28));
					
					
				int connectedNum = srv->getConnectedControllerNum();

				if (connectedNum > 0){

					std::string posx = FloatToString((100 * pos.X*(float)move_speed));
					std::string posy = FloatToString((100 * pos.Y*(float)move_speed));
					std::string posz = FloatToString((100 * pos.Z*(float)move_speed));
					std::string pos_ = "POSITION";
					pos_ += ":" + posx + "," + posy + "," + posz;


					std::string st = "KINECT_DATA ";
					st += pos_;
					QMap::iterator it = mq.begin();

					while (it != mq.end())
					{
						st += " ";
						st += (GetStringFromQuaternion((*it).first.c_str(), (*it).second));
						it++;
					}
					st += " END:";
					std::vector<std::string> names = srv->getAllConnectedEntitiesName();
					for (int i = 0; i < connectedNum; i++){
						srv->sendMsgToCtr(names[i].c_str(), st);
					}
				}

			} // if(DiffVec(
			cnt = speed;
		} //if(cnt == 0)
		cnt--;
	}



void KinectApp::showVector4(std::string str, Vector4 vector4)
{
	printf("%s:\t", str.c_str());
	printf("w=%f\t", vector4.w);
	printf("x=%f\t", vector4.x);
	printf("y=%f\t", vector4.y);
	printf("z=%f\n", vector4.z);
}

void KinectApp::showXnSkeletonJointPosition(std::string str, XnSkeletonJointPosition point)
{
	printf("%s:", str.c_str());
	//printf("fconfidence=%f\t", point.fConfidence);
	printf("x=%f\t", point.position.X);//right for man:+ left    for man:-  
	printf("y=%f\t", point.position.Y);//down  for man:+ up      for man:-  
	printf("z=%f\t\n", point.position.Z);//back  for man:+ forward for man:-   
}



void KinectApp::showQuaternion(std::string str, Quaternion quaternion){
	double angle = acos(quaternion.qw) * 2 * 57.29578;
	char buf[256];
	sprintf_s(buf, "%s:\tangle=%f\nw=%f\tx=%f\ty=%f\tz=%f\n", str.c_str(), angle, quaternion.qw, quaternion.qx, quaternion.qy, quaternion.qz);
	printf("%s", buf);

}

std::string KinectApp::strQuaternion(std::string str, Quaternion quaternion){
	double angle = acos(quaternion.qw) * 2 * 57.29578;
	char buf[256];
	sprintf_s(buf, "%s:%f,%f,%f,%f ", str.c_str(), quaternion.qw, quaternion.qx, quaternion.qy, quaternion.qz);
	std::string strMsg = buf;
	return strMsg;
}

std::string KinectApp::FloatToString(float x)
{
	char tmp[32];
	sprintf_s(tmp, "%f", x);
	std::string str;
	str = std::string(tmp);
	return str;
}

std::string KinectApp::GetStringFromQuaternion(std::string jname, Quaternion q)
{
	std::string qw = FloatToString(q.qw);
	std::string qx = FloatToString(q.qx);
	std::string qy = FloatToString(q.qy);
	std::string qz = FloatToString(q.qz);
	std::string s = jname + ":" + qw + "," + qx + "," + qy + "," + qz;

	return s;
}
Quaternion KinectApp::Q_val(float w, float x, float y, float z)
{
	Quaternion quaternion;
	quaternion.qw = w;
	quaternion.qx = x;
	quaternion.qy = y;
	quaternion.qz = z;
	return quaternion;
}


std::string KinectApp::GetStringFromQuaternion_Vectror4(std::string jname, Vector4 q)
{
	std::string qw = FloatToString(q.w);
	std::string qx = FloatToString(q.x);
	std::string qy = FloatToString(q.y);
	std::string qz = FloatToString(q.z);
	std::string s = jname + ":" + qw + "," + qx + "," + qy + "," + qz;

	return s;
}

void KinectApp::drawLine(cv::Mat bodyImage, Joint joint1, Joint joint2, cv::Scalar color)
{

	cv::Point point1;
	BodyToScreen(joint1.Position, point1, width, height);
	cv::Point point2;
	BodyToScreen(joint2.Position, point2, width, height);
	cv::line(bodyImage, point1, point2, cv::Scalar(0, 0, 200), 20, 8, 0);
}


//http://wagashi1349.hatenablog.com/entry/2014/08/14/003618
void KinectApp::BodyToScreen(const CameraSpacePoint& bodyPoint, cv::Point& point, int  width, int height)
{

	ComPtr<ICoordinateMapper> mapper;
	ERROR_CHECK(kinect->get_CoordinateMapper(&mapper));
	DepthSpacePoint depthPoint = { 0 };
	mapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);
	point.x = (int)((depthPoint.X * width) / 512);
	point.y = (int)((depthPoint.Y * height) / 424);

}

void KinectApp::drawEllipse(cv::Mat& bodyImage, const Joint& joint, int r, const cv::Scalar& color)
{

	cv::Point point;
	BodyToScreen(joint.Position, point, width, height);
	cv::circle(bodyImage, point, r, color, -1);
	char buf[128];
	sprintf_s(buf, "%d", joint.JointType);

}

void KinectApp::drawHandState(cv::Mat& bodyImage, Joint joint, TrackingConfidence handConfidence, HandState handState)
{
	const int R = 40;

	if (handConfidence != TrackingConfidence::TrackingConfidence_High){
		return;
	}

	//translate camera coordinate system into Depth coordinate system
	//ComPtr<ICoordinateMapper> mapper;
	//ERROR_CHECK(kinect->get_CoordinateMapper(&mapper));

	//DepthSpacePoint point;
	//mapper->MapCameraPointToDepthSpace(joint.Position, &point);
	cv::Point point;
	BodyToScreen(joint.Position, point, width, height);

	//open
	if (handState == HandState::HandState_Open){
		cv::circle(bodyImage, point, R, cv::Scalar(0, 255, 255), R / 4);
	}
	//scissors
	else if (handState == HandState::HandState_Lasso){
		cv::circle(bodyImage, point, R, cv::Scalar(255, 0, 255), R / 4);
	}
	//close
	else if (handState == HandState::HandState_Closed){
		cv::circle(bodyImage, point, R, cv::Scalar(255, 255, 0), R / 4);
	}
}

/**
Quaternion KinectApp::CalcQuaternion(XnPoint3D kvec, XnPoint3D svec)
{
	Quaternion q;
	if (kvec.X == svec.X && kvec.Y == svec.Y && kvec.Z == svec.Z)
	{
		q.qw = 1; q.qx = 0; q.qy = 0; q.qz = 0;
		return q;
	}


	double x = kvec.Y*svec.Z - kvec.Z*svec.Y;
	double y = kvec.Z*svec.X - kvec.X*svec.Z;
	double z = kvec.X*svec.Y - kvec.Y*svec.X;

	double sum = sqrt(x*x + y*y + z*z);
	x = x / sum;
	y = y / sum;
	z = z / sum;

	double angle = acos(kvec.X*svec.X + kvec.Y*svec.Y + kvec.Z*svec.Z);
	q.qw = (float)(cos(angle / 2));
	q.qx = (float)(x*sin(angle / 2));
	q.qy = (float)(y*sin(angle / 2));
	q.qz = (float)(z*sin(angle / 2));

	return q;
}
*/


Vector4 KinectApp::convertQuaternion2Vector4(Quaternion quaternion)
{
	Vector4 vector4;
	vector4.w = quaternion.qw;
	vector4.x = quaternion.qx;
	vector4.y = quaternion.qy;
	vector4.z = quaternion.qz;
	return vector4;
}

/**
*[out] XnPoint3D &rvec: relative position info with direction
*[in] XnSkeletonJointPosition jvec: parent joint position info with confidence
*[in] XnSkeletonJointPosition kvec: child joint  position info with confidence
*/
bool KinectApp::DiffVec(XnPoint3D &rvec, XnSkeletonJointPosition jvec, XnSkeletonJointPosition kvec)
{
	if (jvec.fConfidence < 0.5 || kvec.fConfidence < 0.5)
	{
		return false;
	}

	//in SIGVerse:
	//left for man:+
	//up for man:+
	//forward for man:+
	rvec.X = -(kvec.position.X - jvec.position.X);
	rvec.Y = kvec.position.Y - jvec.position.Y;
	rvec.Z = -(kvec.position.Z - jvec.position.Z);

	double length = sqrt(rvec.X*rvec.X + rvec.Y*rvec.Y + rvec.Z*rvec.Z);
	rvec.X = (XnFloat)(rvec.X / length);
	rvec.Y = (XnFloat)(rvec.Y / length);
	rvec.Z = (XnFloat)(rvec.Z / length);
	return true;
}

//// Make Rotational quaternion      
Quaternion KinectApp::MakeRotationalQuaternion(double   radian, double AxisX, double AxisY, double AxisZ)
{
	Quaternion   ans;
	double   norm;
	double   ccc, sss;

	ans.qw = ans.qx = ans.qy = ans.qz = 0.0;

	norm = AxisX *  AxisX + AxisY *  AxisY + AxisZ *  AxisZ;
	if (norm <= 0.0) return ans;

	norm = 1.0 / sqrt(norm);
	AxisX *= norm;
	AxisY *= norm;
	AxisZ *= norm;

	ccc = cos(0.5 * radian);
	sss = sin(0.5 * radian);

	ans.qw = (float)(ccc);
	ans.qx = (float)(sss * AxisX);
	ans.qy = (float)(sss * AxisY);
	ans.qz = (float)(sss * AxisZ);

	return   ans;
}

void KinectApp::showXnPoint3D(std::string name, XnPoint3D vector){
	printf("%s: x=%f, y=%f, z=%f\n", name.c_str(), vector.X, vector.Y, vector.Z);
}

void KinectApp::GetSkeletonJointPosition2(XnSkeletonJoint eJoint, XnSkeletonJointPosition& Joint){
	switch (eJoint)
	{
	case XN_SKEL_HEAD:
		if (joints[3].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[3].Position.X;
		Joint.position.Y = joints[3].Position.Y;
		Joint.position.Z = joints[3].Position.Z;
		break;
	case XN_SKEL_NECK:
		if (joints[2].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[2].Position.X;
		Joint.position.Y = joints[2].Position.Y;
		Joint.position.Z = joints[2].Position.Z;
		break;
	case XN_SKEL_SPINEBASE:
		if (joints[0].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[0].Position.X;
		Joint.position.Y = joints[0].Position.Y;
		Joint.position.Z = joints[0].Position.Z;
		break;
	case XN_SKEL_TORSO:
		if (joints[1].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[1].Position.X;
		Joint.position.Y = joints[1].Position.Y;
		Joint.position.Z = joints[1].Position.Z;
		break;
	case XN_SKEL_WAIST:
		if (joints[0].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[0].Position.X;
		Joint.position.Y = joints[0].Position.Y;
		Joint.position.Z = joints[0].Position.Z;
		break;
	case XN_SKEL_LEFT_COLLAR:
		if (joints[20].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[20].Position.X;
		Joint.position.Y = joints[20].Position.Y;
		Joint.position.Z = joints[20].Position.Z;
		break;
	case XN_SKEL_LEFT_SHOULDER:
		if (joints[4].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[4].Position.X;
		Joint.position.Y = joints[4].Position.Y;
		Joint.position.Z = joints[4].Position.Z;
		break;
	case XN_SKEL_LEFT_ELBOW:
		if (joints[5].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[5].Position.X;
		Joint.position.Y = joints[5].Position.Y;
		Joint.position.Z = joints[5].Position.Z;
		break;
	case XN_SKEL_LEFT_WRIST:
		if (joints[6].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[6].Position.X;
		Joint.position.Y = joints[6].Position.Y;
		Joint.position.Z = joints[6].Position.Z;
		break;
	case XN_SKEL_LEFT_HAND:
		if (joints[7].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[7].Position.X;
		Joint.position.Y = joints[7].Position.Y;
		Joint.position.Z = joints[7].Position.Z;
		break;
	case XN_SKEL_LEFT_FINGERTIP:
		if (joints[21].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[21].Position.X;
		Joint.position.Y = joints[21].Position.Y;
		Joint.position.Z = joints[21].Position.Z;
		break;
	case XN_SKEL_RIGHT_COLLAR:
		if (joints[20].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[20].Position.X;
		Joint.position.Y = joints[20].Position.Y;
		Joint.position.Z = joints[20].Position.Z;
		break;
	case XN_SKEL_RIGHT_SHOULDER:
		if (joints[8].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[8].Position.X;
		Joint.position.Y = joints[8].Position.Y;
		Joint.position.Z = joints[8].Position.Z;
		break;
	case XN_SKEL_RIGHT_ELBOW:
		if (joints[9].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[9].Position.X;
		Joint.position.Y = joints[9].Position.Y;
		Joint.position.Z = joints[9].Position.Z;
		break;
	case XN_SKEL_RIGHT_WRIST:
		if (joints[10].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[10].Position.X;
		Joint.position.Y = joints[10].Position.Y;
		Joint.position.Z = joints[10].Position.Z;
		break;
	case XN_SKEL_RIGHT_HAND:
		if (joints[11].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[11].Position.X;
		Joint.position.Y = joints[11].Position.Y;
		Joint.position.Z = joints[11].Position.Z;
		break;
	case XN_SKEL_RIGHT_FINGERTIP:
		if (joints[23].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[23].Position.X;
		Joint.position.Y = joints[23].Position.Y;
		Joint.position.Z = joints[23].Position.Z;
		break;
	case XN_SKEL_LEFT_HIP:
		if (joints[12].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[12].Position.X;
		Joint.position.Y = joints[12].Position.Y;
		Joint.position.Z = joints[12].Position.Z;
		break;
	case XN_SKEL_LEFT_KNEE:
		if (joints[13].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[13].Position.X;
		Joint.position.Y = joints[13].Position.Y;
		Joint.position.Z = joints[13].Position.Z;
		break;
	case XN_SKEL_LEFT_ANKLE:
		if (joints[14].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[14].Position.X;
		Joint.position.Y = joints[14].Position.Y;
		Joint.position.Z = joints[14].Position.Z;
		break;
	case XN_SKEL_LEFT_FOOT:
		if (joints[15].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[15].Position.X;
		Joint.position.Y = joints[15].Position.Y;
		Joint.position.Z = joints[15].Position.Z;
		break;
	case XN_SKEL_RIGHT_HIP:
		if (joints[16].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[16].Position.X;
		Joint.position.Y = joints[16].Position.Y;
		Joint.position.Z = joints[16].Position.Z;
		break;
	case XN_SKEL_RIGHT_KNEE:
		if (joints[17].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[17].Position.X;
		Joint.position.Y = joints[17].Position.Y;
		Joint.position.Z = joints[17].Position.Z;
		break;
	case XN_SKEL_RIGHT_ANKLE:
		if (joints[18].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[18].Position.X;
		Joint.position.Y = joints[18].Position.Y;
		Joint.position.Z = joints[18].Position.Z;
		break;
	case XN_SKEL_RIGHT_FOOT:
		if (joints[19].TrackingState == 0){ Joint.fConfidence = 0; }
		else{ Joint.fConfidence = 1; }
		Joint.position.X = joints[19].Position.X;
		Joint.position.Y = joints[19].Position.Y;
		Joint.position.Z = joints[19].Position.Z;
		break;
	default:
		break;
	}
}

void KinectApp::showPosition(std::string strMsg, CameraSpacePoint& point)
{
	printf("%s:\t%f\t%f\t%f\n", strMsg, point.X, point.Y, point.Z);
}


void KinectApp::transQuaternion(Quaternion q, int i, int j, int k)
{
	//do not input i, j, k anything except for 1 or -1
	if ((i == 1 || i == -1) && (j == 1 || j == -1) && (k == 1 || k == -1))
	{
		q.qx = q.qx * i;
		q.qy = q.qy * j;
		q.qz = q.qz * k;
	}
	else
	{
		//printf("i, j, k  must be 1 or -1");
		exit(1);
	}
}
void  KinectApp::showJointOrientations(std::string str, JointOrientation jointOrientation)
{
	double angle = acos(jointOrientation.Orientation.w) * 2 * 57.29578; \
		printf("%s:\tangle=%f\nw=%f\tx=%f\ty=%f\tz=%f\n",
		str.c_str(),
		angle,
		jointOrientation.Orientation.w,
		jointOrientation.Orientation.x,
		jointOrientation.Orientation.y,
		jointOrientation.Orientation.z
		);
}
Quaternion KinectApp::transQuaternion2(Quaternion q, int i, int j, int k)
{
	Quaternion q2;
	q2.qw = 0.0f;
	q2.qx = 0.0f;
	q2.qy = 0.0f;
	q2.qz = 0.0f;
	//do not input i, j, k anything except for 1 or -1
	if ((i == 1 || i == -1) && (j == 1 || j == -1) && (k == 1 || k == -1))
	{
		q2.qw = q.qw;
		q2.qx = q.qx * i;
		q2.qy = q.qy * j;
		q2.qz = q.qz * k;
	}
	else{
		printf("i, j, k  must be 1 or -1");
		//exit(1);
	}
	return q2;
}

//copy from SceneDrawer.cpp
Quaternion KinectApp::MultiQuaternion(Quaternion p, Quaternion q)
{
	Quaternion r;
	r.qw = p.qw * q.qw - p.qx * q.qx - p.qy * q.qy - p.qz * q.qz;
	r.qx = p.qw * q.qx + p.qx * q.qw + p.qy * q.qz - p.qz * q.qy;
	r.qy = p.qw * q.qy - p.qx * q.qz + p.qy * q.qw + p.qz * q.qx;
	r.qz = p.qw * q.qz + p.qx * q.qy - p.qy * q.qx + p.qz * q.qw;

	return r;
}

XnPoint3D KinectApp::convertCameraSpacePoint2Point2XnPoint3D(CameraSpacePoint csp)
{
	XnPoint3D xp3;
	xp3.X = csp.X;
	xp3.Y = csp.Y;
	xp3.Z = csp.Z;
	//printf("x=%f, y=%f, z=%f\n",xp3.X,xp3.Y,xp3.Z);
	return xp3;
}

Quaternion KinectApp::convertVector42Quaternion(Vector4 vector4)
{
	Quaternion quaternion;
	quaternion.qw = vector4.w;
	quaternion.qx = vector4.x;
	quaternion.qy = vector4.y;
	quaternion.qz = vector4.z;
	return quaternion;
}

void KinectApp::drawString(cv::Mat& bodyImage, const Joint& joint, int r, const cv::Scalar& color)
{
	if (joint.TrackingState == TrackingState::TrackingState_Tracked)
	{
		//tlanslate camera coordinate system into Depth coordinate system
		ComPtr<ICoordinateMapper> mapper;
		ERROR_CHECK(kinect->get_CoordinateMapper(&mapper));

		DepthSpacePoint point;
		mapper->MapCameraPointToDepthSpace(joint.Position, &point);

		//cv::circle(bodyImage, cv::Point((int)point.X, (int)point.Y), r, color, -1);
		char buf[128];
		sprintf_s(buf, "%d", joint.JointType);
		cv::putText(bodyImage, buf, cv::Point(width - (int)point.X, (int)point.Y), CV_FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 200), 2, CV_AA);
	}
}

/*!
* @brief copy JointPosition
*/
void KinectApp::copy_joint_position(XnSkeletonJointPosition& s, XnSkeletonJointPosition& t)
{
	t.fConfidence = s.fConfidence;
	t.position.X = s.position.X;
	t.position.Y = s.position.Y;
	t.position.Z = s.position.Z;
}


/*!
* ------------------------------------------------------------------------------------------
* @brief 
* @param[in/out] XnPoint3D&
* @param[in]     Quaterion&
* @param[in]     bool       
* ------------------------------------------------------------------------------------------
*/
void KinectApp::RotVec_Zaxis(XnPoint3D &v, Quaternion q, bool axis1) {

	if (axis1 == true) {
		q.qy = 0.0;
	}

	float rw = v.X *  q.qx + v.Y *  q.qy + v.Z * -q.qz;
	float rx = v.X *  q.qw + v.Y *  q.qz + v.Z *  q.qy;
	float ry = v.X * -q.qz + v.Y *  q.qw + v.Z * -q.qx;
	float rz = v.X * -q.qy + v.Y *  q.qx + v.Z *  q.qw;

	v.X = q.qx *  rw + q.qy * -rz + q.qz * -ry + q.qw *  rx;
	v.Y = q.qx *  rz + q.qy *  rw + q.qz *  rx + q.qw *  ry;
	v.Z = q.qx *  ry + q.qy * -rx + q.qz *  rw + q.qw * -rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @brief
* @param[in/out] XnPoint3D& 
* @param[in]     Quaterion& 
* @param[in]     bool      
* ------------------------------------------------------------------------------------------
*/
void KinectApp::RotVec_Xaxis(XnPoint3D &v, Quaternion q, bool axis1) {

	if (axis1 == true) {
		q.qy = 0.0;
	}

	float rw = v.X * -q.qx + v.Y *  q.qy + v.Z *  q.qz;
	float rx = v.X *  q.qw + v.Y * -q.qz + v.Z *  q.qy;
	float ry = v.X *  q.qz + v.Y *  q.qw + v.Z *  q.qx;
	float rz = v.X * -q.qy + v.Y * -q.qx + v.Z *  q.qw;

	v.X = q.qx *  rw + q.qy *  rz + q.qz * -ry + q.qw * -rx;
	v.Y = q.qx * -rz + q.qy *  rw + q.qz * -rx + q.qw *  ry;
	v.Z = q.qx *  ry + q.qy *  rx + q.qz *  rw + q.qw *  rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @brief 
* @param[in/out] XnPoint3D& 
* @param[in]     Quaterion&
* @param[in]     bool     
* ------------------------------------------------------------------------------------------
*/
void KinectApp::RotVec_Yaxis(XnPoint3D &v, Quaternion q, bool axis1) {

	if (axis1 == true) {
		q.qy = 0.0;
	}

	float rw = v.X *  q.qx + v.Y * -q.qy + v.Z *  q.qz;
	float rx = v.X *  q.qw + v.Y * -q.qz + v.Z * -q.qy;
	float ry = v.X *  q.qz + v.Y *  q.qw + v.Z * -q.qx;
	float rz = v.X *  q.qy + v.Y *  q.qx + v.Z *  q.qw;

	v.X = q.qx *  rw + q.qy *  rz + q.qz *  ry + q.qw * rx;
	v.Y = q.qx * -rz + q.qy *  rw + q.qz *  rx + -q.qw * ry;
	v.Z = q.qx * -ry + q.qy * -rx + q.qz *  rw + q.qw * rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @deprecated
* @brief 
* @param[in/out] XnPoint3D&
* @param[in]     Quaterion& 
* @param[in]     bool  
* ------------------------------------------------------------------------------------------
*/
void KinectApp::RotVec_Reverse(XnPoint3D &v, Quaternion q, bool axis1) {

	float rw = v.X * -q.qx + v.Y * -q.qy + v.Z * -q.qz;
	float rx = v.X *  q.qw + v.Y *  q.qz + v.Z * -q.qy;
	float ry = v.X * -q.qz + v.Y *  q.qw + v.Z *  q.qx;
	float rz = v.X *  q.qy + v.Y * -q.qx + v.Z *  q.qw;

	v.X = q.qx *  rw + q.qy * -rz + q.qz *  ry + q.qw * -rx;
	v.Y = q.qx *  rz + q.qy *  rw + q.qz * -rx + q.qw * -ry;
	v.Z = q.qx * -ry + q.qy *  rx + q.qz *  rw + q.qw * -rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @brief
* @param[in/out] XnPoint3D&
* @param[in]     Quaterion& 
* @param[in]     bool      
* ------------------------------------------------------------------------------------------
*/
void KinectApp::RotVec_Xaxis_Reverse(XnPoint3D &v, Quaternion q, bool axis1) {

	if (axis1 == true) {
		q.qy = 0.0;
	}

	// ------------------------------------------------------------
	float rw = v.X *  q.qx + v.Y * -q.qy + v.Z * -q.qz;
	float rx = v.X *  q.qw + v.Y *  q.qz + v.Z * -q.qy;
	float ry = v.X * -q.qz + v.Y *  q.qw + v.Z * -q.qx;
	float rz = v.X *  q.qy + v.Y *  q.qx + v.Z *  q.qw;

	// ------------------------------------------------------------
	v.X = q.qx *  rw + q.qy * -rz + q.qz *  ry + q.qw *  rx;
	v.Y = q.qx *  rz + q.qy *  rw + q.qz *  rx + q.qw * -ry;
	v.Z = q.qx * -ry + q.qy * -rx + q.qz *  rw + q.qw * -rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @deprecated
* @brief
* @param[in/out] XnPoint3D&
* @param[in]     Quaterion& 
* @param[in]     bool       
* ------------------------------------------------------------------------------------------
*/
void KinectApp::RotVec_Zaxis_Reverse(XnPoint3D &v, Quaternion q, bool axis1) 
{
	float rw = v.X * -q.qx + v.Y * -q.qy + v.Z *  q.qz;
	float rx = v.X *  q.qw + v.Y * -q.qz + v.Z * -q.qy;
	float ry = v.X *  q.qz + v.Y *  q.qw + v.Z *  q.qx;
	float rz = v.X *  q.qy + v.Y * -q.qx + v.Z *  q.qw;
	v.X = q.qx *  rw + q.qy *  rz + q.qz *  ry + q.qw * -rx;
	v.Y = q.qx * -rz + q.qy *  rw + q.qz * -rx + q.qw * -ry;
	v.Z = q.qx * -ry + q.qy *  rx + q.qz *  rw + q.qw *  rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @deprecated
* @brief
* @param[in/out] XnPoint3D& 
* @param[in]     Quaterion&
* @param[in]     bool       
* ------------------------------------------------------------------------------------------
*/
void KinectApp::RotVec_Yaxis_Reverse(XnPoint3D &v, Quaternion q, bool axis1) {


	float rw = v.X * -q.qx + v.Y *  q.qy + v.Z * -q.qz;
	float rx = v.X *  q.qw + v.Y *  q.qz + v.Z *  q.qy;
	float ry = v.X * -q.qz + v.Y *  q.qw + v.Z *  q.qx;
	float rz = v.X * -q.qy + v.Y * -q.qx + v.Z *  q.qw;

	v.X = q.qx *  rw + q.qy * -rz + q.qz * -ry + q.qw * -rx;
	v.Y = q.qx *  rz + q.qy *  rw + q.qz * -rx + q.qw *  ry;
	v.Z = q.qx *  ry + q.qy *  rx + q.qz *  rw + q.qw * -rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @deprecated
* @brief
* @param[in/out] XnPoint3D& 
* @param[in]     Quaterion& 
* @param[in]     bool      
* ------------------------------------------------------------------------------------------
*/
void KinectApp::RotVec_YZaxis_Reverse(XnPoint3D &v, Quaternion q, bool axis1) 
{

	float rw = v.X * -q.qx + v.Y *  q.qy + v.Z *  q.qz;
	float rx = v.X *  q.qw + v.Y * -q.qz + v.Z *  q.qy;
	float ry = v.X *  q.qz + v.Y *  q.qw + v.Z *  q.qx;
	float rz = v.X * -q.qy + v.Y * -q.qx + v.Z *  q.qw;

	v.X = q.qx *  rw + q.qy *  rz + q.qz * -ry + q.qw * -rx;
	v.Y = q.qx * -rz + q.qy *  rw + q.qz * -rx + q.qw *  ry;
	v.Z = q.qx *  ry + q.qy *  rx + q.qz *  rw + q.qw *  rz;

}

/*!
* ------------------------------------------------------------------------------------------
* @brief 
* @param[out] XnPoint3D&             
* @param[in]  XnSkeltonJointPosition 
* @param[in]  XnSkeltonJointPosition 
* ------------------------------------------------------------------------------------------
*/
bool  KinectApp::DiffVec_Xaxis(XnPoint3D &rvec, XnSkeletonJointPosition jvec, XnSkeletonJointPosition kvec) {

	if (jvec.fConfidence < 0.5 || kvec.fConfidence < 0.5) {
		return false;
	}
	rvec.X = (kvec.position.X - jvec.position.X);
	rvec.Y = kvec.position.Y - jvec.position.Y;
	rvec.Z = -(kvec.position.Z - jvec.position.Z);

	float length = sqrt(rvec.X * rvec.X + rvec.Y * rvec.Y + rvec.Z * rvec.Z);
	rvec.X = rvec.X / length;
	rvec.Y = rvec.Y / length;
	rvec.Z = rvec.Z / length;

	return true;
}
