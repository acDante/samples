/*
 * ManNiiController.cpp
 *
 *  Created on: 2015/03/09
 *      Author: sigverse
 */

#include "ManNiiAvatarController.h"
#include <sigverse/SimObj.h>

int h = 0;

///@brief Initialize this controller.
void ManNiiAvatarController::onInit(InitEvent &evt)
{
	this->kinectV2Service = NULL;
	this->oculusDK1Service = NULL;

	this->defaultHeadJoint0Quaternion[0] = 1.0;
	this->defaultHeadJoint0Quaternion[1] = 0.0;
	this->defaultHeadJoint0Quaternion[2] = 0.0;
	this->defaultHeadJoint0Quaternion[3] = 0.0;

	this->prevYaw = this->prevPitch = this->prevRoll = 0.0;

	SimObj *my = getObj(myname());
}


///@brief Movement of the robot.
double ManNiiAvatarController::onAction(ActionEvent &evt)
{
	this->kinectV2ServiceName = kinectV2ServiceNameDefault;
	bool kinectV2Available = checkService(this->kinectV2ServiceName);
	if (kinectV2Available && this->kinectV2Service == NULL) {
		this->kinectV2Service = connectToService(this->kinectV2ServiceName);
	}
	else if (!kinectV2Available && this->kinectV2Service != NULL) {
		this->kinectV2Service = NULL;
	}

	this->oculusDK1ServiceName = oculusDK1ServiceNameDefault;
	bool oculusDK1Available = checkService(this->oculusDK1ServiceName);
	if (oculusDK1Available && this->oculusDK1Service == NULL) {
		this->oculusDK1Service = connectToService(this->oculusDK1ServiceName);
	}
	else if (!oculusDK1Available && this->oculusDK1Service != NULL) {
		this->oculusDK1Service = NULL;
	}

	return 1.0;
}

void ManNiiAvatarController::onRecvMsg(RecvMsgEvent &evt)
{

	const std::string allMsg = evt.getMsg();
	//std::cout << "hogehoge" << allMsg << std::endl;

	this->convertMessage2AvatarPosture(allMsg);

	//SimObj *myself = getObj(myname());
	//this->setJointQuaternions(myself);
}

//void ManNiiAvatarController::convertMessage2AvatarPosture(const std::string &message, const std::string &recordDelim, const std::string &keyDelim, const std::string &valueDelim)
void ManNiiAvatarController::convertMessage2AvatarPosture(const std::string &message)
{
	const std::string recordDelim = recordDelimDefault;
	const std::string keyDelim = keyDelimDefault;
	const std::string valueDelim = valueDelimDefault;

	const std::string messageHeader = this->getMessageHeader(message);
	//std::cout << messageHeader << std::endl;

//	bool messageHeaderOK = this->checkMessageHeader(messageHeader);
//	if (!messageHeaderOK) {
//		std::cout << "Illegal message header : " << messageHeader << std::endl;
//	}

	// Split string to records by ";".
	std::vector<std::string> records;
	AvatarController::splitString(message, recordDelim, records);

	// Get device type.
	// Split one record to KEY and VALUE by ":".
	const std::string firstRecord = records[0];
	std::vector<std::string> firstKeyAndValue;
	AvatarController::splitString(firstRecord, keyDelim, firstKeyAndValue);
	const std::string firstKey = firstKeyAndValue[0];
	const std::string firstValue = firstKeyAndValue[1];

	if (firstValue == devTypeOculus) {

		const std::string eulerRecord = records[2];
		std::vector<std::string> keyAndValue;
		AvatarController::splitString(eulerRecord, keyDelim, keyAndValue);

		const std::string key = keyAndValue[0];
		const std::string value = keyAndValue[1];

		if (key != "EULER") {
			std::cout << "Illegal key :" << key << std::endl;
			return;
		}

		// Split VALUE to vector by ",", if VALUE is vector.
		std::vector<std::string> vector;
		AvatarController::splitString(value, valueDelim, vector);

		if ((int)vector.size() == 1) {

		}
		else if ((int)vector.size() > 1) {
			// When VALUE is vector data.
			//std::cout << key << " = " << value << std::endl;

			// ベクトルデータの時は、「(」が第一要素の前に入っている。
			// この「(」をキャンセルする。
			vector[0].replace(0, 1, ""); // 第一要素の先頭の文字を空の文字列に置換する

			const float yaw = atof(vector[0].c_str());
			const float pitch = atof(vector[1].c_str());
			const float roll = atof(vector[2].c_str());

			if (yaw == this->prevYaw && pitch == this->prevPitch && roll == this->prevRoll) {
				return;
			}
			else {
				this->prevYaw = yaw;
				this->prevPitch = pitch;
				this->prevRoll = roll;
			}

			EulerAngleType eulerAngle;
			eulerAngle.yaw = yaw;
			eulerAngle.pitch = pitch;
			eulerAngle.roll = roll;

			ManNiiAvatarPosture tmpManNiiPosture;
			this->convertEulerAngle2ManNiiAvatarPosture(eulerAngle, tmpManNiiPosture);

			SimObj *obj = getObj(myname());
			this->setJointQuaternionsForOculus(obj, &tmpManNiiPosture);
		}
	}

	if (firstValue == devTypeKinectV2) {
		KinectV2JointOrientation kinectV2Joints[KinectV2JointType_Count];

		for(int i = 0; i < (int)records.size(); i++) {

			// Split one record to KEY and VALUE by ":".
			const std::string oneRecord = records[i];
			std::vector<std::string> keyAndValue;
			AvatarController::splitString(oneRecord, keyDelim, keyAndValue);

			// Get key and value as string.
			const std::string key = keyAndValue[0];
			const std::string value = keyAndValue[1];

			// Split VALUE to vector by ",", if VALUE is vector.
			std::vector<std::string> vector;
			AvatarController::splitString(value, valueDelim, vector);

			if ((int)vector.size() == 1) {
				// When VALUE is scalar or string data.
	//			char *endPtrStrtod;
	//			const double scalarValue = strtod(vector[0].c_str(), &endPtrStrtod);
				// vector[0]をdouble型に変換します。
				// 変換できなかった文字が第3引数に入ります。
				// 詳しくはstrtod()の動作を見てください。

			}
			else if ((int)vector.size() > 1) {
				// When VALUE is vector data.
				//std::cout << key << " = ";

				// ベクトルデータの時は、「(」が第一要素の前に入っている。
				// この「(」をキャンセルする。
				vector[0].replace(0, 1, ""); // 第一要素の先頭の文字を空の文字列に置換する

				const KinectV2JointType jt = this->getJointTypeFromKey(key);

				if (jt != KinectV2JointType_Count) {
					const double w = atof(vector[0].c_str());
					const double x = atof(vector[1].c_str());
					const double y = atof(vector[2].c_str());
					const double z = atof(vector[3].c_str());
					KinectV2JointOrientation tmpJO;
					tmpJO.jointType = jt;
					tmpJO.orientation.w = w;
					tmpJO.orientation.x = x;
					tmpJO.orientation.y = y;
					tmpJO.orientation.z = z;
					kinectV2Joints[tmpJO.jointType] = tmpJO;
					//std::cout << w << ", " << x << ", " << y << ", " << z << std::endl;
				}
				else if (key == "SpBs_P") {
					// ルートのポジションを入れるときの処理をここに書く
				}
				else {
					std::cout << "ELLEGAL JointType : " << key << std::endl;
				}
	//
	//			for (int j = 0; j < (int)vector.size(); j++) {
	//				const double element = atof(vector[j].c_str());
	//				std::cout << element;
	//				if (j != ((int)vector.size() - 1)) {
	//					std::cout << ",";
	//				}
	//			}
	//			std::cout << " VECTOR DATA." <<  std::endl;
			}
		}

		ManNiiAvatarPosture tmpManNiiPosture;

		this->convertKinectV2JointOrientations2ManNiiAvatarPosture(kinectV2Joints, tmpManNiiPosture);

		SimObj *obj = getObj(myname());
		this->setJointQuaternions(obj, &tmpManNiiPosture);
	}
}


void ManNiiAvatarController::setJointQuaternion(SimObj *obj, const ManNiiJointQuaternion &jq)
{
	obj->setJointQuaternion(manNiiJointTypeStr(jq.manJointType).c_str(), jq.quaternion.w, jq.quaternion.x, jq.quaternion.y, jq.quaternion.z);
	//std::cout << manNiiJointTypeStr(jq.manJointType).c_str() << jq.quaternion.w << jq.quaternion.x << jq.quaternion.y << jq.quaternion.z << std::endl;
}

void ManNiiAvatarController::setJointQuaternions(SimObj *obj)
{
	//ManNiiAvatarPosture *pos = this->avatarPosture;
	//const ManNiiJointQuaternion *jq = dynamic_cast<ManNiiJointQuaternion*>(&this->avatarPosture);
	//ManNiiAvatarPosture *jq = dynamic_cast<ManNiiAvatarPosture*>(this->avatarPosture);
//	ManNiiAvatarPosture *jq = this->avatarPosture;
//
//	this->setJointQuaternion(obj, jq->joints[WAIST_JOINT1]);
//	this->setJointQuaternion(obj, jq->joints[RARM_JOINT2]);
//	this->setJointQuaternion(obj, jq->joints[LARM_JOINT2]);
//	this->setJointQuaternion(obj, jq->joints[RLEG_JOINT2]);
//	this->setJointQuaternion(obj, jq->joints[LLEG_JOINT2]);
//	this->setJointQuaternion(obj, jq->joints[RLEG_JOINT4]);
//	this->setJointQuaternion(obj, jq->joints[LLEG_JOINT4]);
//	this->setJointQuaternion(obj, jq->joints[RARM_JOINT3]);
//	this->setJointQuaternion(obj, jq->joints[LARM_JOINT3]);
	//this->setJointQuaternion(obj, jq[RARM_JOINT5]);
	//this->setJointQuaternion(obj, jq[LARM_JOINT5]);
}

void ManNiiAvatarController::setJointQuaternions(SimObj *obj, ManNiiAvatarPosture *manNiiAvatarPosture)
{
	//Man
	//this->avatarPosture =
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[WAIST_JOINT1]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[RARM_JOINT2]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[LARM_JOINT2]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[RLEG_JOINT2]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[LLEG_JOINT2]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[RLEG_JOINT4]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[LLEG_JOINT4]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[RARM_JOINT3]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[LARM_JOINT3]);
	this->avatarPosture = manNiiAvatarPosture;
}

void ManNiiAvatarController::setJointQuaternionsForOculus(SimObj *obj, ManNiiAvatarPosture *manNiiAvatarPosture)
{
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[HEAD_JOINT0]);
	this->avatarPosture = manNiiAvatarPosture;
}


void ManNiiAvatarController::convertKinectV2JointOrientations2ManNiiAvatarPosture(KinectV2JointOrientation* kinectV2Joints, ManNiiAvatarPosture &manNiiAvatarPosture)
{
	const double coef = 1.0 / sqrt(2.0);

	Quaternion q0 = Quaternion(kinectV2Joints[0].orientation);
	Quaternion q1 = Quaternion(kinectV2Joints[1].orientation);
	Quaternion q1_h = Quaternion(0.0, 0.0, 1.0, 0.0);
	Quaternion q1_rot = Quaternion::calcCrossProduct(q1_h, q1);
	Quaternion q2 = Quaternion(kinectV2Joints[20].orientation);
	Quaternion q2_h = Quaternion(0.0, 0.0, -1.0, 0.0);
	Quaternion q2_rot = Quaternion::calcCrossProduct(q2, q2_h);

	Quaternion q3 = Quaternion(kinectV2Joints[8].orientation);
	Quaternion q4 = Quaternion(kinectV2Joints[9].orientation);
	Quaternion q7 = Quaternion(kinectV2Joints[5].orientation);
	Quaternion q9 = Quaternion(kinectV2Joints[10].orientation);
	Quaternion q13 = Quaternion(kinectV2Joints[17].orientation);
	Quaternion q15 = Quaternion(kinectV2Joints[13].orientation);
	Quaternion q17 = Quaternion(kinectV2Joints[18].orientation);
	Quaternion q19 = Quaternion(kinectV2Joints[14].orientation);
	Quaternion q20 = Quaternion(kinectV2Joints[20].orientation);
	Quaternion q20_rot = Quaternion::calcCrossProduct(q1_h, q20);
	Quaternion q21 = Quaternion(kinectV2Joints[6].orientation);
	Quaternion q25 = Quaternion(kinectV2Joints[11].orientation);
	Quaternion q26 = Quaternion(kinectV2Joints[7].orientation);
	Quaternion q1_con = Quaternion(q20_rot.w, -q20_rot.x, -q20_rot.y, -q20_rot.z);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////  Right Arm quaternion //////////////////////
	Quaternion q5;
	Quaternion q5_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q5_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion q5_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion q5_rotx = Quaternion::calcCrossProduct(q5_x, q4);
	q5.w = q5_rotx.w;
	q5.x = q5_rotx.y;
	q5.y = q5_rotx.z;
	q5.z = q5_rotx.x;
	q5 = Quaternion::calcCrossProduct(q2_h, q5);

	q5 = Quaternion::calcCrossProduct(q5_y, q5);
	//q5.w = q5.w;
	q5.x = -q5.x;
	q5.y = -q5.y;
	//q5.z = q5.z;
	q5 = Quaternion::calcCrossProduct(q1_con, q5);
	//////////////////////////////////////////////////////////////////////

	////////////// Left Arm quaternion ///////////////////////////////////
	Quaternion q6;
	Quaternion q6_x = Quaternion(coef, coef, 0.0, 0.0);
	Quaternion q6_z = Quaternion(coef, 0.0, 0.0, coef);
	Quaternion q6_y = Quaternion(coef, 0.0, coef, 0.0);
	Quaternion q6_rotx = Quaternion::calcCrossProduct(q6_x, q7);
	q6.w = q6_rotx.w;
	q6.x = q6_rotx.y;
	q6.y = q6_rotx.z;
	q6.z = q6_rotx.x;
	q6 = Quaternion::calcCrossProduct(q2_h, q6);
	q6 = Quaternion::calcCrossProduct(q6_y, q6);
	//q6.w = q6.w;
	//q6.x = q6.x;
	//q6.y = q6.y;
	//q6.z = q6.z;
	q6 = Quaternion::calcCrossProduct(q1_con, q6);

	//////////////////////////////////////////////////////////////////////////

	///////////////////////////  Right hipe quaternion //////////////////////
	Quaternion q12;
	Quaternion q12_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q12_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion q12_y = Quaternion(coef, 0.0, coef, 0.0);
	Quaternion q12_rotx = Quaternion::calcCrossProduct(q12_x, q13);
	Quaternion q3_h = Quaternion(0.0, 0.0, 1.0, 0.0);
	Quaternion q12_h = Quaternion(0.0, 0.0, 0.0, -1.0);
	q12 = Quaternion::calcCrossProduct(q12_h, q13);
	Quaternion q12_b = Quaternion::calcCrossProduct(q12_y, q12);
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
	Quaternion q16_rotx = Quaternion::calcCrossProduct(q16_x, q17);
	Quaternion q16_h = Quaternion(0.0, 0.0, 0.0, -1.0);
	q16 = Quaternion::calcCrossProduct(q16_h, q17);
	Quaternion q16_b = Quaternion::calcCrossProduct(q16_y, q16);
	q16.w = q16_b.w;
	q16.x = -q16_b.z;
	q16.y = -q16_b.y;
	q16.z = -q16_b.x;

	Quaternion  q12_con;
	q12_con.w = q12.w;
	q12_con.x = -q12.x;
	q12_con.y = -q12.y;
	q12_con.z = -q12.z;
	q16 = Quaternion::calcCrossProduct(q12_con, q16);
	//////////////////////////////////////////////////////////////////////


	///////////////////////////  left hipe quaternion //////////////////////
	Quaternion q14;
	Quaternion q14_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q14_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion q14_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion q14_rotx = Quaternion::calcCrossProduct(q14_x, q15);

	Quaternion q14_h = Quaternion(0.0, 0.0, 0.0, 1.0);
	q14 = Quaternion::calcCrossProduct(q14_h, q15);
	Quaternion q14_b = Quaternion::calcCrossProduct(q14_y, q14);
	q14.w = q14_b.w;
	q14.x = q14_b.z;
	q14.y = -q14_b.y;
	q14.z = q14_b.x;
	//q14 = Quaternion::calcCrossProduct(q2_h, q14);
	//////////////////////////////////////////////////////////////////////


	///////////////////////////  left leg quaternion //////////////////////
	Quaternion q18;
	Quaternion q18_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q18_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion q18_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion q18_rotx = Quaternion::calcCrossProduct(q18_x, q19);
	Quaternion q18_h = Quaternion(0.0, 0.0, 0.0, 1.0);
	q18 = Quaternion::calcCrossProduct(q18_h, q19);
	Quaternion q18_b = Quaternion::calcCrossProduct(q18_y, q18);
	q18.w = q18_b.w;
	q18.x = q18_b.z;
	q18.y = -q18_b.y;
	q18.z = q18_b.x;

	Quaternion  q14_con;
	q14_con.w = q14.w;
	q14_con.x = -q14.x;
	q14_con.y = -q14.y;
	q14_con.z = -q14.z;
	q18 = Quaternion::calcCrossProduct(q14_con, q18);
	//////////////////////////////////////////////////////////////////////


	////////////// Right for Arm  quaternion /////////////////////////////
	Quaternion q8;
	Quaternion q8_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q8_z = Quaternion(coef, 0.0, 0.0, coef);
	Quaternion q8_y = Quaternion(coef, 0.0, coef, 0.0);
	Quaternion q8_rotx = Quaternion::calcCrossProduct(q8_x, q9);

	q8.w = q8_rotx.w;
	q8.x = q8_rotx.y;
	q8.y = q8_rotx.z;
	q8.z = q8_rotx.x;
	q8 = Quaternion::calcCrossProduct(q8_y, q8);
	Quaternion q8_xx = Quaternion(0.0, 1.0, 0.0, 0.0);
	Quaternion q8_zz = Quaternion(0.0, 0.0, 0.0, 1.0);
	Quaternion  q8_va = Quaternion(q8.w, q8.x, q8.y, q8.z);

	q8 = Quaternion::calcCrossProduct(q2_h, q8);
	q8.w = q8_va.w;
	q8.x = -q8_va.x;
	q8.y = -q8_va.y;
	q8.z = q8_va.z;

	Quaternion q5_con;
	q5_con.w = q5.w;
	q5_con.x = -q5.x;
	q5_con.y = -q5.y;
	q5_con.z = -q5.z;
	q8 = Quaternion::calcCrossProduct(q5_con, q8);
	////////////////////////////////////////////////////////////////////////

	////////////// Left for Arm  quaternion ////////////////////////////////
	Quaternion q22;
	Quaternion q22_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q22_z = Quaternion(coef, 0.0, 0.0, coef);
	Quaternion q22_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion q22_rotx = Quaternion::calcCrossProduct(q22_x, q21);

	q22.w = q22_rotx.w;
	q22.x = q22_rotx.y;
	q22.y = q22_rotx.z;
	q22.z = q22_rotx.x;
	q22 = Quaternion::calcCrossProduct(q22_y, q22);

	Quaternion q22_xx = Quaternion(0.0, 1.0, 0.0, 0.0);
	Quaternion q22_zz = Quaternion(0.0, 0.0, 0.0, 1.0);
	Quaternion  q22_va = Quaternion(q22.w, q22.x, q22.y, q22.z);

	q22 = Quaternion::calcCrossProduct(q2_h, q8);
	q22.w = q22_va.w;
	q22.x = q22_va.x;
	q22.y = -q22_va.y;
	q22.z = -q22_va.z;

	Quaternion q6_con;
	q6_con.w = q6.w;
	q6_con.x = -q6.x;
	q6_con.y = -q6.y;
	q6_con.z = -q6.z;
	q22 = Quaternion::calcCrossProduct(q6_con, q22);

	//////////////////////////////////////////////////////////////////////////



	////////////// Right hand  quaternion ////////////////////////////////////
	Quaternion q27;
	Quaternion q27_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q27_z = Quaternion(coef, 0.0, 0.0, coef);
	Quaternion q27_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion q27_rotx = Quaternion::calcCrossProduct(q27_x, q25);
	q27.w = q27_rotx.w;
	q27.x = q27_rotx.y;
	q27.y = q27_rotx.z;
	q27.z = q27_rotx.x;
	Quaternion q27_xx = Quaternion(0.0, 1.0, 0.0, 0.0);
	Quaternion q27_zz = Quaternion(0.0, 0.0, 0.0, 1.0);
	Quaternion  q27_va = Quaternion(q27.w, q27.x, q27.y, q27.z);
	q27 = Quaternion::calcCrossProduct(q2_h, q27);
	q27.w = q27_va.w;
	q27.x = q27_va.x;
	q27.y = q27_va.y;
	q27.z = q27_va.z;
	Quaternion q8_con;
	q8_con.w = q8.w;
	q8_con.x = -q8.x;
	q8_con.y = -q8.y;
	q8_con.z = -q8.z;
	q27 = Quaternion::calcCrossProduct(q8_con, q27);
	//////////////////////////////////////////////////////////////////////////


	////////////// Left Hand  quaternion ///////////////////////////////////
	Quaternion q28;
	Quaternion q28_x = Quaternion(coef, -coef, 0.0, 0.0);
	Quaternion q28_z = Quaternion(coef, 0.0, 0.0, coef);
	Quaternion q28_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion q28_rotx = Quaternion::calcCrossProduct(q28_x, q26);

	q28.w = q28_rotx.w;
	q28.x = q28_rotx.y;
	q28.y = q28_rotx.z;
	q28.z = q28_rotx.x;
	q28 = Quaternion::calcCrossProduct(q28_y, q28);
	Quaternion q28_xx = Quaternion(0.0, 1.0, 0.0, 0.0);
	Quaternion q28_zz = Quaternion(0.0, 0.0, 0.0, 1.0);
	Quaternion  q28_va = Quaternion(q28.w, q28.x, q28.y, q28.z);
	q28 = Quaternion::calcCrossProduct(q2_h, q28);
	q28.w = q28_va.w;
	q28.x = q28_va.x;
	q28.y = -q28_va.y;
	q28.z = -q28_va.z;
	Quaternion q22_con;
	q22_con.w = q22.w;
	q22_con.x = -q22.x;
	q22_con.y = -q22.y;
	q22_con.z = -q22.z;
	q28 = Quaternion::calcCrossProduct(q22_con, q28);

	//ManNiiAvatarPosture posture;
	manNiiAvatarPosture.joints[WAIST_JOINT1].quaternion = q1_rot;
	manNiiAvatarPosture.joints[RARM_JOINT2].quaternion = q5;
	manNiiAvatarPosture.joints[LARM_JOINT2].quaternion = q6;
	manNiiAvatarPosture.joints[RLEG_JOINT2].quaternion = q12;
	manNiiAvatarPosture.joints[LLEG_JOINT2].quaternion = q14;
	manNiiAvatarPosture.joints[RLEG_JOINT4].quaternion = q16;
	manNiiAvatarPosture.joints[LLEG_JOINT4].quaternion = q18;
	manNiiAvatarPosture.joints[RARM_JOINT3].quaternion = q8;
	manNiiAvatarPosture.joints[LARM_JOINT3].quaternion = q22;
//	this->posture.joints[ManNiiJointType_RARM_JOINT5].quaternion = q27;
//	this->posture.joints[ManNiiJointType_LARM_JOINT5].quaternion = q28;

	manNiiAvatarPosture.joints[WAIST_JOINT1].manJointType = WAIST_JOINT1;
	manNiiAvatarPosture.joints[RARM_JOINT2].manJointType = RARM_JOINT2;
	manNiiAvatarPosture.joints[LARM_JOINT2].manJointType = LARM_JOINT2;
	manNiiAvatarPosture.joints[RLEG_JOINT2].manJointType = RLEG_JOINT2;
	manNiiAvatarPosture.joints[LLEG_JOINT2].manJointType = LLEG_JOINT2;
	manNiiAvatarPosture.joints[RLEG_JOINT4].manJointType = RLEG_JOINT4;
	manNiiAvatarPosture.joints[LLEG_JOINT4].manJointType = LLEG_JOINT4;
	manNiiAvatarPosture.joints[RARM_JOINT3].manJointType = RARM_JOINT3;
	manNiiAvatarPosture.joints[LARM_JOINT3].manJointType = LARM_JOINT3;
	manNiiAvatarPosture.joints[RARM_JOINT5].manJointType = RARM_JOINT5;
	manNiiAvatarPosture.joints[LARM_JOINT5].manJointType = LARM_JOINT5;

	this->avatarPosture = &manNiiAvatarPosture;



	//this->avatarPosture = posture;

//	this->posture.joints[WAIST_JOINT1].quaternion = q1_rot;
//	this->posture.joints[RARM_JOINT2].quaternion = q5;
//	this->posture.joints[LARM_JOINT2].quaternion = q6;
//	this->posture.joints[RLEG_JOINT2].quaternion = q12;
//	this->posture.joints[LLEG_JOINT2].quaternion = q14;
//	this->posture.joints[RLEG_JOINT4].quaternion = q16;
//	this->posture.joints[LLEG_JOINT4].quaternion = q18;
//	this->posture.joints[RARM_JOINT3].quaternion = q8;
//	this->posture.joints[LARM_JOINT3].quaternion = q22;
////	this->posture.joints[ManNiiJointType_RARM_JOINT5].quaternion = q27;
////	this->posture.joints[ManNiiJointType_LARM_JOINT5].quaternion = q28;
//
//	this->posture.joints[WAIST_JOINT1].manJointType = WAIST_JOINT1;
//	this->posture.joints[RARM_JOINT2].manJointType = RARM_JOINT2;
//	this->posture.joints[LARM_JOINT2].manJointType = LARM_JOINT2;
//	this->posture.joints[RLEG_JOINT2].manJointType = RLEG_JOINT2;
//	this->posture.joints[LLEG_JOINT2].manJointType = LLEG_JOINT2;
//	this->posture.joints[RLEG_JOINT4].manJointType = RLEG_JOINT4;
//	this->posture.joints[LLEG_JOINT4].manJointType = LLEG_JOINT4;
//	this->posture.joints[RARM_JOINT3].manJointType = RARM_JOINT3;
//	this->posture.joints[LARM_JOINT3].manJointType = LARM_JOINT3;
//	this->posture.joints[RARM_JOINT5].manJointType = RARM_JOINT5;
//	this->posture.joints[LARM_JOINT5].manJointType = LARM_JOINT5;

}

void ManNiiAvatarController::convertEulerAngle2ManNiiAvatarPosture(const EulerAngleType &eulerAngle, ManNiiAvatarPosture &manNiiAvatarPosture)
{
	dQuaternion qyaw;
	dQuaternion qpitch;
	dQuaternion qroll;

	qyaw[0] = cos(-eulerAngle.yaw/2.0);
	qyaw[1] = 0.0;
	qyaw[2] = sin(-eulerAngle.yaw/2.0);
	qyaw[3] = 0.0;

	qpitch[0] = cos(-eulerAngle.pitch/2.0);
	qpitch[1] = sin(-eulerAngle.pitch/2.0);
	qpitch[2] = 0.0;
	qpitch[3] = 0.0;

	qroll[0] = cos(-eulerAngle.roll/2.0);
	qroll[1] = 0.0;
	qroll[2] = 0.0;
	qroll[3] = sin(-eulerAngle.roll/2.0);
	dQuaternion tmpQ1;
	dQuaternion tmpQ2;

	dQMultiply0(tmpQ1, qyaw, qpitch);

	dQMultiply0(tmpQ2, tmpQ1, qroll);


	dQuaternion tmpQ3;

	//dQuaternion defaultQ = this->defaultHeadJointQuaternion;
	dQMultiply1(tmpQ3, this->defaultHeadJoint0Quaternion, tmpQ2);

	Quaternion tmpQ4(tmpQ3[0], tmpQ3[1], tmpQ3[2], tmpQ3[3]);

	manNiiAvatarPosture.joints[HEAD_JOINT0].manJointType = HEAD_JOINT0;
	manNiiAvatarPosture.joints[HEAD_JOINT0].quaternion = tmpQ4;

	this->avatarPosture = &manNiiAvatarPosture;

	//my->setJointQuaternion("HEAD_JOINT0", tmpQ3[0], tmpQ3[1], tmpQ3[2], tmpQ3[3]);
}


KinectV2JointType ManNiiAvatarController::getJointTypeFromKey(const std::string &key)
{
	if (key == "SpBs_Q") {
		return SpineBase;
	}
	else if (key ==	"SpMd_Q") {
		return SpineMid;
	}
	else if (key ==	"Neck_Q") {
		return Neck;
	}
	else if (key ==	"Head_Q") {
		return Head;
	}
	else if (key ==	"ShL_Q") {
		return ShoulderLeft;
	}
	else if (key ==	"LbL_Q") {
		return ElbowLeft;
	}
	else if (key ==	"WrL_Q") {
		return WristLeft;
	}
	else if (key ==	"HndL_Q") {
		return HandLeft;
	}
	else if (key ==	"ShR_Q") {
		return ShoulderRight;
	}
	else if (key ==	"LbR_Q") {
		return ElbowRight;
	}
	else if (key ==	"WrR_Q") {
		return WristRight;
	}
	else if (key ==	"HndR_Q") {
		return HandRight;
	}
	else if (key ==	"HpL_Q") {
		return HipLeft;
	}
	else if (key ==	"NeeL_Q") {
		return KneeLeft;
	}
	else if (key ==	"AnkL_Q") {
		return AnkleLeft;
	}
	else if (key ==	"FtL_Q") {
		return FootLeft;
	}
	else if (key ==	"HpR_Q") {
		return HandTipRight;
	}
	else if (key ==	"NeeR_Q") {
		return KneeRight;
	}
	else if (key ==	"AnkR_Q") {
		return AnkleRight;
	}
	else if (key ==	"FtR_Q") {
		return FootRight;
	}
	else if (key ==	"SpSh_Q") {
		return SpineShoulder;
	}
	else if (key ==	"HTL_Q") {
		return HandTipLeft;
	}
	else if (key ==	"ThmL_Q") {
		return ThumbLeft;
	}
	else if (key ==	"HTR_Q") {
		return HandTipRight;
	}
	else if (key ==	"ThmR_Q") {
		return ThumbRight;
	}

	return KinectV2JointType_Count;
}
