/*
 * MirrorTherapyController.cpp
 *
 *  Created on: 2015/03/16
 *      Author: Nozaki
 */

#include "MirrorTherapyController.h"

#include <fstream>

///@brief Initialize this controller.
void MirrorTherapyController::onInit(InitEvent &evt)
{
	this->kinectV2Service = NULL;
	this->oculusDK1Service = NULL;

	this->defaultHeadJoint0Quaternion[0] = 1.0;
	this->defaultHeadJoint0Quaternion[1] = 0.0;
	this->defaultHeadJoint0Quaternion[2] = 0.0;
	this->defaultHeadJoint0Quaternion[3] = 0.0;

	this->prevYaw = this->prevPitch = this->prevRoll = 0.0;

	std::ifstream ifs(parameterFileName.c_str());
	if (!ifs.fail())
	{
		std::cout << "Read parameter file: " << parameterFileName << std::endl;
		double delaytime;
		int revMode;
		ifs >> delaytime >> revMode;
		this->targetDelayTime = delaytime;
		this->reverseMode = reverseModes[revMode];
	}
	else {
		std::cout << "Start without parameter file." << std::endl;

		// For Mirror therapy initialize.
		this->reverseMode = reverseModes[RIGHTHAND]; // Set reverse mode.
		//this->reverseMode = reverseModes[NOREVERSE];

		// Set delay variables.
		this->targetDelayTime = defaultDelayTime;
		gettimeofday(&this->initTimeVal, NULL);
		this->frameNumber = 0;
		this->pastPostures = std::vector<TimeAndPostureType>(timeSeriesBufferSize);
	}

	std::cout << "Reverse: " << this->reverseMode << ", Delay: " << this->targetDelayTime << "[ms]." << std::endl;


	SimObj *my = getObj(myname());
}


///@brief Movement of the robot.
double MirrorTherapyController::onAction(ActionEvent &evt)
{
	std::cout << '\r' << "Reverse: " << this->reverseMode << ", Delay: " << this->targetDelayTime << "[ms].";
	fflush(stdout);

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

void MirrorTherapyController::onRecvMsg(RecvMsgEvent &evt)
{

	const std::string allMsg = evt.getMsg();

	//std::cout << allMsg << std::endl;

	if (this->setReverseModeAndDelayTime(allMsg)) {
		return;
	}

	this->convertMessage2Posture(allMsg);

}

//void MirrorTherapyController::convertMessage2AvatarPosture(const std::string &message, const std::string &recordDelim, const std::string &keyDelim, const std::string &valueDelim)
void MirrorTherapyController::convertMessage2Posture(const std::string &message)
{
	std::vector<Record> records;
	this->convertMessage2Records(message, records);

	Record firstRecord = records[0];

	if (firstRecord.key != devTypeKey) {
		std::cout << "Illegal device type: " << firstRecord.key << std::endl;
		return;
	}

	if (firstRecord.value == devTypeOculus) {

		Record eulerRecord = records[2];

		EulerAngleType eulerAngle = this->generateEulerAngleFromRecord(eulerRecord);

		ManNiiPosture tmpManNiiPosture;
		this->convertEulerAngle2ManNiiPosture(eulerAngle, tmpManNiiPosture);

		SimObj *obj = getObj(myname());
		this->setJointQuaternionsForOculus(obj, &tmpManNiiPosture);

	}

	if (firstRecord.value == devTypeKinectV2) {

		std::vector<Record> kinectRecords;

		KinectV2JointOrientation kinectV2JointOrientations[KinectV2JointType_Count];
		this->convertRecords2KinectV2JointOrientations(records, kinectV2JointOrientations);

		ManNiiPosture tmpManNiiPosture;
		this->convertKinectV2JointOrientations2ManNiiPosture(kinectV2JointOrientations, tmpManNiiPosture);


		// 反転と遅延のための記述 //

		// Store current posture and time stamp.
		TimeAndPostureType current;
		current.timeStamp = this->generateCurrentTimeStamp();
		for (int i = 0; i < ManNiiJointType_Count - 1; i++) {
			current.posture.joints[i] = tmpManNiiPosture.joints[i];
		}
		const unsigned long index = this->frameNumber % timeSeriesBufferSize;
		this->pastPostures[index] = current;

		// Search nearest neighbor of target delay time in stored time stamps.
		// this->pastPostures に、過去の500メッセージ分の姿勢とタイムスタンプが格納されています。
		// （現在時刻 - 目標の遅延時刻）に最も近いタイムスタンプをもつ姿勢を取得したいと思います。
		// 過去の姿勢とタイムスタンプは、this->pastPostures に入っていて、this->pastPostures の型がstd::vector<TimeAndPosture>です。
		// したがって、this->pastPostures[0]～this->pastPostures[timeSeriesBufferSize]のうちから、最も（現在時刻 - 目標の遅延時刻）に
		// 近いタイムスタンプをもつ this->pastPostures の添字( = nearestIndex )を見つけます。
		int nearestIndexTmp = index;
		if (targetDelayTime > 0.0) { // 目標の遅延時間が0秒以下なら何もしません
			if(current.timeStamp > this->targetDelayTime) { // 経過時間が目標の遅延時間未満の時も何もしません

				const double idealPastTime = current.timeStamp - (this->targetDelayTime);

				double nearestTimeDistance = DBL_MAX;
				nearestIndexTmp = 0;


				for (int i = timeSeriesBufferSize - 1; i >= 0; i--) {
					double tmpTimeDistance = abs(idealPastTime - this->pastPostures[i].timeStamp);

					if (tmpTimeDistance < nearestTimeDistance) {
						nearestTimeDistance = tmpTimeDistance;
						nearestIndexTmp = i;
					}
				}

				//std::cout << "nearest: " <<
			}
		}
		const int nearestIndex = nearestIndexTmp; // 上記の「何もしない」場合は、最新の姿勢情報が反転されることになります。

		if (this->reverseMode == reverseModes[RIGHTHAND]) { // 左手の動きを右手へ

			// Get original quaternions.
			double w2, x2, y2, z2, w3, x3, y3, z3, w5, x5, y5, z5; // These are temporary variables.
			this->pastPostures[nearestIndex].posture.joints[LARM_JOINT2].quaternion.getQuaternions(w2, x2, y2, z2);
			this->pastPostures[nearestIndex].posture.joints[LARM_JOINT3].quaternion.getQuaternions(w3, x3, y3, z3);

			// Set reverse quaternions.
			// KinectV2のクォータニオンを使用するようになったことに伴い、y軸（ひねり）が反映されるように
			// なったため、反転する要素が以前と変わりました。
			// 以前は(w,x,y,z)のうちx,y,zの符号を反転させて逆の腕にセットしていましたが、
			// y,zの符号を反転させてセットしています。
			tmpManNiiPosture.joints[RARM_JOINT2].quaternion.setQuaternions(w2, x2, -y2, -z2);
			tmpManNiiPosture.joints[RARM_JOINT3].quaternion.setQuaternions(w3, x3, -y3, -z3);
			//tmpManNiiPosture.joints[RARM_JOINT5].quaternion.setQuaternions(w5, -x5, -y5, -z5);
		}
		else if (this->reverseMode == reverseModes[LEFTHAND]) { // 右手の動きを左手へ
			// Get original quaternions.
			double w2, x2, y2, z2, w3, x3, y3, z3, w5, x5, y5, z5;
			this->pastPostures[nearestIndex].posture.joints[RARM_JOINT2].quaternion.getQuaternions(w2, x2, y2, z2);
			this->pastPostures[nearestIndex].posture.joints[RARM_JOINT3].quaternion.getQuaternions(w3, x3, y3, z3);

			// Set reverse quaternions.
			tmpManNiiPosture.joints[LARM_JOINT2].quaternion.setQuaternions(w2, x2, -y2, -z2);
			tmpManNiiPosture.joints[LARM_JOINT3].quaternion.setQuaternions(w3, x3, -y3, -z3);
			//tmpManNiiPosture.joints[RARM_JOINT5].quaternion.setQuaternions(w5, -x5, -y5, -z5);

		}

//		if (this->reverseMode == reverseModes[RIGHTHAND]) {
//
//			// Get original quaternions.
//			double w2, x2, y2, z2, w3, x3, y3, z3, w5, x5, y5, z5;
//			tmpManNiiPosture.joints[LARM_JOINT2].quaternion.getQuaternions(w2, x2, y2, z2);
//			tmpManNiiPosture.joints[LARM_JOINT3].quaternion.getQuaternions(w3, x3, y3, z3);
//			//tmpManNiiPosture.joints[LARM_JOINT5].quaternion.getQuaternions(w5, x5, y5, z5);
//
//			// Set reverse quaternions.
//			// KinectV2のクォータニオンを使用するようになったことに伴い、y軸（ひねり）が反映されるように
//			// なったため、反転する要素が以前と変わりました。
//			// 以前は(w,x,y,z)のうちx,y,zの符号を反転させて逆の腕にセットしていましたが、
//			// y,zの符号を反転させてセットしています。
//			tmpManNiiPosture.joints[RARM_JOINT2].quaternion.setQuaternions(w2, x2, -y2, -z2);
//			tmpManNiiPosture.joints[RARM_JOINT3].quaternion.setQuaternions(w3, x3, -y3, -z3);
//			//tmpManNiiPosture.joints[RARM_JOINT5].quaternion.setQuaternions(w5, -x5, -y5, -z5);
//		}
//		else if (this->reverseMode = reverseModes[LEFTHAND]) {
//			// Get original quaternions.
//			double w2, x2, y2, z2, w3, x3, y3, z3, w5, x5, y5, z5;
//			tmpManNiiPosture.joints[RARM_JOINT2].quaternion.getQuaternions(w2, x2, y2, z2);
//			tmpManNiiPosture.joints[RARM_JOINT3].quaternion.getQuaternions(w3, x3, y3, z3);
//			//tmpManNiiPosture.joints[LARM_JOINT5].quaternion.getQuaternions(w5, x5, y5, z5);
//
//			// Set reverse quaternions.
//			tmpManNiiPosture.joints[LARM_JOINT2].quaternion.setQuaternions(w2, x2, -y2, -z2);
//			tmpManNiiPosture.joints[LARM_JOINT3].quaternion.setQuaternions(w3, x3, -y3, -z3);
//			//tmpManNiiPosture.joints[RARM_JOINT5].quaternion.setQuaternions(w5, -x5, -y5, -z5);
//
//		}

		SimObj *obj = getObj(myname());
		this->setJointQuaternionsForKinect(obj, &tmpManNiiPosture);

		this->frameNumber++;

	}
}

void MirrorTherapyController::convertMessage2Records(const std::string &message, std::vector<Record> &records)
{
	try {
		const std::string recordDelim = recordDelimDefault;
		const std::string keyDelim = keyDelimDefault;
		const std::string valueDelim = valueDelimDefault;

		//const std::string messageHeader = this->getMessageHeader(message);

	//	bool messageHeaderOK = this->checkMessageHeader(messageHeader);
	//	if (!messageHeaderOK) {
	//		std::cout << "Illegal message header : " << messageHeader << std::endl;
	//	}

		// Split string to records by ";". One record has one KEY and one VALUE.
		std::vector<std::string> recordsString;
		AvatarController::splitString(message, recordDelim, recordsString);

		// Get device type.
		const Record firstRecord = Record(recordsString[0]);

//		if (firstRecord.key != devTypeKey) {
//			std::cout << "Illegal device type: " << firstRecord.key << std::endl;
//			throw("illegal device type");
//		}

		for (int i = 0; i < (int)recordsString.size(); i++) {
			const Record oneRecord = Record(recordsString[i]);
			records.push_back(oneRecord);
		}

	}
	catch (std::exception &ex) {
		std::cout << ex.what() << std::endl;
	}
}


void MirrorTherapyController::setJointQuaternion(SimObj *obj, const ManNiiJointQuaternion &jq)
{
	obj->setJointQuaternion(manNiiJointTypeStr(jq.manJointType).c_str(), jq.quaternion.w, jq.quaternion.x, jq.quaternion.y, jq.quaternion.z);
	//std::cout << manNiiJointTypeStr(jq.manJointType).c_str() << ":" << jq.quaternion.w << "," << jq.quaternion.x <<  "," << jq.quaternion.y <<  "," << jq.quaternion.z << std::endl;
}

void MirrorTherapyController::setJointQuaternions(SimObj *obj)
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

void MirrorTherapyController::setJointQuaternionsForKinect(SimObj *obj, ManNiiPosture *manNiiAvatarPosture)
{
	//Man
	//this->avatarPosture =
	//this->setJointQuaternion(obj, manNiiAvatarPosture->joints[HEAD_JOINT0]);

	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[LARM_JOINT2]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[LARM_JOINT3]);
	//this->setJointQuaternion(obj, manNiiAvatarPosture->joints[LARM_JOINT5]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[RARM_JOINT2]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[RARM_JOINT3]);
	//this->setJointQuaternion(obj, manNiiAvatarPosture->joints[RARM_JOINT5]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[WAIST_JOINT1]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[LLEG_JOINT2]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[LLEG_JOINT4]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[RLEG_JOINT2]);
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[RLEG_JOINT4]);
	this->avatarPosture = manNiiAvatarPosture;
	//double w = obj->getJointAngle("HEAD_JOINT1");
	//std::cout << "Joint angle: " << w << std::endl;
}

void MirrorTherapyController::setJointQuaternionsForOculus(SimObj *obj, ManNiiPosture *manNiiAvatarPosture)
{
	this->setJointQuaternion(obj, manNiiAvatarPosture->joints[HEAD_JOINT0]);
	this->avatarPosture = manNiiAvatarPosture;
}

void MirrorTherapyController::convertRecords2KinectV2JointOrientations(std::vector<Record> &kinectRecords, KinectV2JointOrientation *kinectV2JointOrientations)
{

	try {
		for(int i = 2; i < (int)kinectRecords.size(); i++) {

			std::vector<double> values = AvatarController::splitValues(kinectRecords[i].value);

			if ((int)values.size() > 1) {
				// When VALUE is vector data.
				const KinectV2JointType jt = this->getJointTypeFromKey(kinectRecords[i].key);

				if (jt != KinectV2JointType_Count) {
					KinectV2JointOrientation tmpJO;
					tmpJO.jointType = jt;
					tmpJO.orientation.w = values[0];
					tmpJO.orientation.x = values[1];
					tmpJO.orientation.y = values[2];
					tmpJO.orientation.z = values[3];
					kinectV2JointOrientations[tmpJO.jointType] = tmpJO;
					//std::cout << w << ", " << x << ", " << y << ", " << z << std::endl;
				}
				else if (kinectRecords[i].key == "SpBs_P") {
					// ルートのポジションを入れるときの処理をここに書く
				}
				else {
					std::cout << "ELLEGAL JointType : " << kinectRecords[i].key << std::endl;
					throw("ELLEGAL JointType");
				}
			}
		}
	}
	catch (std::exception &ex) {
		std::cout << ex.what() << std::endl;
	}
}

void MirrorTherapyController::convertKinectV2JointOrientations2ManNiiPosture(KinectV2JointOrientation* kinectV2Joints, ManNiiPosture &manNiiAvatarPosture)
{
	// 野崎からのコメント：
	// 手首（LARM_JOINT5とRARM_JOINT5）と足首（LLEG_JOINT6とRLEG_JOINT6）向けの変換は未実装です。
	// KinectV2が算出する全身のクォータニオンには、手首と足首のクォータニオンが含まれていないためです。
	// サービスプロバイダの方で描画されるウィンドウでは、手首と足首の分も計算されているように見えますが、
	// 表示しているのは関節位置（に点を表示しているだけ）なので、手首と足首のクォータニオンとは無関係です。
	// 手先と足先のポジションを使ってクォータニオンを計算するしか無いと思います（引数が増えます）。

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
	Quaternion q30_con = Quaternion(q1_rot.w, -q1_rot.x, -q1_rot.y, -q1_rot.z);
	Quaternion q30 = Quaternion(kinectV2Joints[2].orientation);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/////////////////////////// Neck quaternion //////////////////////
	q30 = Quaternion::calcCrossProduct(q1_h, q30);
	q30 = Quaternion::calcCrossProduct(q30_con, q30);
	//std::cout << "w, x, y, z:q1_rot  " << q1_rot.w << "," << q1_rot.x <<  "," << q1_rot.y << "," << q1_rot.z << std::endl;
	//std::cout << "w, x, y, z: " << q30.w << "," << q30.x <<  "," << q30.y << "," << q30.z << std::endl;

	////////////////////////////////////////////////////////////////////



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



	manNiiAvatarPosture.joints[WAIST_JOINT1].quaternion = q1_rot;

	manNiiAvatarPosture.joints[LARM_JOINT2].quaternion = q6;
	manNiiAvatarPosture.joints[LARM_JOINT3].quaternion = q22;
//	manNiiAvatarPosture.joints[LARM_JOINT5].quaternion = q28; // Calculating *_JOINT5, it isn't implemented.
	manNiiAvatarPosture.joints[RARM_JOINT2].quaternion = q5;
	manNiiAvatarPosture.joints[RARM_JOINT3].quaternion = q8;
//	manNiiAvatarPosture.joints[RARM_JOINT5].quaternion = q27;


	manNiiAvatarPosture.joints[RLEG_JOINT2].quaternion = q12;
	manNiiAvatarPosture.joints[LLEG_JOINT2].quaternion = q14;
	manNiiAvatarPosture.joints[RLEG_JOINT4].quaternion = q16;
	manNiiAvatarPosture.joints[LLEG_JOINT4].quaternion = q18;


	manNiiAvatarPosture.joints[WAIST_JOINT1].manJointType = WAIST_JOINT1;

	manNiiAvatarPosture.joints[RARM_JOINT2].manJointType = RARM_JOINT2;
	manNiiAvatarPosture.joints[LARM_JOINT2].manJointType = LARM_JOINT2;
	manNiiAvatarPosture.joints[RARM_JOINT3].manJointType = RARM_JOINT3;
	manNiiAvatarPosture.joints[LARM_JOINT3].manJointType = LARM_JOINT3;

	manNiiAvatarPosture.joints[RLEG_JOINT2].manJointType = RLEG_JOINT2;
	manNiiAvatarPosture.joints[LLEG_JOINT2].manJointType = LLEG_JOINT2;
	manNiiAvatarPosture.joints[RLEG_JOINT4].manJointType = RLEG_JOINT4;
	manNiiAvatarPosture.joints[LLEG_JOINT4].manJointType = LLEG_JOINT4;

//	manNiiAvatarPosture.joints[RARM_JOINT5].manJointType = RARM_JOINT5;
//	manNiiAvatarPosture.joints[LARM_JOINT5].manJointType = LARM_JOINT5;

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

EulerAngleType MirrorTherapyController::generateEulerAngleFromRecord(const Record &record)
{
	EulerAngleType eulerAngle;

	try {
		//const std::string eulerRecord = records[2];
		const Record eulerRecord(record);

		if (eulerRecord.key != "EULER") {
			std::cout << "Illegal key :" << eulerRecord.key << std::endl;
			throw(std::exception());
		}

		// Split VALUE to vector by ",", if VALUE is vector.
		std::vector<double> values = AvatarController::splitValues(eulerRecord.value);

		if ((int)values.size() == 1) {
			std::cout << "Not euler angle: " << std::cout << eulerRecord.value << std::endl;
		}
		else if ((int)values.size() > 1) {
			// When VALUE is vector data.

			const float yaw = values[0];
			const float pitch = values[1];
			const float roll = values[2];

			if (yaw == this->prevYaw && pitch == this->prevPitch && roll == this->prevRoll) {
				throw(std::exception());
			}
			else {
				this->prevYaw = yaw;
				this->prevPitch = pitch;
				this->prevRoll = roll;
			}

			eulerAngle.yaw = yaw;
			eulerAngle.pitch = pitch;
			eulerAngle.roll = roll;

		}
	}
	catch (std::exception &ex) {
		std::cout << ex.what() << std::endl;
	}

	return eulerAngle;
}

void MirrorTherapyController::convertEulerAngle2ManNiiPosture(const EulerAngleType &eulerAngle, ManNiiPosture &manNiiAvatarPosture)
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


KinectV2JointType MirrorTherapyController::getJointTypeFromKey(const std::string &key)
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

const bool MirrorTherapyController::isReverseModeOk(const std::string &mode_string)
{
	for (int i = 0; i < ReverseMode_Count; i++) {
		if (mode_string == reverseModes[i]) {
			return true;
		}
	}
	return false;
};

bool MirrorTherapyController::setReverseModeAndDelayTime(const std::string &message)
{
	std::vector<Record> records;
	this->convertMessage2Records(message, records);

	Record firstRecord = records[0];

	if (firstRecord.key == reverseKey) {
		if (this->isReverseModeOk(firstRecord.value)) {
			this->reverseMode = firstRecord.value;
			std::cout << std::endl << "Set reverse mode: " << firstRecord.value << std::endl;
		}
		else {
			std::cout << "Illegal reverse mode: " << firstRecord.value << std::endl;
		}
		return true;
	}
	else if (firstRecord.key == delayKey) {
		//const double delayValue = ::atof(orderValue.c_str());
		if (this->isDelayTimeRangeOk(firstRecord.value)) {
			this->targetDelayTime = atof(firstRecord.value.c_str());
			std::cout << std::endl << "Set target delay time: " << firstRecord.value << std::endl;
		}
		else {
			std::cout << "Illegal delay time: " << firstRecord.value << std::endl;
		}
		return true;
	}
	return false;
}

const double MirrorTherapyController::generateCurrentTimeStamp()
{
	// Get current timestamp.
	struct timeval currentTimeVal;
	gettimeofday(&currentTimeVal, NULL);
	const double currentTimeStamp = (currentTimeVal.tv_sec - this->initTimeVal.tv_sec) * 1000.0 + (currentTimeVal.tv_usec - this->initTimeVal.tv_usec) / 1000.0;
	return currentTimeStamp;
}

const bool MirrorTherapyController::isDelayTimeRangeOk(const double &delayTime)
{
	if (delayTime < 0.0) {
		return false;
	}
	else if (delayTime > DBL_MAX) {
		return false;
	}
	return true;
}

const bool MirrorTherapyController::isDelayTimeRangeOk(const std::string &delayTime)
{
	char *endPtr;

	const double delayValue = ::strtod(delayTime.c_str(), &endPtr);
	if (*endPtr == '\0') {
		return this->isDelayTimeRangeOk(delayValue);
	}
	else {
		return false;
	}
}
