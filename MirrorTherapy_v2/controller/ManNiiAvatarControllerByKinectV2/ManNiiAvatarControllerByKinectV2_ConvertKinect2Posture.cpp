/*
 * ManNiiAvatarController.cpp
 *
 *  Created on: 2015/06/29
 *      Author: tome-yamada
 *  Separated from ManNiiAvatarControllerByKinectV2.cpp
 */

#include "../../common/device/SensorData.h"
#include "../../common/device/KinectV2SensorData.h"
#include "ManNiiAvatarControllerByKinectV2.h"


ManNiiPosture ManNiiAvatarControllerByKinectV2::convertKinectV2JointOrientations2ManNiiPosture(const KinectV2SensorData::KinectV2JointOrientation* kinectV2Joints)
{
	ManNiiPosture manNiiPosture;

	// TODO: Calculate for Neck, both wrists and both ankles.
	const double coef = 1.0 / sqrt(2.0);

//	Quaternion qSB = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::SpineBase    ].orientation);
	Quaternion qSM = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::SpineMid     ].orientation);
	Quaternion qSS = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::SpineShoulder].orientation);
//	Quaternion qNK = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::Neck         ].orientation);

	Quaternion qRE = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::ElbowRight   ].orientation);
	Quaternion qRW = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::WristRight   ].orientation);
	Quaternion qRH = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::HandRight    ].orientation);
	Quaternion qRK = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::KneeRight    ].orientation);
	Quaternion qRA = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::AnkleRight   ].orientation);

	Quaternion qLE = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::ElbowLeft    ].orientation);
	Quaternion qLW = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::WristLeft    ].orientation);
	Quaternion qLH = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::HandLeft     ].orientation);
	Quaternion qLK = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::KneeLeft     ].orientation);
	Quaternion qLA = Quaternion(kinectV2Joints[KinectV2SensorData::KinectV2JointType::AnkleLeft    ].orientation);

	Quaternion qSM_h   = Quaternion(0.0, 0.0, +1.0, 0.0);
	Quaternion qNK_h   = Quaternion(0.0, 0.0, -1.0, 0.0);

	Quaternion qSM_rot = Quaternion::calcCrossProduct(qSM_h, qSM);
//	Quaternion qNK_rot = Quaternion::calcCrossProduct(qSS, qNK_h);
	Quaternion qSS_rot = Quaternion::calcCrossProduct(qSM_h, qSS);

	Quaternion qSM_con = Quaternion(qSS_rot.w, -qSS_rot.x, -qSS_rot.y, -qSS_rot.z);
//	Quaternion q30_con = Quaternion(qSM_rot.w, -qSM_rot.x, -qSM_rot.y, -qSM_rot.z);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////


//	//////////////  Neck quaternion  ///////////////////////////////////
//	qNK = Quaternion::calcCrossProduct(qSM_h, qNK);
//	qNK = Quaternion::calcCrossProduct(q30_con, qNK);
//	////////////////////////////////////////////////////////////////////


	//////////////  RARM_LINK2 quaternion  ///////////////////////////////////
	Quaternion qRA2;
	Quaternion qRA2_x = Quaternion(coef, -coef, 0.0, 0.0);
//	Quaternion qRA2_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion qRA2_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion qRA2_rotx = Quaternion::calcCrossProduct(qRA2_x, qRE);
	qRA2.w = qRA2_rotx.w;
	qRA2.x = qRA2_rotx.y;
	qRA2.y = qRA2_rotx.z;
	qRA2.z = qRA2_rotx.x;
	qRA2 = Quaternion::calcCrossProduct(qNK_h, qRA2);
	qRA2 = Quaternion::calcCrossProduct(qRA2_y, qRA2);
	qRA2.w = +qRA2.w;
	qRA2.x = -qRA2.x;
	qRA2.y = -qRA2.y;
	qRA2.z = +qRA2.z;
	qRA2 = Quaternion::calcCrossProduct(qSM_con, qRA2);
	//////////////////////////////////////////////////////////////////////

	//////////////  RARM_LINK4 quaternion  ///////////////////////////////////
	Quaternion qRA2_con;
	qRA2_con.w = +qRA2.w;
	qRA2_con.x = -qRA2.x;
	qRA2_con.y = -qRA2.y;
	qRA2_con.z = -qRA2.z;

	Quaternion qRA4;
	Quaternion qRA4_x = Quaternion(coef, -coef, 0.0, 0.0);
//	Quaternion qRA4_z = Quaternion(coef, 0.0, 0.0, coef);
	Quaternion qRA4_y = Quaternion(coef, 0.0, coef, 0.0);
	Quaternion qRA4_rotx = Quaternion::calcCrossProduct(qRA4_x, qRW);
	qRA4.w = qRA4_rotx.w;
	qRA4.x = qRA4_rotx.y;
	qRA4.y = qRA4_rotx.z;
	qRA4.z = qRA4_rotx.x;
	qRA4 = Quaternion::calcCrossProduct(qRA4_y, qRA4);
	Quaternion qRA4_va = Quaternion(qRA4.w, qRA4.x, qRA4.y, qRA4.z);
	qRA4 = Quaternion::calcCrossProduct(qNK_h, qRA4);
	qRA4.w = +qRA4_va.w;
	qRA4.x = -qRA4_va.x;
	qRA4.y = -qRA4_va.y;
	qRA4.z = +qRA4_va.z;
	qRA4 = Quaternion::calcCrossProduct(qRA2_con, qRA4);
	////////////////////////////////////////////////////////////////////////

//	//////////////  RARM_LINK7 quaternion  ///////////////////////////////////
//	Quaternion qRA4_con;
//	qRA4_con.w = +qRA4.w;
//	qRA4_con.x = -qRA4.x;
//	qRA4_con.y = -qRA4.y;
//	qRA4_con.z = -qRA4.z;
//
//	Quaternion qRA7;
//	Quaternion qRA7_x = Quaternion(coef, -coef, 0.0, 0.0);
////	Quaternion qRA7_z = Quaternion(coef, 0.0, 0.0, coef);
//	Quaternion qRA7_y = Quaternion(coef, 0.0, -coef, 0.0);
//	Quaternion qRA7_rotx = Quaternion::calcCrossProduct(qRA7_x, qRH);
//	qRA7.w = qRA7_rotx.w;
//	qRA7.x = qRA7_rotx.y;
//	qRA7.y = qRA7_rotx.z;
//	qRA7.z = qRA7_rotx.x;
//	qRA7 = Quaternion::calcCrossProduct(qRA7_y, qRA7);
//	Quaternion qRA7_va = Quaternion(qRA7.w, qRA7.x, qRA7.y, qRA7.z);
//	qRA7 = Quaternion::calcCrossProduct(qNK_h, qRA7);
//	qRA7.w = qRA7_va.w;
//	qRA7.x = qRA7_va.x;
//	qRA7.y = qRA7_va.y;
//	qRA7.z = qRA7_va.z;
//	qRA7 = Quaternion::calcCrossProduct(qRA4_con, qRA7);
//	//////////////////////////////////////////////////////////////////////////


	//////////////  LARM_LINK2 quaternion  ///////////////////////////////////
	Quaternion qLA2;
	Quaternion qLA2_x = Quaternion(coef, coef, 0.0, 0.0);
//	Quaternion qLA2_z = Quaternion(coef, 0.0, 0.0, coef);
	Quaternion qLA2_y = Quaternion(coef, 0.0, coef, 0.0);
	Quaternion qLA2_rotx = Quaternion::calcCrossProduct(qLA2_x, qLE);
	qLA2.w = qLA2_rotx.w;
	qLA2.x = qLA2_rotx.y;
	qLA2.y = qLA2_rotx.z;
	qLA2.z = qLA2_rotx.x;
	qLA2 = Quaternion::calcCrossProduct(qNK_h, qLA2);
	qLA2 = Quaternion::calcCrossProduct(qLA2_y, qLA2);
	qLA2.w = +qLA2.w;
	qLA2.x = +qLA2.x;
	qLA2.y = +qLA2.y;
	qLA2.z = +qLA2.z;
	qLA2 = Quaternion::calcCrossProduct(qSM_con, qLA2);
	//////////////////////////////////////////////////////////////////////////

	//////////////  LARM_LINK4 quaternion  ///////////////////////////////////
	Quaternion qLA2_con;
	qLA2_con.w = +qLA2.w;
	qLA2_con.x = -qLA2.x;
	qLA2_con.y = -qLA2.y;
	qLA2_con.z = -qLA2.z;

	Quaternion qLA4;
	Quaternion qLA4_x = Quaternion(coef, -coef, 0.0, 0.0);
//	Quaternion qLA4_z = Quaternion(coef, 0.0, 0.0, coef);
	Quaternion qLA4_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion qLA4_rotx = Quaternion::calcCrossProduct(qLA4_x, qLW);
	qLA4.w = qLA4_rotx.w;
	qLA4.x = qLA4_rotx.y;
	qLA4.y = qLA4_rotx.z;
	qLA4.z = qLA4_rotx.x;
	qLA4 = Quaternion::calcCrossProduct(qLA4_y, qLA4);
	Quaternion qLA4_va = Quaternion(qLA4.w, qLA4.x, qLA4.y, qLA4.z);
	qLA4 = Quaternion::calcCrossProduct(qNK_h, qRA4);
	qLA4.w = +qLA4_va.w;
	qLA4.x = +qLA4_va.x;
	qLA4.y = -qLA4_va.y;
	qLA4.z = -qLA4_va.z;
	qLA4 = Quaternion::calcCrossProduct(qLA2_con, qLA4);
	//////////////////////////////////////////////////////////////////////////

//	//////////////  LARM_LINK7 quaternion  ///////////////////////////////////
//	Quaternion qLA4_con;
//	qLA4_con.w = +qLA4.w;
//	qLA4_con.x = -qLA4.x;
//	qLA4_con.y = -qLA4.y;
//	qLA4_con.z = -qLA4.z;
//
//	Quaternion qLA7;
//	Quaternion qLA7_x = Quaternion(coef, -coef, 0.0, 0.0);
////	Quaternion qLA7_z = Quaternion(coef, 0.0, 0.0, coef);
//	Quaternion qLA7_y = Quaternion(coef, 0.0, -coef, 0.0);
//	Quaternion qLA7_rotx = Quaternion::calcCrossProduct(qLA7_x, qLH);
//	qLA7.w = qLA7_rotx.w;
//	qLA7.x = qLA7_rotx.y;
//	qLA7.y = qLA7_rotx.z;
//	qLA7.z = qLA7_rotx.x;
//	qLA7 = Quaternion::calcCrossProduct(qLA7_y, qLA7);
//	Quaternion qLA7_va = Quaternion(qLA7.w, qLA7.x, qLA7.y, qLA7.z);
//	qLA7 = Quaternion::calcCrossProduct(qNK_h, qLA7);
//	qLA7.w = +qLA7_va.w;
//	qLA7.x = +qLA7_va.x;
//	qLA7.y = -qLA7_va.y;
//	qLA7.z = -qLA7_va.z;
//	qLA7 = Quaternion::calcCrossProduct(qLA4_con, qLA7);
//	//////////////////////////////////////////////////////////////////////////


	//////////////  RLEG_LINK2 quaternion  ///////////////////////////////////
	Quaternion qRL2;
//	Quaternion qRL2_x = Quaternion(coef, -coef, 0.0, 0.0);
//	Quaternion qRL2_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion qRL2_y = Quaternion(coef, 0.0, coef, 0.0);
	Quaternion qRL2_h = Quaternion(0.0, 0.0, 0.0, -1.0);
	qRL2 = Quaternion::calcCrossProduct(qRL2_h, qRK);
	Quaternion qRL2_b = Quaternion::calcCrossProduct(qRL2_y, qRL2);
	qRL2.w = +qRL2_b.w;
	qRL2.x = -qRL2_b.z;
	qRL2.y = -qRL2_b.y;
	qRL2.z = -qRL2_b.x;
	//////////////////////////////////////////////////////////////////////

	//////////////  RLEG_LINK4 quaternion  ///////////////////////////////////
	Quaternion  qRL2_con;
	qRL2_con.w = +qRL2.w;
	qRL2_con.x = -qRL2.x;
	qRL2_con.y = -qRL2.y;
	qRL2_con.z = -qRL2.z;

	Quaternion qRL4;
//	Quaternion qRL4_x = Quaternion(coef, -coef, 0.0, 0.0);
//	Quaternion qRL4_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion qRL4_y = Quaternion(coef, 0.0, coef, 0.0);
	Quaternion qRL4_h = Quaternion(0.0, 0.0, 0.0, -1.0);
	qRL4 = Quaternion::calcCrossProduct(qRL4_h, qRA);
	Quaternion qRL4_b = Quaternion::calcCrossProduct(qRL4_y, qRL4);
	qRL4.w = +qRL4_b.w;
	qRL4.x = -qRL4_b.z;
	qRL4.y = -qRL4_b.y;
	qRL4.z = -qRL4_b.x;
	qRL4 = Quaternion::calcCrossProduct(qRL2_con, qRL4);
	//////////////////////////////////////////////////////////////////////


	//////////////  LLEG_LINK2 quaternion  ///////////////////////////////////
	Quaternion qLL2;
//	Quaternion qLL2_x = Quaternion(coef, -coef, 0.0, 0.0);
//	Quaternion qLL2_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion qLL2_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion qLL2_h = Quaternion(0.0, 0.0, 0.0, 1.0);
	qLL2 = Quaternion::calcCrossProduct(qLL2_h, qLK);
	Quaternion qLL2_b = Quaternion::calcCrossProduct(qLL2_y, qLL2);
	qLL2.w = +qLL2_b.w;
	qLL2.x = +qLL2_b.z;
	qLL2.y = -qLL2_b.y;
	qLL2.z = +qLL2_b.x;
	//////////////////////////////////////////////////////////////////////

	//////////////  LLEG_LINK4 quaternion  ///////////////////////////////////
	Quaternion  qLL2_con;
	qLL2_con.w = +qLL2.w;
	qLL2_con.x = -qLL2.x;
	qLL2_con.y = -qLL2.y;
	qLL2_con.z = -qLL2.z;

	Quaternion qLL4;
//	Quaternion qLL4_x = Quaternion(coef, -coef, 0.0, 0.0);
//	Quaternion qLL4_z = Quaternion(coef, 0.0, 0.0, -coef);
	Quaternion qLL4_y = Quaternion(coef, 0.0, -coef, 0.0);
	Quaternion qLL4_h = Quaternion(0.0, 0.0, 0.0, 1.0);
	qLL4 = Quaternion::calcCrossProduct(qLL4_h, qLA);
	Quaternion qLL4_b = Quaternion::calcCrossProduct(qLL4_y, qLL4);
	qLL4.w = +qLL4_b.w;
	qLL4.x = +qLL4_b.z;
	qLL4.y = -qLL4_b.y;
	qLL4.z = +qLL4_b.x;
	qLL4 = Quaternion::calcCrossProduct(qLL2_con, qLL4);
	//////////////////////////////////////////////////////////////////////


	/*
	 * ManNiiAvatarPosture posture;
	 */
	manNiiPosture.joint[ManNiiPosture::WAIST_JOINT1].quaternion = qSM_rot;
	//manNiiPosture.joint[ManNiiPosture::HEAD_JOINT0].quaternion = q30;

	manNiiPosture.joint[ManNiiPosture::RARM_JOINT2].quaternion = qRA2;
	manNiiPosture.joint[ManNiiPosture::LARM_JOINT2].quaternion = qLA2;
	manNiiPosture.joint[ManNiiPosture::RARM_JOINT3].quaternion = qRA4;
	manNiiPosture.joint[ManNiiPosture::LARM_JOINT3].quaternion = qLA4;
	//	this->posture.joint[ManNiiPosture::RARM_JOINT5].quaternion = qRA7;
	//	this->posture.joint[ManNiiPosture::LARM_JOINT5].quaternion = qLA7;

	manNiiPosture.joint[ManNiiPosture::RLEG_JOINT2].quaternion = qRL2;
	manNiiPosture.joint[ManNiiPosture::LLEG_JOINT2].quaternion = qLL2;
	manNiiPosture.joint[ManNiiPosture::RLEG_JOINT4].quaternion = qRL4;
	manNiiPosture.joint[ManNiiPosture::LLEG_JOINT4].quaternion = qLL4;

	return manNiiPosture;
}


/*
 * SIGVerse送信用動作情報電文を作成する
 */
ManNiiPosture ManNiiAvatarControllerByKinectV2::convertKinectV2JointPosition2ManNiiPosture(const KinectV2SensorData::KinectV2JointPosition* positionArray)
{
	ManNiiPosture manNiiPosture;

	Quaternion q_waist, q_waist_joint1;
	Quaternion q_head_joint1;
	Quaternion q_rarm_joint2, q_rarm_joint3, q_rarm_joint5;
	Quaternion q_larm_joint2, q_larm_joint3, q_larm_joint5;
	Quaternion q_rleg_joint2, q_rleg_joint4, q_rleg_joint6;
	Quaternion q_lleg_joint2, q_lleg_joint4, q_lleg_joint6;

	try
	{
		SigCmn::Vector3 spineBase, torso, neck, head,
			lshoul, lelb, lwrist, lhand, rshoul, relb, rwrist, rhand,
			lhip, lknee, lankle, lfoot, rhip, rknee, rankle, rfoot;

		spineBase = positionArray[KinectV2SensorData::KinectV2JointType::SpineBase].position;
		torso     = positionArray[KinectV2SensorData::KinectV2JointType::SpineMid].position;
		neck      = positionArray[KinectV2SensorData::KinectV2JointType::Neck].position;
		head      = positionArray[KinectV2SensorData::KinectV2JointType::Head].position;

		lshoul = positionArray[KinectV2SensorData::KinectV2JointType::ShoulderLeft].position;
		lelb   = positionArray[KinectV2SensorData::KinectV2JointType::ElbowLeft].position;
		lwrist = positionArray[KinectV2SensorData::KinectV2JointType::WristLeft].position;
		lhand  = positionArray[KinectV2SensorData::KinectV2JointType::HandLeft].position;

		rshoul = positionArray[KinectV2SensorData::KinectV2JointType::ShoulderRight].position;
		relb   = positionArray[KinectV2SensorData::KinectV2JointType::ElbowRight].position;
		rwrist = positionArray[KinectV2SensorData::KinectV2JointType::WristRight].position;
		rhand  = positionArray[KinectV2SensorData::KinectV2JointType::HandRight].position;

		lhip   = positionArray[KinectV2SensorData::KinectV2JointType::HipLeft].position;
		lknee  = positionArray[KinectV2SensorData::KinectV2JointType::KneeLeft].position;
		lankle = positionArray[KinectV2SensorData::KinectV2JointType::AnkleLeft].position;
		lfoot  = positionArray[KinectV2SensorData::KinectV2JointType::FootLeft].position;

		rhip   = positionArray[KinectV2SensorData::KinectV2JointType::HipRight].position;
		rknee  = positionArray[KinectV2SensorData::KinectV2JointType::KneeRight].position;
		rankle = positionArray[KinectV2SensorData::KinectV2JointType::AnkleRight].position;
		rfoot  = positionArray[KinectV2SensorData::KinectV2JointType::FootRight].position;


		/*
		 * SIGVerse送信用動作情報電文を作成し、MAPに保持する
		 */
		//右ヒップと左ヒップの位置から体全体の回転を求める
		SigCmn::Vector3 khip_vec;

		if (SigCmn::diffVec(khip_vec, rhip, lhip))
		{
			q_waist = Quaternion::calcQuaternionFromVector(this->getSigVec(SigVec::HIP), khip_vec);
			Quaternion rrootq = Quaternion::calcQuaternionFromVector(khip_vec, this->getSigVec(SigVec::HIP));

			//腰
			SigCmn::Vector3 kwaist_vec;

			if (SigCmn::diffVec(kwaist_vec, spineBase, neck))
			{
				//体全体の回転による座標変換
				Quaternion::rotVec(kwaist_vec, rrootq);

				SigCmn::Vector3 swaist_vec = this->getSigVec(WAIST);

				q_waist_joint1 = Quaternion::calcQuaternionFromVector(swaist_vec, kwaist_vec);

				Quaternion rwaist = Quaternion::calcQuaternionFromVector(kwaist_vec, swaist_vec); //逆回転

				//首
				SigCmn::Vector3 kneck_vec;
				if (SigCmn::diffVec(kneck_vec, neck, head))
				{
					//体全体と腰の回転による座標変換
					Quaternion::rotVec(kneck_vec, rrootq);
					Quaternion::rotVec(kneck_vec, rwaist);

					//クォータニオン計算
					q_head_joint1 = Quaternion::calcQuaternionFromVector(swaist_vec, kneck_vec);
				}

				//右肩
				SigCmn::Vector3 krsh_vec;
				if (SigCmn::diffVec(krsh_vec, rshoul, relb))
				{
					SigCmn::Vector3 srsh_vec = this->getSigVec(RSHOULDER);

					//体全体と腰の回転による座標変換
					Quaternion::rotVec(krsh_vec, rrootq);
					Quaternion::rotVec(krsh_vec, rwaist);

					//クォータニオン計算
					q_rarm_joint2 = Quaternion::calcQuaternionFromVector(srsh_vec, krsh_vec);
					Quaternion rrsh = Quaternion::calcQuaternionFromVector(krsh_vec, srsh_vec); //逆回転

					//右ひじ
					SigCmn::Vector3 krel_vec;
					if (SigCmn::diffVec(krel_vec, relb, rwrist))
					{
						//体全体と腰の回転による座標変換
						Quaternion::rotVec(krel_vec, rrootq);
						Quaternion::rotVec(krel_vec, rwaist);
						Quaternion::rotVec(krel_vec, rrsh);

						//クォータニオン計算
						q_rarm_joint3 = Quaternion::calcQuaternionFromVector(srsh_vec, krel_vec);
						Quaternion rrel = Quaternion::calcQuaternionFromVector(krel_vec, srsh_vec); //逆回転

						//右手首
						SigCmn::Vector3 krwrist_vec;
						if (SigCmn::diffVec(krwrist_vec, rwrist, rhand))
						{
							//体全体と腰の回転による座標変換
							Quaternion::rotVec(krwrist_vec, rrootq);
							Quaternion::rotVec(krwrist_vec, rwaist);
							Quaternion::rotVec(krwrist_vec, rrsh);
							Quaternion::rotVec(krwrist_vec, rrel);

							//クォータニオン計算
							q_rarm_joint5 = Quaternion::calcQuaternionFromVector(srsh_vec, krwrist_vec);
						}
					}
				}

				//左肩
				SigCmn::Vector3 klsh_vec;
				if (SigCmn::diffVec(klsh_vec, lshoul, lelb))
				{
					SigCmn::Vector3 slsh_vec = this->getSigVec(LSHOULDER);

					//体全体と腰の回転による座標変換
					Quaternion::rotVec(klsh_vec, rrootq);
					Quaternion::rotVec(klsh_vec, rwaist);

					//クォータニオン計算
					q_larm_joint2 = Quaternion::calcQuaternionFromVector(slsh_vec, klsh_vec);
					Quaternion rlsh = Quaternion::calcQuaternionFromVector(klsh_vec, slsh_vec); //逆回転

					//左ひじ
					SigCmn::Vector3 klel_vec;
					if (SigCmn::diffVec(klel_vec, lelb, lwrist))
					{
						//体全体と腰の回転による座標変換
						Quaternion::rotVec(klel_vec, rrootq);
						Quaternion::rotVec(klel_vec, rwaist);
						Quaternion::rotVec(klel_vec, rlsh);

						//クォータニオン計算
						q_larm_joint3 = Quaternion::calcQuaternionFromVector(slsh_vec, klel_vec);
						Quaternion rlel = Quaternion::calcQuaternionFromVector(klel_vec, slsh_vec); //逆回転

						//左手首
						SigCmn::Vector3 klwrist_vec;
						if (SigCmn::diffVec(klwrist_vec, lwrist, lhand))
						{
							//体全体と腰の回転による座標変換
							Quaternion::rotVec(klwrist_vec, rrootq);
							Quaternion::rotVec(klwrist_vec, rwaist);
							Quaternion::rotVec(klwrist_vec, rlsh);
							Quaternion::rotVec(klwrist_vec, rlel);

							//クォータニオン計算
							q_larm_joint5 = Quaternion::calcQuaternionFromVector(slsh_vec, klwrist_vec);
						}
					}
				}
			}

			SigCmn::Vector3 sleg_vec = this->getSigVec(LEG);
			SigCmn::Vector3 sfoot_vec = this->getSigVec(FOOT);

			//右足付け根
			SigCmn::Vector3 krhip_vec;
			if (SigCmn::diffVec(krhip_vec, rhip, rknee))
			{
				//キネクトにおけるベクトルを親関節により座標変換
				Quaternion::rotVec(krhip_vec, rrootq);

				//クォータニオン計算
				q_rleg_joint2 = Quaternion::calcQuaternionFromVector(sleg_vec, krhip_vec);
				Quaternion rrhp = Quaternion::calcQuaternionFromVector(krhip_vec, sleg_vec);

				//右ひざ
				SigCmn::Vector3 krknee_vec;
				if (SigCmn::diffVec(krknee_vec, rknee, rankle))
				{
					//キネクトにおけるベクトルを親関節により座標変換
					Quaternion::rotVec(krknee_vec, rrootq);
					Quaternion::rotVec(krknee_vec, rrhp);

					//クォータニオン計算
					q_rleg_joint4 = Quaternion::calcQuaternionFromVector(sleg_vec, krknee_vec);
					Quaternion rrknee = Quaternion::calcQuaternionFromVector(krknee_vec, sleg_vec); //逆回転

					//右足首
					SigCmn::Vector3 krankle_vec;
					if (SigCmn::diffVec(krankle_vec, rankle, rfoot))
					{
						//体全体と腰の回転による座標変換
						Quaternion::rotVec(krankle_vec, rrootq);
						Quaternion::rotVec(krankle_vec, rrhp);
						Quaternion::rotVec(krankle_vec, rrknee);

						//クォータニオン計算
						q_rleg_joint6 = Quaternion::calcQuaternionFromVector(sfoot_vec, krankle_vec);
					}
				}
			}

			//左足付け根
			SigCmn::Vector3 klhip_vec;
			if (SigCmn::diffVec(klhip_vec, lhip, lknee))
			{
				//キネクトにおけるベクトルを親関節により座標変換
				Quaternion::rotVec(klhip_vec, rrootq);

				//クォータニオン計算
				q_lleg_joint2 = Quaternion::calcQuaternionFromVector(sleg_vec, klhip_vec);
				Quaternion rlhp = Quaternion::calcQuaternionFromVector(klhip_vec, sleg_vec); //逆回転

				//左ひざ
				SigCmn::Vector3 klknee_vec;
				if (SigCmn::diffVec(klknee_vec, lknee, lankle))
				{
					//キネクトにおけるベクトルを親関節により座標変換
					Quaternion::rotVec(klknee_vec, rrootq);
					Quaternion::rotVec(klknee_vec, rlhp);

					//クォータニオン計算
					q_lleg_joint4 = Quaternion::calcQuaternionFromVector(sleg_vec, klknee_vec);
					Quaternion rlknee = Quaternion::calcQuaternionFromVector(klknee_vec, sleg_vec); //逆回転

					//左足首
					SigCmn::Vector3 klankle_vec;
					if (SigCmn::diffVec(klankle_vec, lankle, lfoot))
					{
						//体全体と腰の回転による座標変換
						Quaternion::rotVec(klankle_vec, rrootq);
						Quaternion::rotVec(klankle_vec, rlhp);
						Quaternion::rotVec(klankle_vec, rlknee);

						//クォータニオン計算
						q_lleg_joint6 = Quaternion::calcQuaternionFromVector(sfoot_vec, klankle_vec);
					}
				}
			}

			//ManNiiAvatarPosture posture;
			manNiiPosture.joint[ManNiiPosture::ROOT_JOINT0].quaternion = q_waist;
			manNiiPosture.joint[ManNiiPosture::WAIST_JOINT1].quaternion = q_waist_joint1;

			manNiiPosture.joint[ManNiiPosture::RARM_JOINT2].quaternion = q_rarm_joint2;
			manNiiPosture.joint[ManNiiPosture::LARM_JOINT2].quaternion = q_larm_joint2;
			manNiiPosture.joint[ManNiiPosture::RARM_JOINT3].quaternion = q_rarm_joint3;
			manNiiPosture.joint[ManNiiPosture::LARM_JOINT3].quaternion = q_larm_joint3;

			manNiiPosture.joint[ManNiiPosture::RLEG_JOINT2].quaternion = q_rleg_joint2;
			manNiiPosture.joint[ManNiiPosture::LLEG_JOINT2].quaternion = q_lleg_joint2;
			manNiiPosture.joint[ManNiiPosture::RLEG_JOINT4].quaternion = q_rleg_joint4;
			manNiiPosture.joint[ManNiiPosture::LLEG_JOINT4].quaternion = q_lleg_joint4;
		}
	}
	catch (std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
	}

	return manNiiPosture;
}
