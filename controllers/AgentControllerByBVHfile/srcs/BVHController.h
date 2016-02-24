#include <string>  //stringをインクルード
#include <stdio.h>
#include <iostream>
#include <map>
#include "Controller.h"
#include "Logger.h"
#include "ControllerEvent.h"
#include "Vector3d.h"
#include "BVH.h"
#include <stdarg.h>
#include "Rotation.h"

#define JOINT_NUM 15 //ジョイント数

using namespace std;  //利用名前空間の定義

class SIGJoint
{
 public:
	string name;

	//初期位置
	double iniPos_x;
	double iniPos_y;
	double iniPos_z;

	//rootジョイントの場合はtrue
	bool rootJoi;

	SIGJoint(string str)
	{
		name = str;
		rootJoi = false;
	}
};


class BVHController : public Controller
{
 public:

	enum JOINT
	{
		HIP,
		WAIST,
		NECK,
		LSHOUL,
		LELB,
		LWRI,
		RSHOUL,
		RELB,
		RWRI,
		LHIP,
		LKNEE,
		LANKLE,
		RHIP,
		RKNEE,
		RANKLE
	};

	//体の向き.どの軸を前とするか
	enum TYPE
	{
		ZDIR,
		XDIR
	};

	//初期化
	void onInit(InitEvent &evt);

	//定期的に呼び出される
	double onAction(ActionEvent &evt);

	//BVHファイルの読み込みを開始する
	bool readBVH(string fname);

	//SIGVerseジョイントとBVHジョイントの設定
	bool setJoint();

	//BVHジョイントの取得
	const BVH::Joint* getBVHJoint(int n, ...);

	//初期のエージェントの向き設定
	bool setDirection();

	//ジョイントのオフセット設定
	void setJointOffset();

	//SIGVerseで設定されているジョイント初期角度取得
	Vector3d getSigJointOffset(JOINT j);

	//SIGVerseのエージェント座標におけるbvhジョイントのオフセットを取得
	Vector3d getBvhJointOffset(JOINT j);

	//ジョイントオフセットの計算
	Rotation calcJointOffset(JOINT j);

	//SIGVerseジョイントのmotion設定
	void setMotion(SIGJoint* sjoi, const BVH::Joint* joint);

	//BVHデータの解放
	void deleteBVH();

 private:

	//体の向き
	TYPE       m_type;

	//bvh
	BVH*       m_bvh;

	//時間間隔
	double     m_interval;

	//フレーム数
	int        m_frNum;

	//チャンネル数
	int m_chNum;

	//SIGVerseジョイント
	SIGJoint* m_sj[JOINT_NUM];

	//BVHジョイント
	const BVH::Joint* m_bj[JOINT_NUM];

	//BVH読み込み時のフレームのカウント
	int m_cnt;

	//BVHファイル読み込み中フラグ
	bool m_readBVH;
};

