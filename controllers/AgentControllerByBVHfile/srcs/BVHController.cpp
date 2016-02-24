#include "BVHController.h"
#include <math.h>
#include <stdlib.h>

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 ) 

using namespace std;  //利用名前空間の定義

void  BVHController::onInit(InitEvent &evt)
{
	m_readBVH = false;
	m_cnt     = 0;
	m_chNum   = 0;
	m_frNum   = 0;

	//自分の取得
	SimObj *my = getObj(myname());

	//bvhファイル名取得
	string filename = my->getAttr("BvhFile").value().getString();

	//bvhファイル読み込み開始
	readBVH(filename);
}

double BVHController::onAction(ActionEvent &evt)
{
	if(m_readBVH)
	{
		if(m_cnt < m_frNum)
		{
			try
			{
				//自分の取得
				SimObj *my = getObj(myname());

				//BVHデータからエージェントの関節を動かす
				for(int i = 0; i < JOINT_NUM; i++)
				{
					if(m_sj[i] != NULL && m_bj[i] != NULL)
					{
						setMotion(m_sj[i], m_bj[i]);
					}
				}
			}
			catch(SimObj::Exception &)
			{
				;
			}
			m_cnt++;
		}
		else
		{
			if(m_readBVH)
			{
				m_readBVH = false;
				m_cnt = 0;
				deleteBVH();
			}
		}

		return m_interval;
	}
	else
	{
		return 1.0;
	}
} 


bool BVHController::readBVH(string fname)
{
	m_bvh = new BVH(fname.c_str());

	if(m_bvh->IsLoadSuccess())
	{
		LOG_MSG(("Loading BVH file is success"));

		m_interval = m_bvh->GetInterval();   //時間間隔の取得
		LOG_MSG(("Interval : %f",m_interval));
		LOG_MSG(("Joint Num : %d",m_bvh->GetNumJoint())); //Joint数の取得

		m_chNum = m_bvh->GetNumChannel();   //チャンネル数の取得
		LOG_MSG(("Channel Num : %d",m_chNum));

		m_frNum = m_bvh->GetNumFrame();   //フレーム数取得
		LOG_MSG(("Frame Num : %d",m_frNum));

		setJoint();       //SIGVerseエージェントとbvhファイルのJoint対応関係設定

		setDirection();   //体の向きの設定

		setJointOffset(); //ジョイントOffsetの設定
		m_readBVH = true; //bvh読み込みフラグtrue
		return true;
	}
	else
	{
		LOG_MSG(("Load BVH file is failed"));
		return false;
	}
}

bool BVHController::setJoint()
{
	//自分の取得
	SimObj *my = getObj(myname());

	//SIGVerseジョイントとBVHジョイントの対応付け.BVHファイルの関節名はBVHファイルによって異なる
	//BVHジョイント名からBVHジョイント取得。複数指定した場合はその中で検索
	for(int i = 0; i < JOINT_NUM; i++)
	{
		switch (i)
		{
			case HIP:
			{
				m_sj[i]    = new SIGJoint("HIP_JOINT");
				m_bj[i]    = getBVHJoint(1,"Hips");

				//ルートジョイントtrue
				m_sj[i]->rootJoi = true;

				//初期位置設定
				m_sj[i]->iniPos_x = my->x();
				m_sj[i]->iniPos_y = my->y();
				m_sj[i]->iniPos_z = my->z();
				break;
			}
			case WAIST:  //腰
			{
				m_sj[i]   = new SIGJoint("WAIST_JOINT");
				m_bj[i]   = getBVHJoint(1,"Chest");
				break;
			}
			case NECK:   //首
			{
				m_sj[i]   = new SIGJoint("NECK_JOINT");
				m_bj[i]   = getBVHJoint(1,"Neck");
				break;
			}
			case LSHOUL: //左肩
			{
				m_sj[i]   = new SIGJoint("LSHOULDER_JOINT");
				m_bj[i]   = getBVHJoint(2,"LeftUpArm",  "LeftShoulder");
				break;
			}
			case LELB:   //左肘
			{
				m_sj[i]   = new SIGJoint("LELBOW_JOINT");
				m_bj[i]   = getBVHJoint(2,"LeftLowArm", "LeftElbow");
				break;
			}
			case LWRI:   //左手首
			{
				m_sj[i]   = new SIGJoint("LWRIST_JOINT");
				m_bj[i]   = getBVHJoint(2,"LeftHand",   "LeftWrist");
				break;
			}
			case RSHOUL: //右肩
			{
				m_sj[i]   = new SIGJoint("RSHOULDER_JOINT");
				m_bj[i]   = getBVHJoint(2,"RightUpArm", "RightShoulder");
				break;
			}
			case RELB:   //右肘
			{
				m_sj[i]   = new SIGJoint("RELBOW_JOINT");
				m_bj[i]   = getBVHJoint(2,"RightLowArm","RightElbow");
				break;
			}
			case RWRI:   //右手首
			{
				m_sj[i]   = new SIGJoint("RWRIST_JOINT");
				m_bj[i]   = getBVHJoint(2,"RightHand",  "RightWrist");
				break;
			}
			case LHIP:   //左脚付け根
			{
				m_sj[i]   = new SIGJoint("LHIP_JOINT");
				m_bj[i]   = getBVHJoint(2,"LeftUpLeg",  "LeftHip");
				break;
			}
			case LKNEE:  //左膝
			{
				m_sj[i]   = new SIGJoint("LKNEE_JOINT");
				m_bj[i]   = getBVHJoint(2,"LeftLowLeg", "LeftKnee");
				break;
			}
			case LANKLE: //左くるぶし
			{
				m_sj[i]   = new SIGJoint("LANKLE_JOINT");
				m_bj[i]   = getBVHJoint(2,"LeftFoot",   "LeftAnkle");
				break;
			}
			case RHIP:  //右脚付け根
			{
				m_sj[i]   = new SIGJoint("RHIP_JOINT");
				m_bj[i]   = getBVHJoint(2,"RightUpLeg", "RightHip");
				break;
			}
			case RKNEE: //右膝
			{
				m_sj[i]   = new SIGJoint("RKNEE_JOINT");
				m_bj[i]   = getBVHJoint(2,"RightLowLeg","RightKnee");
				break;
			}
			case RANKLE://右くるぶし
			{
				m_sj[i]   = new SIGJoint("RANKLE_JOINT");
				m_bj[i]  = getBVHJoint(2,"RightFoot",  "RightAnkle");
				break;
			}
			default:
			{
				break;
			}
		}
	}
	return true;
}


bool BVHController::setDirection()
{
	//左足付け根のオフセットで向きを判定
	double lh_offx = m_bj[LHIP]->offset[0];
	double lh_offy = m_bj[LHIP]->offset[1];
	double lh_offz = m_bj[LHIP]->offset[2];

	//Z軸が前方方向
	if(lh_offy == 0.0 && lh_offz == 0.0)
	{
		m_type = ZDIR;
		return true;
	}

	//X軸が前方方向
	else if(lh_offx == 0.0 && lh_offy == 0.0)
	{
		m_type = XDIR;
		return true;
	}

	//Z軸が前方方向
	else if(lh_offx > 1.0 && fabs(lh_offy) < 0.1 && fabs(lh_offz) < 0.1)
	{
		m_type = ZDIR;
		return true;
	}

	//X軸が前方方向
	else if(lh_offz < -1.0 && fabs(lh_offx) < 0.1 && fabs(lh_offy) < 0.1)
	{
		m_type = XDIR;
		return true;
	}
	else
	{
		return false;
		LOG_ERR(("cannot read bvh file correctly"));
	}
}


void BVHController::setJointOffset()
{
	//自分の取得
	SimObj *my = getObj(myname());

	//SIGVerseエージェントとbvhの関節の初期角度の角度と回転軸を計算
	Rotation lelb   = calcJointOffset(LELB);
	Rotation relb   = calcJointOffset(RELB);
	Rotation lwri   = calcJointOffset(LWRI);
	Rotation rwri   = calcJointOffset(RWRI);
	Rotation lknee  = calcJointOffset(LKNEE);
	Rotation rknee  = calcJointOffset(RKNEE);
	Rotation lankle = calcJointOffset(LKNEE);
	Rotation rankle = calcJointOffset(RKNEE);

	//SIGVerseエージェントの初期角度設定
	//クオータニオンによる関節の回転
	//最後の引数をtrueにすることにより回転後の関節角度が初期角度になる
	//※最後の引数をtrueにしたときバージョン110223ではbvh読み込み用サンプルモデルで以下の関節を指定したときのみ正常に動作
	my->setJointQuaternion("LSHOULDER_JOINT_OFFSET",lelb.qw(),   lelb.qx(),   lelb.qy(),   lelb.qz(),   true);
	my->setJointQuaternion("RSHOULDER_JOINT_OFFSET",relb.qw(),   relb.qx(),   relb.qy(),   relb.qz(),   true);
	my->setJointQuaternion("LELBOW_JOINT_OFFSET",   lwri.qw(),   lwri.qx(),   lwri.qy(),   lwri.qz(),   true);
	my->setJointQuaternion("RELBOW_JOINT_OFFSET",   rwri.qw(),   rwri.qx(),   rwri.qy(),   rwri.qz(),   true);
	my->setJointQuaternion("LHIP_JOINT_OFFSET",     lknee.qw(),  lknee.qx(),  lknee.qy(),  lknee.qz(),  true);
	my->setJointQuaternion("RHIP_JOINT_OFFSET",     rknee.qw(),  rknee.qx(),  rknee.qy(),  rknee.qz(),  true);
	my->setJointQuaternion("LKNEE_JOINT_OFFSET",    lankle.qw(), lankle.qx(), lankle.qy(), lankle.qz(), true);
	my->setJointQuaternion("RKNEE_JOINT_OFFSET",    rankle.qw(), rankle.qx(), rankle.qy(), rankle.qz(), true);
}

Rotation BVHController::calcJointOffset(JOINT j)
{
	//SIGVerse座標系におけるbvhジョイントOffset取得
	Vector3d bv = getBvhJointOffset(j);

	//SIGVerseの初期関節ベクトル取得
	Vector3d sv = getSigJointOffset(j);

	//内積からSIGVerse初期関節ベクトルとbvh関節Offsetベクトルとの角度を得る
	double angle = acos(sv.angle(bv));

	//外積から回転軸を得る
	Vector3d axis(sv.y()*bv.z() - sv.z()*bv.y(), sv.z()*bv.x() - sv.x()*bv.z(), sv.x()*bv.y() - sv.y()*bv.x());
	Rotation r;

	r.setAxisAndAngle(axis.x(), axis.y(), axis.z(), angle);
	return r;
}

//SIGVerseエージェントの初期ジョイントベクトル取得
Vector3d BVHController::getSigJointOffset(JOINT j)
{
	Vector3d v;

	//関節の場所によって異なる
	if(j == LELB)
	{
		v.x(1.0);
		v.y(0.0);
		v.z(0.0);
	}
	else if(j == RELB)
	{
		v.x(-1.0);
		v.y(0.0);
		v.z(0.0);
	}
	else if(j == LKNEE || j == RKNEE)
	{
		v.x(0.0);
		v.y(-1.0);
		v.z(0.0);
	}
	//親ジョイントのベクトル取得
	else if(j == LWRI)
	{
		v = getBvhJointOffset(LELB);
	}
	else if(j == RWRI)
	{
		v = getBvhJointOffset(RELB);
	}
	else if(j == LANKLE)
	{
		v = getBvhJointOffset(LKNEE);
	}
	else if(j == RANKLE)
	{
		v = getBvhJointOffset(RKNEE);
	}

	return v;
}

//SIGVerseエージェント座標系におけるbvhジョイントのOffsetを取得する
Vector3d BVHController::getBvhJointOffset(JOINT j)
{
	double x, y, z ;

	if(m_type==ZDIR)
	{
		x = m_bj[j]->offset[0];
		y = m_bj[j]->offset[1];
		z = m_bj[j]->offset[2];
	}
	else if(m_type==XDIR)
	{
		z = m_bj[j]->offset[0];
		y = m_bj[j]->offset[1];
		x = -1 * m_bj[j]->offset[2];
	}

	Vector3d vec(x,y,z);
	return vec;
}

//bvhジョイント取得
const BVH::Joint* BVHController::getBVHJoint(int n, ...)
{
	va_list list;
	char *jname;

	va_start(list, n);

	for(int cnt= 0 ; cnt < n; cnt++)
	{
		jname=va_arg(list, char*);
		const BVH::Joint *jo = m_bvh->GetJoint(jname);

		if(jo != NULL){ return jo; }
	}
	LOG_MSG(("cannot get BVH Joint %s", jname));
	return NULL;
}

//motionデータから関節の角度を回転
void BVHController::setMotion(SIGJoint* sjoi, const BVH::Joint* bjoi)
{
	//自分の取得
	SimObj *my = getObj(myname());

	//チャンネル取得
	vector<BVH::Channel*> ch = bjoi->channels;

	//SIGVerseジョイント名取得
	string jname = sjoi->name;

	//ROOT JOINT
	if(sjoi->rootJoi)
	{
		double pos[3];

		for(int i = 0; i < ch.size(); i++)
		{
			//motionデータ取得
			double motion = m_bvh->GetMotion( m_cnt, ch[i]->index );

			//体の向きがｘ軸方向の場合
			if(m_type == XDIR)
			{
				//体全体回転
				if(     ch[i]->type == BVH::Z_ROTATION){ my->setJointAngle((jname + "_X").c_str() ,-DEG2RAD(motion)); }
				else if(ch[i]->type == BVH::X_ROTATION){ my->setJointAngle((jname + "_Z").c_str() ,+DEG2RAD(motion)); }
				else if(ch[i]->type == BVH::Y_ROTATION){ my->setJointAngle((jname + "_Y").c_str() ,+DEG2RAD(motion)); }
				//体の位置データ
				else if(ch[i]->type == BVH::X_POSITION){ pos[2] = +motion; }
				else if(ch[i]->type == BVH::Y_POSITION){ pos[1] = +motion; }
				else if(ch[i]->type == BVH::Z_POSITION){ pos[0] = -motion; }
			}
			//体の向きがz軸方向の場合
			else if (m_type == ZDIR)
			{
				//体全体回転
				if(     ch[i]->type == BVH::Z_ROTATION){ my->setJointAngle((jname + "_Z").c_str() ,DEG2RAD(motion)); }
				else if(ch[i]->type == BVH::X_ROTATION){ my->setJointAngle((jname + "_X").c_str() ,DEG2RAD(motion)); }
				else if(ch[i]->type == BVH::Y_ROTATION){ my->setJointAngle((jname + "_Y").c_str() ,DEG2RAD(motion)); }
				//体の位置データ
				else if(ch[i]->type == BVH::X_POSITION){ pos[0] = motion; }
				else if(ch[i]->type == BVH::Y_POSITION){ pos[1] = motion; }
				else if(ch[i]->type == BVH::Z_POSITION){ pos[2] = motion; }
			}
		}

		//体全体の位置設定
		my->setPosition( sjoi->iniPos_x + pos[0], sjoi->iniPos_y + pos[1], sjoi->iniPos_z + pos[2]);
	}
	//ROOT JOINT以外のJOINT
	else
	{
		for(int i = 0; i < ch.size(); i++)
		{
			//モーションデータ取得
			double motion = m_bvh->GetMotion( m_cnt, ch[i]->index );

			//体の向きがｘ軸方向の場合
			if(m_type == XDIR)
			{
				//体全体回転
				if(     ch[i]->type == BVH::Z_ROTATION){ my->setJointAngle((jname + "_X").c_str() ,-DEG2RAD(motion)); }
				else if(ch[i]->type == BVH::X_ROTATION){ my->setJointAngle((jname + "_Z").c_str() ,+DEG2RAD(motion)); }
				else if(ch[i]->type == BVH::Y_ROTATION){ my->setJointAngle((jname + "_Y").c_str() ,+DEG2RAD(motion)); }
			}
			//体の向きがz軸方向の場合
			else if (m_type == ZDIR)
			{
				if(     ch[i]->type == BVH::Z_ROTATION){ my->setJointAngle((jname + "_Z").c_str() ,DEG2RAD(motion)); }
				else if(ch[i]->type == BVH::X_ROTATION){ my->setJointAngle((jname + "_X").c_str() ,DEG2RAD(motion)); }
				else if(ch[i]->type == BVH::Y_ROTATION){ my->setJointAngle((jname + "_Y").c_str() ,DEG2RAD(motion)); }
			}
		}
	}
}

//bvhデータのクリア
void BVHController::deleteBVH()
{
	for(int n = 0; n < JOINT_NUM; n++)
	{
		delete  m_sj[n];
		delete  m_bj[n];
	}

	delete m_bvh;
}

extern "C"  Controller * createController ()
{
	return new BVHController;
}

