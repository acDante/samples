/* $Id: SimObj.h,v 1.21 2012-03-27 04:06:44 noma Exp $ */ 
#ifndef SimObj_h
#define SimObj_h

#include <map>
#include <iostream>
#include <vector>

#include "systemdef.h"
#include "SimObjBase.h"
#include "Attribute.h"
#include "CParts.h"
#include "Value.h"

class Vector3d;
#ifdef DEPRECATED
class Command;
#endif

class CommDataEncoder;
class JointForce;
class CommDataResult;

/**
 * @brief �G���e�B�e�B�ɑΉ�����N���X(�N���C�A���g�T�C�h�Ŏg�p)
 * �V�~�����[�V�����T�[�o����Ή�����G���e�B�e�B�̏�����
 * �����A�N���C�A���g�T�C�h�ł����ɃA�N�Z�X�ł���悤�ɂ�
 * ��BSimObj�ɑ΂��鑀��E�������̓V�~�����[�V�����T�[�o��
 * �]������A���̌��ʂ����f�����B
 *
 */
class SimObj : public SimObjBase
{
public:
	//! �p�P�b�g����M�C���^�[�t�F�[�X
	class RequestSender {
	public:
		virtual ~RequestSender() {}
		virtual bool send(CommDataEncoder &) = 0;
		virtual CommDataResult * recv(int bufsize) = 0;
	};
	//! �����}�b�v�^
	typedef std::map<S, CParts*> PartsM;

	//! �R���X�g���N�^
	SimObj();
	//! �f�X�g���N�^
	virtual ~SimObj();

	// added by kawamoto@tome (2011/04/15)
#ifdef WIN32
	//! �p�P�b�g����M�����N���X�C���X�^���X��ݒ肷��
	void setRequestSener(RequestSender *s) {
		m_sender = s;
	}
#else
	//! this is request-- �p�P�b�g����M�����N���X�C���X�^���X��ݒ肷��
	void setRequestSener(RequestSender *s);
	/* {
		m_sender = s;
	}*/
#endif

#ifdef WIN32
	// ++++++++++++++ for test only +++++++++++++++++
	// added by sekikawa(2007/11/30)
	void setId(int id);
	// +++++++ this should be deleted later +++++++++
#endif

	/**
	 * @brief �G���e�B�e�B���\�����镔�i���擾����
	 * @param name ���i��
	 */
	CParts  *getParts(const char *name);

	/**
	 * @brief Joint�������Ȃ��G���e�B�e�B�̃��C���p�[�c���擾����
	 */
	CParts  *getMainParts(){
	  return getParts("body");
	}

	// added by sekikaw(2007/11/30)
	std::map<std::string, CParts *>& getPartsCollection();

	//! ������Ԃ��R�s�[����
	void copy(const SimObj &o);

protected:
	//! �ێ����Ă��铮�I���������������
	void	free_();
	RequestSender * m_sender;

private:
	typedef SimObjBase Super;
	PartsM	m_parts;

  
	class Iterator : public PartsIterator {
	private:
		typedef std::map<std::string, CParts*> M;
	private:
		M &m_map;
		M::iterator m_i;
	public:
		Iterator(M &m) : m_map(m) {
			m_i = m_map.begin();
		}
	private:
		Parts *next() {
			if (m_i == m_map.end()) { return NULL; }
			CParts *p = m_i->second;
			m_i++;
			return p;
		}
	};

public:

	/**
	 * �p�[�c�C�e���[�^���쐬���܂�
	 */
	PartsIterator *getPartsIterator() {
		return new Iterator(m_parts);
	}

	void 	push(Attribute *attr) {
		Super::push(attr);
	}

	//! �p�[�c��ǉ�����
	void	push(CParts *p);

	int setBinary(char *data, int n);
	
	/**
	 * �ԗ��̃_�C�i�~�N�X�𗘗p���邩�ǂ���
	 */
	enum{
		MODE_NOT_USE_WHEEL = 0,	//	�ԗ��_�C�i�~�N�X���g��Ȃ�
		MODE_USE_WHEEL		//	�ԗ��_�C�i�~�N�X���g�p
	};
	/**
	 * �ԗ��͊w���[�h�̃t���O
	 */
	int dynamicsMode;

	////// added by okamoto@tome(2012/08/02)
	/**
	 * @brief �J�������t�����Ă��郊���N���𓾂�
	 * @param camID �擾���邷��J����ID
	 * @return  �����N��
	 */
	std::string getCameraLinkName(int camID = 1);

	/**
	 * @brief �J�����̈ʒu���擾����
	 * @param v �J�����ݒu���ꂽ�����N����̈ʒu(�����N���W�n�j
	 * @param camID �擾����J����ID
	 * @param requestToServer �T�[�o�ɖ₢���킹��ꍇ��true
	 */
	bool getCamPos(Vector3d &pos, int camID = 1, bool requestToServer = true);

	/**
	 * @brief �J�����̈ʒu��ݒ肷��
	 * @param v �J�����ݒu���ꂽ�����N����̈ʒu(�����N���W�n�j
	 * @param camID �ݒ肷��J����ID
	 */
	bool setCamPos(Vector3d pos, int camID = 1);

	/**
	 * @brief �J�����̕���(�����N���W)���擾����
	 * @param v �J�����̌����x�N�g��(�����N�W�n�j
	 * @param camID �擾����J����ID
	 * @param requestToServer �T�[�o�ɖ₢���킹��ꍇ��true
	 */
	bool getCamDir(Vector3d &v, int camID = 1, bool requestToServer = true);

	/**
	 * @brief �J�����̃N�I�[�^�j�I�����擾����
	 * @param v �J�����̌����x�N�g��(�����N�W�n�j
	 * @param camID �擾����J����ID
	 */
	bool getCamQuaternion(double &qw,double &qx, double &qy, double &qz, int camID = 1);

	/**
	 * @brief �J�����̕���(�����N���W)��ݒ肷��
	 * @param v �J�����̌����x�N�g��(�����N�W�n�j
	 * @param camID �ݒ肷��J����ID
	 */
	bool setCamDir(Vector3d v, int camID = 1);

	/**
	 * @brief  �J������y��������p���擾����
	 * @param camID �擾����J����ID
	 * @return ����p(degree)
	 */
	double getCamFOV(int camID = 1);

	/**
	 * @brief  �J������y��������p��ݒ肷��
	 * @param  fov   ����p(degree)
	 * @param  camID �J����ID
	 * @return 
	 */
	bool setCamFOV(double fov, int camID = 1);

	/**
	 * @brief  �J�����̃A�X�y�N�g����擾����
	 * @return camID �ݒ肷��J����ID
	 */
	double getCamAS(int camID = 1);

	/**
	 * @brief  �J�����̃A�X�y�N�g���ݒ肷��
	 * @return camID �ݒ肷��J����ID
	 */
	bool setCamAS(double as, int camID = 1);

	/**
	 * @brief �ݒu���ꂽ�J�����̃����N�����擾����
	 * @return camID �擾����J����ID
	 */
	std::string getCamLink(int camID = 1);

	//[ToDo]
	// �J�����̃����N����ݒ肷��
	void setCamLink(std::string link, int camID = 1);

	//! ���̃G���e�B�e�B�ɂ��grasp����Ă��邩�ǂ������擾����
	bool getIsGrasped();

#ifdef CONTROLLER

public:

	void print();
	/*!
	 * @brief �G���e�B�e�B�̈ʒu��ݒ肵�܂�
	 * @brief Set entity position.
	 * @param x X���W�̈ʒu
	 * @param y Y���W�̈ʒu
	 * @param z Z���W�̈ʒu
	 */
	void setPosition(double x, double y, double z);

	/*!
	 * @brief �G���e�B�e�B�̈ʒu��ݒ肵�܂�
	 * @brief Set entity position.
	 * @param v �ʒu�x�N�g��
	 */
	void setPosition(const Vector3d &v);

	/*!
	 * @brief �G���e�B�e�B�̌�����ݒ肷��
	 *
	 * @param ax ��]���x�N�g����X����
	 * @param ay ��]���x�N�g����Y����
	 * @param az ��]���x�N�g����Z����
	 * @param angle ��]�p�x[rad]
	 */
	void setAxisAndAngle(double ax, double ay, double az, double angle);

	//for Okonomiyaki
        void setAxisAndAngle(double ax, double ay, double az, double angle, double direct);

	
	/*!
	 * �G���e�B�e�B�̌�����ݒ肷��
	 * @param r ��]�s��
	 */
	void setRotation(const Rotation &r);
	
	/*!
	 * @brief �G���e�B�e�B�ɂ�����͂�ݒ肷��(Dynamics ON �ł̂ݎg�p��)
	 * @param fx X������
	 * @param fy Y������
	 * @param fz Z������
	 */
	void setForce(double fx, double fy, double fz);

	/*!
	 * @brief �G���e�B�e�B�ɗ͂�������(Dynamics ON �ł̂ݎg�p��)
	 * @brief Add force to a body using absolute coordinates. 
	 * @param fx �͂�X������
	 * @param fy �͂�Y������
	 * @param fz �͂�Z������
	 */
	void addForce(double fx, double fy, double fz);

	/*!
	 * @brief �G���e�B�e�B�ɉ����́E��p�_(�O���[�o�����W)�̐ݒ�(Dynamics ON �ł̂ݎg�p��)
	 * @brief Add force to a entity using absolute coordinates at specified absolute position. The supplied position vector specifies the point at which the force is supplied in global coordinates. 
	 * @param fx �͂�X������
	 * @param fy �͂�Y������
	 * @param fz �͂�Z������
	 * @param px ��p�_x�l  
	 * @param py ��p�_y�l  
	 * @param pz ��p�_z�l  
	 */
	void addForceAtPos(double fx, double fy, double fz, double px, double py, double pz);

	/*!
	 * @brief �G���e�B�e�B�ɉ����́E��p�_(�G���e�B�e�B����̑��Έʒu)�̐ݒ�(Dynamics ON �ł̂ݎg�p��)
	 * @brief Add force to a entity using absolute coordinates at specified relative position. The supplied position vector specifies the point at which the force is supplied in entity-relative coordinates. 
	 * @param fx �͂�X������
	 * @param fy �͂�Y������
	 * @param fz �͂�Z������
	 * @param px ��p�_x�l  
	 * @param py ��p�_y�l  
	 * @param pz ��p�_z�l  
	 */
	void addForceAtRelPos(double fx, double fy, double fz, double px, double py, double pz);

	/*!
	 * @brief �G���e�B�e�B�ɗ͂�������(�G���e�B�e�B���W�n),(Dynamics ON �ł̂ݎg�p��)
	 * @brief Add force to a entity using relative coordinates. This function takes a force vector that is relative to the entity's own frame of reference.  
	 * @param fx X������ 
	 * @param fy Y������ 
	 * @param fz Z������ 
	 */
	void addRelForce(double fx, double fy, double fz);
	
	/*!
	 * @brief �G���e�B�e�B�ɉ�����(�G���e�B�e�B���W�n)�E��p�_(�O���[�o�����W)�̐ݒ�(Dynamics ON �ł̂ݎg�p��)
	 * @brief Add force to a entity using entity-relative coordinates at specified absolute position. The supplied position vector specifies the point at which the force is supplied in global coordinates. 
	 * @param fx �͂�X������ 
	 * @param fy �͂�Y������
	 * @param fz �͂�Z������
	 * @param px ��p�_x�l 
	 * @param py ��p�_y�l 
	 * @param pz ��p�_z�l 
	 */
	void addRelForceAtPos(double fx, double fy, double fz, double px, double py, double pz);

	/*!
	 * @brief �G���e�B�e�B�ɉ�����(�G���e�B�e�B���W�n)�E��p�_(�G���e�B�e�B���W)�̐ݒ�(Dynamics ON �ł̂ݎg�p��)
	 * @brief Add force to a entity using entity-relative coordinates at specified relative position. The supplied position vector specifies the point at which the force is supplied in entity-relative coordinates. 
	 * @param fx �͂�X������
	 * @param fy �͂�Y������
	 * @param fz �͂�Z������
	 * @param px ��p�_x�l
	 * @param py ��p�_y�l
	 * @param pz ��p�_z�l
	 */
	void addRelForceAtRelPos(double fx, double fy, double fz, double px, double py, double pz);

	/*!
	 * @brief �G�[�W�F���g�̃p�[�c�ɗ͂�������(Dynamics ON �ł̂ݎg�p��)
	 * @brief Add force to a entity parts.
	 * @param parts �p�[�c��
	 * @param fx �͂�X������
	 * @param fy �͂�Y������
	 * @param fz �͂�Z������
	 */
	void addForceToParts(const char* parts, double fx, double fy, double fz);

	/*!
	 * @brief �d�͂̉e�����󂯂邩�ǂ����ݒ肷��
	 * @brief Set Gravity mode.
	 * @param gravity �d�̓t���O
	 */
	void setGravityMode(bool gravity);

	/*!
	 * @brief �d�͂̉e�����󂯂邩�ǂ������擾����
	 * @brief Get Gravity mode.
	 * @retval 0 �ꍇ�d�͂̉e�����󂯂Ȃ��B(Gravity mode on)
	 * @retval 1 �ꍇ�d�͂̉e�����󂯂�B(Gravity mode off)
	 * @retval -1 �擾���s�B(Failed to get gravity mode)
	 */
	int getGravityMode();

	/*!
	 * @brief ���͊w���Z�̗L���܂��͖�����ݒ�
	 * @brief Enable or disalbe gravity mode.
	 * @param dynamics (true�̏ꍇ�L���j
	 */
	void setDynamicsMode(bool dynamics);

	/*!
	 * @brief ���͊w���Z�̗L���܂��͖�����ݒ�
	 * @brief Enable or disalbe dynamics mode.
	 * @return true�̏ꍇ���͊w���Z�L��
	 */
	bool getDynamicsMode();

	/*!
	 * @brief �G���e�B�e�B�̎��ʂ�ݒ肷��
	 * @brief Set the mass of the entity
	 * @param mass ����
	 */
	void setMass(double mass);

	/*!
	 * @brief �G���e�B�e�B�̎��ʂ��擾����
	 * @briefGet the mass of the entity
	 * @return ����
	 */
	double getMass();

	/*!
	 * �G���e�B�e�B�ɉ����x��ݒ肷��
	 * @param ax �����xX������
	 * @param ay �����xY������
	 * @param az �����xZ������
	 */
	void setAccel(double ax, double ay, double az);
	
	/*!
	 * @brief�G���e�B�e�B�ɂ�����g���N��ݒ肷��
	 * @param x X������
	 * @param y Y������
	 * @param z Z������
	 */
	void setTorque(double x, double y, double z);

	/**
	 * @brief �G���e�B�e�B�̑��x��ݒ肷��
	 * @param vx ���x�x�N�g����X������
	 * @param vy ���x�x�N�g����Y������
	 * @param vz ���x�x�N�g����Z������
	 */
	void setVelocity(double vx,double vy,double vz);

	/**
	 * @brief �G���e�B�e�B�̑��x���擾����
	 * @param vec ���x�x�N�g��
	 */
	void getVelocity(Vector3d &vec);

	/**
	 * @brief �G���e�B�e�B�̊p���x��ݒ肷��(Dynamics ON �ł̂ݎg�p��)
	 * @param x X������[rad/s]
	 * @param y Y������[rad/s]
	 * @param z Z������[rad/s]
	 */
	void setAngularVelocity(double x,double y,double z);

	/**
	 * @brief �G���e�B�e�B�̊p���x���擾����(Dynamics ON �ł̂ݎg�p��)
	 * @param vec �p���x�x�N�g��[rad/s]
	 */
	void getAngularVelocity(Vector3d &vec);

	/**
	 * �֐߂̊p�x��ݒ肵�܂�(Dynamics OFF �ł̂ݎg�p��)
	 * @param jointName �֐߂̖��O
	 * @param angle �p�x
	 */
	void setJointAngle(const char *jointName, double angle);

	//added by okamoto@tome (2011/2/17)
	/**
	 * @brief �֐߂̃N�I�[�^�j�I����ݒ肷��(Dynamics OFF �ł̂ݎg�p��)
	 * @param jointName �֐ߖ�
	 * @param qw w����
	 * @param qx x����
	 * @param qy y����
	 * @param qz z����
	 * @param offset true�̏ꍇ��,���̕t�����Ȃǂ̊֐߂̏����p�x��ݒ�ł���
	 */
	void setJointQuaternion(const char *jointName, double qw, double qx, double qy, double qz , bool offset = false);

	//added by okamoto@tome (2011/3/3)
	/**
	 * @brief �֐߂Ƀg���N��������(Dynamics ON �ł̂ݎg�p��)
	 * @param jointName �֐ߖ�
	 * @param t �g���N
	 */
	void addJointTorque(const char *jointName, double t);

	//added by okamoto@tome (2011/3/9)
	/**
	 * @brief �֐߂̊p���x��ݒ肷��(Dynamics ON �ł̂ݎg�p��)
	 * @param jointName �֐ߖ�
	 * @param v �p���x [rad/s]
	 * @param max �ő�g���N 
	 */
	void setJointVelocity(const char *jointName, double v, double max);


	void setObjectVelocity(const char *objectName, double v, double max);

	//added by okamoto@tome (2011/3/9)
	/**
	 * @brief �֐߂̊p�x���擾����
	 * @param jointName �֐ߖ�
	 */
	double getJointAngle(const char *jointName);

	/**
	 * @brief �S�֐߂̊p�x���擾����
	 * @return �֐ߖ��Ɗp�x �}�b�v
	 */
	std::map<std::string, double> getAllJointAngles();

	/**
	 * @brief �֐߂̈ʒu���擾����
	 * @param position �ʒu
	 * @param jointName �֐ߖ�
	 * @retval true  �擾����
	 * @retval false �擾���s
	 */
	bool getJointPosition(Vector3d &pos, const char *jointName);

	/**
	 * @brief �p�[�c�̈ʒu���擾����
	 * @param position �ʒu
	 * @param partsName �֐ߖ�
	 * @retval true  �擾����
	 * @retval false �擾���s
	 */
	bool getPartsPosition(Vector3d &pos, const char *partsName);
	
	/**
	 * @brief �֐߂ɂ������Ă���͂��擾����(dynamics on �Ŏg�p)
	 */
	bool getJointForce(const char *jointName, JointForce &jf1, JointForce &jf2);

	/**
	 * @brief �Փ˔�����s�����ǂ����ݒ肷��
	 */
	void setCollisionEnable(bool flag);


	void	connectJoint(const char *jointName, const char *targetName)
	{
		connectJoint(jointName, NULL, targetName, NULL);
	}

	void	connectJoint(const char *jointName, const char *myParts,
			     const char *targetName, const char *targetParts);
	void	releaseJoint(const char *jointName);

	//! �ʒu�擾
	Vector3d & getPosition(Vector3d &v);

	//! �G���e�B�e�B�̉�]���擾����
	Rotation & getRotation(Rotation &r);

	// fx, fy, fz
	Vector3d & getForce(Vector3d &v);

	Vector3d & getAccel(Vector3d &v);

	// tqx, tqy, tqz
	Vector3d & getTorque(Vector3d &v);

	// lepx, lepy, lepz
	//Vector3d & getLeftViewPoint(Vector3d &v);
	// repx, repy, repz
	//Vector3d & getRightViewPoint(Vector3d &v);


	////////
	////// added by okamoto@tome(2012/01/05)
	/**
	 * @brief �J�����̈ʒu��ݒ肷��(before v2.1.0)
	 * @param v �J�����ݒu���ꂽ�����N����̈ʒu�i�����N���W�n�j
	 * @param camID �ݒ肷��J����ID
	 */
	void setCameraViewPoint(Vector3d v, int camID = 1);

	/**
	 * @brief �J�����̈ʒu���擾����(before v2.1.0)
	 * @param v �J�����ݒu���ꂽ�����N����̈ʒu�i�����N���W�n�j
	 * @param camID �擾����J����ID
	 */
	Vector3d & getCameraViewPoint(Vector3d &v, int camID = 1);

	/**
	 * @brief �J�����̌�����ݒ肷��(before v2.1.0)
	 * @param v �J�����̕����x�N�g��
	 * @param camID �ݒ肷��J����ID
	 */
	void setCameraViewVector(Vector3d v, int camID = 1);

	/**
	 * @brief �J�����̌������擾����(before v2.1.0)
	 * @param v �J�����̕����x�N�g��
	 * @param camID�@�擾����J����ID
	 */
	Vector3d & getCameraViewVector(Vector3d &v, int camID = 1);

	// �J�����̐��𑝂₷ added by yahara@tome (2011/02/14)
	// epx1, epy1, epz1
	Vector3d & getCamera1ViewPoint(Vector3d &v);
	// epx2, epy2, epz2
	Vector3d & getCamera2ViewPoint(Vector3d &v);
	// epx3, epy3, epz3
	Vector3d & getCamera3ViewPoint(Vector3d &v);
	// epx4, epy4, epz4
	Vector3d & getCamera4ViewPoint(Vector3d &v);
	// epx5, epy5, epz5
	Vector3d & getCamera5ViewPoint(Vector3d &v);
	// epx6, epy6, epz6
	Vector3d & getCamera6ViewPoint(Vector3d &v);
	// epx7, epy7, epz7
	Vector3d & getCamera7ViewPoint(Vector3d &v);
	// epx8, epy8, epz8
	Vector3d & getCamera8ViewPoint(Vector3d &v);
	// epx9, epy9, epz9
	Vector3d & getCamera9ViewPoint(Vector3d &v);


	// levx, levy, levz
	//Vector3d & getLeftViewVector(Vector3d &v);
	// revx, revy, revz
	//Vector3d & getRightViewVector(Vector3d &v);
	// �J�����̐��𑝂₷ added by yahara@tome (2011/02/14)
	// evx1, evy1, evz1
	Vector3d & getCamera1ViewVector(Vector3d &v);
	// evx2, evy2, evz2
	Vector3d & getCamera2ViewVector(Vector3d &v);
	// evx3, evy3, evz3
	Vector3d & getCamera3ViewVector(Vector3d &v);
	// evx4, evy4, evz4
	Vector3d & getCamera4ViewVector(Vector3d &v);
	// evx5, evy5, evz5
	Vector3d & getCamera5ViewVector(Vector3d &v);
	// evx6, evy6, evz6
	Vector3d & getCamera6ViewVector(Vector3d &v);
	// evx7, evy7, evz7
	Vector3d & getCamera7ViewVector(Vector3d &v);
	// evx8, evy8, evz8
	Vector3d & getCamera8ViewVector(Vector3d &v);
	// evx9, evy9, evz9
	Vector3d & getCamera9ViewVector(Vector3d &v);

	// levx, levy, levz
	//void setLeftViewVector(const Vector3d &v);
	// revx, revy, revz
	//void setRightViewVector(const Vector3d &v);
	// �J�����̐��𑝂₷ added by yahara@tome (2011/02/14)
	// evx1, evy1, evz1



	void setCamera1ViewVector(const Vector3d &v);
	// evx2, evy2, evz2
	void setCamera2ViewVector(const Vector3d &v);
	// evx3, evy3, evz3
	void setCamera3ViewVector(const Vector3d &v);
	// evx4, evy4, evz4
	void setCamera4ViewVector(const Vector3d &v);
	// evx5, evy5, evz5
	void setCamera5ViewVector(const Vector3d &v);
	// evx6, evy6, evz6
	void setCamera6ViewVector(const Vector3d &v);
	// evx7, evy7, evz7
	void setCamera7ViewVector(const Vector3d &v);
	// evx8, evy8, evz8
	void setCamera8ViewVector(const Vector3d &v);
	// evx9, evy9, evz9
	void setCamera9ViewVector(const Vector3d &v);
	

	////// added by noma@tome(2012/02/20)
	/**
	 * @brief �w�����ꂽ�G���e�B�e�B�̖��O�𓾂�
	 * @param speakerName ���b�҂̖��O
	 * @param lrFlag ��(0)���E(1)��
	 * @param lineID ����̐ݒ� 1�Ȃ�ڂ��n�_,2�Ȃ�I���n�_�Ƃ������I�_�Ƃ�������x�N�g��
	 * @param typicalType 0 �G���e�B�e�B�̃T�C�Y�l���Ȃ�
	 *                    1 �G���e�B�e�B�̃��C���p�[�c�̊O�ڋ����a
	 *                    2 �G���e�B�e�B�̃��C���p�[�c�̑̐ς�3�捪
	 */
	std::vector<const char*> getPointedObject(const char* speakerName, int lrFlag, int lineID, int typicalType=1);

	////// added by okamoto@tome(2012/11/14)
	/**
	 * @brief 2�̊֐߂��Ȃ��x�N�g�����擾����
	 * @param joint1 �n�_�֐� 
	 * @param joint2 �I�_�֐� 
	 */
	bool getPointingVector(Vector3d &vec, const char *joint1, const char *joint2);

	////// added by okamoto@tome(2012/11/14)
	/**
	 * @brief �I���n�_�Ƃ������I�_�Ƃ�������x�N�g��(��΍��W�n)���擾����
	 * @param lrFlag ��(0)���E(1)��
	 */
	bool getPointingVector(Vector3d &vec, int lrFlag = 0);

 private:
	//! �G���e�B�e�B�̉�]���T�[�o�ɑ���(abs == true�̂Ƃ���Ίp�x)
	bool sendEntityQuaternion(const dReal *rot, bool abs);

	bool sendRequest(std::string name, int requestNum);

 public:

	
#ifdef DEPRECATED
	Command * createJointControlCommand();
#endif // DEPRECATED

#endif // CONTROLLER


#ifdef DEPRECATED
private:
	typedef std::map<S, double> JointValueM;
	JointValueM	m_jointValues;
#endif // DEPRECATED


};



/**
 * @brief ���{�b�g�p�I�u�W�F�N�g
 *
 */
class RobotObj : public SimObj
{
 public:

  //! �R���X�g���N�^
 RobotObj() : 
  SimObj(), 
    m_wheelRadius(0.0),
    m_wheelDistance(0.0)
      {}


  /**
   * @brief �ԗւ̐ݒ�(Dynamics OFF �ł̂ݎg�p)
   * @param wheelRadius    �ԗւ̔��a
   * @param wheelDistance  2�̎ԗւ̊Ԋu
   */
  bool setWheel(double wheelRadius, double wheelDistance);

  /**
   * @brief �ԗւ̊p���x�ݒ�(Dynamics OFF �ł̂ݎg�p)
   * @param leftWheel    ���ԗւ̊p���x
   * @param rightWheel   �E�ԗւ̊p���x
   */
  bool setWheelVelocity(double leftWheel, double rightWheel);
  
  

 private:

  double m_wheelRadius;
  double m_wheelDistance;
  
};
#endif // SimObj_h
 
