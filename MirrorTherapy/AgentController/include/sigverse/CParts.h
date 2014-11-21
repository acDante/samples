/* $Id: CParts.h,v 1.4 2011-05-12 08:33:50 msi Exp $ */ 
#ifndef CParts_h
#define CParts_h

#include "systemdef.h"
#include "Parts.h"
//#include "SimObj.h"

/**
 * @brief �G���e�B�e�B�̕��i�N���X(�N���C�A���g�T�C�h�Ŏg�p)
 *
 * @see SParts
 */
class CParts : public Parts
{
protected:
	/**
	 * @brief �R���X�g���N�^
	 *
	 * @param t ���i�^�C�v
	 * @param name ���i��
	 * @param pos ���i�̒��S�ʒu
	 */
	CParts(PartsType t, const char *name, const Position &pos)
		: Parts(t, name, pos) {}

	//! �R�s�[�R���X�g���N�^
	CParts(const CParts &o)
		: Parts(o) {}

  SOCKET m_sock;
  
  // �p�[�c���L�I�u�W�F�N�g��
  std::string m_owner;
private:
	void 	setQuaternion(dReal q0, dReal q1, dReal q2, dReal q3)
	{
		m_rot.setQuaternion(q0, q1, q2, q3);
	}
public:
	//! �f�X�g���N�^
	virtual ~CParts() {}
#ifdef CONTROLLER
	bool getPosition(Vector3d &v);
	Rotation & getRotation(Rotation &r);
#endif
	//! �����̕��i�N���X�I�u�W�F�N�g���쐬����
	virtual CParts * clone() = 0;

	const dReal * getPosition();
	const dReal * getRotation();
	const dReal * getQuaternion();


	//! �T�[�o���M�p�\�P�b�g
	void setSocket(SOCKET sock)
	{
		m_sock = sock;
	}

	//! ���L�҃I�u�W�F�N�g
	void setOwner(const char *name)
	{
	  m_owner = name;
	}

	//! �I�u�W�F�N�g��͂�
	bool graspObj(std::string name);

	//! �I�u�W�F�N�g�𗣂�
	void releaseObj();

	//! ���̃I�u�W�F�N�g�ƏՓ˒����ǂ����擾����
	bool getCollisionState();
	

public:
#ifdef SIGVERSE_OGRE_CLIENT
	static CParts * decode(char *data, int aid);
#elif (defined IRWAS_OGRE_CLIENT)
	static CParts * decode(char *data, int aid);
#else
	/**
	 * @brief �o�C�i�������ꂽ���i�f�[�^�𕜌�����
	 */
	static CParts * decode(char *);
#endif	// SIGVERSE_OGRE_CLIENT
};

class BoxPartsCmpnt;

/**
 * @brief ���`��̕��i�N���X(�N���C�A���g�T�C�h�Ŏg�p)
 */
class BoxParts : public CParts
{
private:
	BoxPartsCmpnt *m_cmpnt;
public:
	/**
	 * @brief �R���X�g���N�^
	 *
	 * @param name ���i��
	 * @param pos ���i�̒��S�ʒu
	 * @param sz  ���̃T�C�Y(�c�~���~����)
	 */
	BoxParts(const char *name, const Position &pos, const Size &sz);
	//! �f�X�g���N�^
	~BoxParts();

	// added by sekikawa(2007/11/30)
	void giveSize(double &x, double &y, double &z);
	void dump();
	
private:
	BoxParts(const BoxParts &o);

	PartsCmpnt * extdata();
private:
	CParts * clone() { return new BoxParts(*this); }

#ifdef IMPLEMENT_DRAWER
	void draw(DrawContext &c);
#endif	
};

class CylinderPartsCmpnt;

/**
 * @brief �V�����_�`��̕��i�N���X(�N���C�A���g�T�C�h�Ŏg�p)
 */
class CylinderParts : public CParts
{
private:
	CylinderPartsCmpnt *m_cmpnt;
public:
	/**
	 * @brief �R���X�g���N�^
	 *
	 * @param name ���i��
	 * @param pos ���i�̒��S�ʒu
	 * @param rad  �V�����_�̉~�̔��a
	 * @param len �V�����_�̒���
	 */
	CylinderParts(const char *name,
		      const Position &pos, dReal rad, dReal len);
	//! �f�X�g���N�^
	~CylinderParts();

	// added by sekikawa(2007/11/30)
	void giveSize(double &radius, double &length);

	void dump();

private:
	CylinderParts(const CylinderParts &o);

private:
	CParts * clone() { return new CylinderParts(*this); }

	PartsCmpnt * extdata();
#ifdef IMPLEMENT_DRAWER
	void draw(DrawContext &c);
#endif

};

class SpherePartsCmpnt;

/**
 * @brief ���`��̕��i�N���X(�N���C�A���g�T�C�h�Ŏg�p)
 */
class SphereParts : public CParts
{
private:
	SpherePartsCmpnt *m_cmpnt;
public:
	/**
	 * @brief �R���X�g���N�^
	 *
	 * @param name ���i��
	 * @param pos ���i�̒��S�ʒu
	 * @param radius  ���̔��a
	 */
	SphereParts(const char *name, const Position &pos, double radius);
	
	//! �f�X�g���N�^
	~SphereParts();

private:
	SphereParts(const SphereParts &o);

//private:
public:
	// added by sekikawa(2007/11/30)
	void giveRadius(double &radius);
	void dump();

private:
	CParts * clone() { return new SphereParts(*this); }
	PartsCmpnt * extdata();
	
#ifdef IMPLEMENT_DRAWER
	void draw(DrawContext &c);
#endif
};

#endif // CParts_h
 
