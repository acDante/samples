/* $Id: Parts.h,v 1.3 2011-05-12 08:33:51 msi Exp $ */
#ifndef Parts_h
#define Parts_h

#include "systemdef.h"

#ifdef USE_ODE
#include <ode/ode.h>
#endif

#include <string>
#include <stdio.h>

#include "Position.h"
#include "Size.h"
#include "Rotation.h"

class ODEObj;
class ODEWorld;
class SimObjBase;

enum {
	PARTS_TYPE_NOT_SET = -1,
	PARTS_TYPE_BOX,
	PARTS_TYPE_CYLINDER,
	PARTS_TYPE_SPHERE,
	PARTS_TYPE_NUM,
};

typedef short PartsType;

/**
 * @brief �p�[�c���\������v�f�C���^�[�t�F�[�X
 */
class PartsCmpnt
{
public:
	//! �f�X�g���N�^
	virtual ~PartsCmpnt() {};

	//! �o�C�i��������ۂ̃T�C�Y���擾����
	virtual int datalen() = 0;

	//! �o�C�i�����f�[�^���擾����
	virtual char *dataBinary() = 0;

	//! �ێ����Ă�����i�T�C�Y�j�̕\��
	virtual void dump() = 0;	// FIX20110421(ExpSS)
};

/**
 * @brief �G���e�B�e�B���\�����镔�i�N���X
 */
class Parts
{
public:
	typedef unsigned Id;
private:
	static	Id	s_cnt;
public:
	static void	initCounter() { s_cnt = 0; }
private:
	typedef std::string S;
protected:
	PartsType	m_type;
	S		m_name;

	Position	m_pos;
	Rotation	m_rot;

	Id		m_id;
	bool		m_blind;

	char * 		m_buf;
	int		m_bufsize;
private:
	void	free_();

protected:
	Parts(PartsType t, const char *name, const Position &pos)
		: m_type(t), m_name(name), m_pos(pos),
		  m_id(0), m_blind(false), m_buf(NULL), m_bufsize(0) {}

	Parts(const Parts &o) :
		m_type(o.m_type), m_name(o.m_name),
		m_pos(o.m_pos), m_rot(o.m_rot),
		m_id(o.m_id), m_blind(o.m_blind),
		m_buf(NULL), m_bufsize(0) {}
public:
	//! �f�X�g���N�^
	virtual ~Parts() {	free_(); }
public:
	/**
	 * @brief ID�̕t�^
	 *
	 * ID�̓p�[�c�ɂ��Ă̈�ӂ̔ԍ��B
	 */
	void	setId(Id id) { m_id = id; }

	/**
	 * @brief ID�̎����t�^
	 *
	 * ID�̓p�[�c�ɂ��Ă̈�ӂ̔ԍ��B
	 */
	void	addId() {
		s_cnt++;
		m_id = s_cnt;
	}
	//! ID�̎擾
	Id	id() { return m_id; }

	//! �p�[�c�̎�ނ̎擾
	PartsType getType() { return m_type; }

	//! �p�[�c���̎擾
	const char *name(){ return m_name.c_str(); }

	//! body(�G���e�B�e�B�̒��S���\������v�f)�ł��邩�H
	bool	isBody() const {
		return strcmp(m_name.c_str(), "body") == 0? true: false;
	}

	//! �s���̃p�[�c�ł��邩�H
	bool	isBlind() const { return m_blind; }

protected:
	void	setBlind(bool b) { m_blind = b; }

public:
	//! �N�H�[�^�j�I���̐ݒ�
	virtual void	setQuaternion(dReal q0, dReal q1, dReal q2, dReal q3) = 0;

	//! �ʒu�̎擾
	virtual const dReal * getPosition() = 0;

	//! ��]�s��̎擾
	virtual const dReal * getRotation() = 0;

	//! �N�H�[�^�j�I���̎擾
	virtual const dReal * getQuaternion() = 0;

	//! �ʒu�̎擾
	void givePosition(double &x, double &y, double &z);

	//! �N�H�[�^�j�I���̎擾
	void giveQuaternion(double &qw, double &qx, double &qy, double &qz);

	/**
	 * @brief �p�[�c�I�u�W�F�N�g�̃o�C�i����
	 *
	 * @param n �o�C�i���f�[�^�T�C�Y
	 * @return	�o�C�i���f�[�^
	 */
	char *	toBinary(int &n);

public:
	virtual PartsCmpnt * extdata() = 0;

	// �_���v���\�b�h(�f�o�b�O�p)
	virtual void	dump();

#ifdef IMPLEMENT_DRAWER
	virtual void draw(DrawContext &c) = 0;
#endif

};

#endif // Parts_h

