/* $Id: JointForce.h,v 1.2 2011-03-31 08:15:56 okamoto Exp $ */ 
#ifndef JointForce_h
#define JointForce_h

#include <assert.h>
#include <string>

#include "Vector3d.h"

/**
 * @brief �p�[�c�ɂ�����͂�\���N���X
 */
class JointForce
{
private:
	std::string m_partsName;
	Vector3d    m_f;
	Vector3d    m_tq;
public:
	//! �p�[�c�����擾����
	const char *getPartsName() { return m_partsName.c_str(); }
	
	//! �p�[�c�ɂ�����͂��擾����
	const Vector3d &getForce() { return m_f; }
	
	//! �p�[�c�ɂ�����g���N���擾����
	const Vector3d &getTorque() { return m_tq; }
	
	//! �e�l��ݒ肷��	
	void	set(const char *partsName,
		    const Vector3d &f,
		    const Vector3d &tq)
	{
		assert(partsName != NULL);
		m_partsName = partsName;
		m_f = f;
		m_tq = tq;
	}
	
	//! �R�s�[���Z�q
	JointForce & operator=(const JointForce &o)
	{
		m_partsName = o.m_partsName;
		m_f = o.m_f;
		m_tq = o.m_tq;
		return *this;
	}
};



#endif // JointForce_h
 
