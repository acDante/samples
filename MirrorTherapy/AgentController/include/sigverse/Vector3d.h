/* $Id: Vector3d.h,v 1.2 2011-03-31 08:15:58 okamoto Exp $ */ 
#ifndef Vector3d_h
#define Vector3d_h

#include <math.h>
#include "Logger.h"
#include "Position.h"
//#include "Rotation.h"

class Rotation;

/**
 * @brief 3�����x�N�g���N���X
 */
class Vector3d
{
private:
	double	m_x, m_y, m_z;
public:
	//! �R���X�g���N�^
	Vector3d() : m_x(0.0), m_y(0.0), m_z(0.0) {;}
	/**
	 * @brief �R�s�[�R���X�g���N�^
	 * @param x_ X���v�f
	 * @param y_ Y���v�f
	 * @param z_ Z���v�f
	 */
	Vector3d(double x_, double y_, double z_) : m_x(x_), m_y(y_), m_z(z_) {;}
	//! �R�s�[�R���X�g���N�^
	Vector3d(const Vector3d &o) : m_x(o.m_x), m_y(o.m_y), m_z(o.m_z) {;}

	//! �e�v�f��ݒ肷��
	void	set(double x_, double y_, double z_)
	{
		m_x = x_; m_y = y_; m_z = z_;
	}

	/**
	 * @brief �x�N�g���𕽍s�ړ�������
	 * @param v ���i�x�N�g��
	 */
	void	shift(const Vector3d &v)
	{
		m_x += v.x(); m_y += v.y(); m_z += v.z();
	}

	/**
	 * @brief �x�N�g���𕽍s�ړ�������
	 * @param x_ ���i�x�N�g��X���v�f
	 * @param y_ ���i�x�N�g��Y���v�f
	 * @param z_ ���i�x�N�g��Z���v�f
	 */
	void	shift(double x_, double y_, double z_)
	{
		m_x += x_; m_y += y_; m_z += z_;
	}

	
	void	x(double v) { m_x = v; } //!< X���v�f��ݒ肷��
	void	y(double v) { m_y = v; } //!< Y���v�f��ݒ肷��
	void	z(double v) { m_z = v; } //!< Z���v�f��ݒ肷��

	
	double x() const { return m_x; } //!< X���v�f���擾����
	double y() const { return m_y; } //!< Y���v�f���擾����
	double z() const { return m_z; } //!< Z���v�f���擾����

	//! +=���Z�q(shift�Ɠ���)
	Vector3d & operator+=(const Vector3d &o)
	{
		this->m_x += o.x();
		this->m_y += o.y();
		this->m_z += o.z();
		return *this;
	}

	//! -=���Z�q(shift�Ɠ���)
	Vector3d & operator-=(const Vector3d &o)
	{
		this->m_x -= o.x();
		this->m_y -= o.y();
		this->m_z -= o.z();
		return *this;
	}
	//! +=���Z�q(shift�Ɠ���)
	Vector3d & operator+=(const Position &o)
	{
		this->m_x += o.x();
		this->m_y += o.y();
		this->m_z += o.z();
		return *this;
	}

	//! -=���Z�q(shift�Ɠ���)
	Vector3d & operator-=(const Position &o)
	{
		this->m_x -= o.x();
		this->m_y -= o.y();
		this->m_z -= o.z();
		return *this;
	}

	//! *=���Z�q(�e�v�f�Ɏw��̒l���|����)
	Vector3d & operator*=(double v)
	{
		this->m_x *= v;
		this->m_y *= v;
		this->m_z *= v;
		return *this;
	}

	//! /=���Z�q(�e�v�f���w��̒l�Ŋ���)
	Vector3d & operator/=(double v)
	{
		this->m_x /= v;
		this->m_y /= v;
		this->m_z /= v;
		return *this;
	}

	//! /=�����ł͂Ȃ�
	bool operator!=(const Vector3d &o)
	{
	  if(this->m_x != o.x() ||
	     this->m_y != o.y() ||
	     this->m_z != o.z())
	    {
	      return true;
	    }
	  else{ return false; }
	}

	//! �x�N�g���̒����擾����
	double length() const { return sqrt(m_x*m_x + m_y*m_y + m_z*m_z); }

	/**
	 * �^����ꂽ�x�N�g���Ɛ����p�𓾂�
	 * @return cos��(-1�`1�͈̔�)
	 */
	double angle(const Vector3d &axis)
	{
		double prod = m_x*axis.m_x + m_y*axis.m_y + m_z*axis.m_z;
		double v = length() * axis.length();
		return prod/v;

	}

	//! �x�N�g�����K�i������(������1�ɂ���)
	void	normalize()
	{
		double l = length();
		m_x /=l; m_y/=l; m_z/=l;
	}

	/*
	  070809 yoshi
	  shared library �Ŏg�p����ƃ��[�h�Ɏ��s���� 
	*/
	/**
	 * @brief �x�N�g������]����
	 * @param r ��]�s��
	 * @return ��]��̃x�N�g��
	 */
	Vector3d & rotate(const Rotation &r);

	// �Ȃ����R���g���[���Ŏg���Ȃ�
	/*
	void rotateByQuaternion(double qw, double qx, double qy, double qz);
	*/
};


#endif // Vector3d_h
 
