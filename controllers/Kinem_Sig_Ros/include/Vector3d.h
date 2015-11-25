/* $Id: Vector3d.h,v 1.2 2011-03-31 08:15:58 okamoto Exp $ */ 
#ifndef Vector3d_h
#define Vector3d_h

#include <math.h>
#include "Logger.h"
#include "Position.h"
//#include "Rotation.h"

class Rotation;

class Vector3d
{
private:
	double	m_x, m_y, m_z;
public:
	Vector3d() : m_x(0.0), m_y(0.0), m_z(0.0) {;}
	Vector3d(double x_, double y_, double z_) : m_x(x_), m_y(y_), m_z(z_) {;}
	Vector3d(const Vector3d &o) : m_x(o.m_x), m_y(o.m_y), m_z(o.m_z) {;}

	void	set(double x_, double y_, double z_)
	{
		m_x = x_; m_y = y_; m_z = z_;
	}

	void	shift(const Vector3d &v)
	{
		m_x += v.x(); m_y += v.y(); m_z += v.z();
	}

	void	shift(double x_, double y_, double z_)
	{
		m_x += x_; m_y += y_; m_z += z_;
	}

	
	void	x(double v) { m_x = v; } 
	void	y(double v) { m_y = v; } 
	void	z(double v) { m_z = v; } 

	
	double x() const { return m_x; } 
	double y() const { return m_y; } 
	double z() const { return m_z; } 

	Vector3d & operator+=(const Vector3d &o)
	{
		this->m_x += o.x();
		this->m_y += o.y();
		this->m_z += o.z();
		return *this;
	}

	Vector3d & operator-=(const Vector3d &o)
	{
		this->m_x -= o.x();
		this->m_y -= o.y();
		this->m_z -= o.z();
		return *this;
	}
	Vector3d & operator+=(const Position &o)
	{
		this->m_x += o.x();
		this->m_y += o.y();
		this->m_z += o.z();
		return *this;
	}

	Vector3d & operator-=(const Position &o)
	{
		this->m_x -= o.x();
		this->m_y -= o.y();
		this->m_z -= o.z();
		return *this;
	}

	Vector3d & operator*=(double v)
	{
		this->m_x *= v;
		this->m_y *= v;
		this->m_z *= v;
		return *this;
	}

	Vector3d & operator/=(double v)
	{
		this->m_x /= v;
		this->m_y /= v;
		this->m_z /= v;
		return *this;
	}

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

	double length() const { return sqrt(m_x*m_x + m_y*m_y + m_z*m_z); }

	double angle(const Vector3d &axis)
	{
		double prod = m_x*axis.m_x + m_y*axis.m_y + m_z*axis.m_z;
		double v = length() * axis.length();
		return prod/v;

	}

	void	normalize()
	{
		double l = length();
		m_x /=l; m_y/=l; m_z/=l;
	}

	Vector3d & rotate(const Rotation &r);

	/*
	void rotateByQuaternion(double qw, double qx, double qy, double qz);
	*/
};


#endif // Vector3d_h
 

