/* $Id: Rotation.h,v 1.3 2011-07-27 07:12:28 okamoto Exp $ */ 
#ifndef Rotation_h
#define Rotation_h

#include "systemdef.h"
#include <string.h>

#ifdef USE_ODE
#include <ode/ode.h>
#endif

#include "Vector3d.h"

enum {
	ROTATION_TYPE_NOT_SET = -1,
	ROTATION_TYPE_QUATERNION,
	ROTATION_TYPE_NUM,
};

typedef short RotationType;

class Rotation
{
private:
	dQuaternion	m_q;
#ifdef USE_ODE
	dMatrix3	m_m;
#endif
private:
	void	makeMatrix()
	{
#ifdef USE_ODE
		dQtoR(m_q, m_m);
#endif
	}
public:
	Rotation();
	Rotation(const Rotation &o);

	Rotation(double w, double x, double y, double z){
	  setQuaternion(w, x, y, z);
	}
	void	setQuaternion(const dReal *q)
	{
	  setQuaternion(q[0], q[1], q[2], q[3]);
	}
	void	setQuaternion(dReal w, dReal x, dReal y, dReal z)
	{
		int i=0; 
		m_q[i] = w; i++;
		m_q[i] = x; i++;
		m_q[i] = y; i++;
		m_q[i] = z; i++;

		makeMatrix();
	}

	dReal	qw(){return m_q[0];}
	dReal	qx(){return m_q[1];}
	dReal	qy(){return m_q[2];}
	dReal	qz(){return m_q[3];}


	const 	dReal * q() const { return m_q; }

#ifdef USE_ODE
	void	setAxisAndAngle(double ax, double ay, double az, double angle)
	{
		dQFromAxisAndAngle(m_q, ax, ay, az, angle);
		makeMatrix();
	}

        void    setAxisAndAngle(double ax, double ay, double az, double angle, double direct)
        {
	  if (direct == 1.0) {
	    // It rotates absolutely for the specification of the angle.
	    dQFromAxisAndAngle(m_q, ax, ay, az, angle);
	    makeMatrix();
	  } else {
	    // It rotates for the specification of the relative angle.
	    dQuaternion qt1, qt2;
	    dQFromAxisAndAngle(qt1, ax, ay, az, angle);
	    dQMultiply0(qt2, q(), qt1); 
	    setQuaternion(qt2[0], qt2[1], qt2[2], qt2[3]);
	  }
        }

	const dReal * matrix() const
	{
		return m_m;
	}

	dReal operator()(int r, int c)  const {
		return m_m[r*4 + c];
	}

	dReal	apply(int row, const Vector3d &v) const
	{
		const Rotation &r = *this;
		return r(row, 0)*v.x()  + r(row, 1)*v.y() + r(row, 2)*v.z();
	}
	
	Vector3d & apply(const Vector3d &v, Vector3d &o) const
	{
		o.x(apply(0, v));
		o.y(apply(1, v));
		o.z(apply(2, v));
		return o;
	}

	Vector3d & apply(Vector3d &v) const
	{
		Vector3d v_;
		v_.x(apply(0, v));
		v_.y(apply(1, v));
		v_.z(apply(2, v));
		v = v_;
		return v;
	}

	Rotation & operator*=(const Rotation &o);
	Rotation & operator=(const Rotation &o);

	bool operator!=(Rotation &o)
	{
	  if(this->m_q[0] != o.qw() ||
	     this->m_q[1] != o.qx() ||
	     this->m_q[2] != o.qy() || 
	     this->m_q[3] != o.qz())
	    {
	      return true;
	    }
	  else{ return false; }
	}


#endif
};


#endif // Rotation_h
 

