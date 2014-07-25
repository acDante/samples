/* $Id: Position.h,v 1.2 2011-03-31 08:15:57 okamoto Exp $ */ 
#ifndef Position_h
#define Position_h

#include "systemdef.h"

// added by sekikawa (2009/3/24)
// compile error occurred (memcpy() not found) when USE_ODE not defined.
#include <string.h>

#ifdef USE_ODE
#include "ode/ode.h"
#endif

#ifdef UNIT_TEST
#include <math.h>
#include <float.h>
#endif

class Position
{
private:
	enum { DIMENSION = 3,};
private:
	dReal m_values[DIMENSION];
public:
	Position() {
		set(0.0, 0.0, 0.0);
	}
	
	Position(dReal x_, dReal y_, dReal z_) 
	{
		set(x_, y_, z_);
	}
	Position(const Position &o) 
	{
		copy(o);
	}
	void	set(dReal x_, dReal y_, dReal z_)
	{
		m_values[0] = x_;
		m_values[1] = y_;
		m_values[2] = z_;
	}
	const dReal * values() { return m_values; }

	dReal x() const { return m_values[0]; }
	dReal y() const { return m_values[1]; }
	dReal z() const { return m_values[2]; }

	void x(dReal v) { m_values[0] = v; }
	void y(dReal v) { m_values[1] = v; }
	void z(dReal v) { m_values[2] = v; }

#ifdef IRWAS_TEST_CLIENT
	dReal glx() const { return x(); }
	dReal gly() const { return y(); }
	dReal glz() const { return z(); }
#endif

	Position &operator=(const Position &o) {
		copy(o);
		return *this;
	}

private:
	void copy(const Position &o){
		memcpy(m_values, o.m_values, sizeof(m_values[0])*DIMENSION);
	}

public:
#ifdef UNIT_TEST
	bool operator==(const Position &o) {
		for (int i=0; i<DIMENSION; i++) {
			if (fabs(m_values[i] - o.m_values[i]) > DBL_EPSILON) {
				return false;
			}
		}
		return true;
	}
#endif
};


#endif // Position_h
 

