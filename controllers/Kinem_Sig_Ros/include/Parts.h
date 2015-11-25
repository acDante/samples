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

class PartsCmpnt
{
public:
	virtual ~PartsCmpnt() {};

	virtual int datalen() = 0;

	virtual char *dataBinary() = 0;

	virtual void dump() = 0;	// FIX20110421(ExpSS)
};

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
	virtual ~Parts() {	free_(); }
public:
	void	setId(Id id) { m_id = id; }

	void	addId() {
		s_cnt++;
		m_id = s_cnt;
	}
	Id	id() { return m_id; }

	PartsType getType() { return m_type; }

	const char *name(){ return m_name.c_str(); }

	bool	isBody() const {
		return strcmp(m_name.c_str(), "body") == 0? true: false;
	}

	bool	isBlind() const { return m_blind; }

protected:
	void	setBlind(bool b) { m_blind = b; }

public:
	virtual void	setQuaternion(dReal q0, dReal q1, dReal q2, dReal q3) = 0;

	virtual const dReal * getPosition() = 0;

	virtual const dReal * getRotation() = 0;

	virtual const dReal * getQuaternion() = 0;

	void givePosition(double &x, double &y, double &z);

	void giveQuaternion(double &qw, double &qx, double &qy, double &qz);

	char *	toBinary(int &n);

public:
	virtual PartsCmpnt * extdata() = 0;

	virtual void	dump();

#ifdef IMPLEMENT_DRAWER
	virtual void draw(DrawContext &c) = 0;
#endif

};

#endif // Parts_h


