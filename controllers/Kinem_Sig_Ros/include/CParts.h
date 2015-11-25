/* Written by msi on 2011-05-12
 * Modified by Tetsunari Inamura on 2013-12-29
 *    Add English comments (Translation from v2.2.0 is finished)
 */

#ifndef CParts_h
#define CParts_h

#include "systemdef.h"
#include "Parts.h"
//#include "SimObj.h"

/**
 * @brief Class for entity in Client side
 *
 * @see SParts
 */
// 'C' of the CParts means Client
class CParts : public Parts
{
protected:
	/**
	 * @brief Constructor
	 *
	 * @param t type of parts
	 * @param name parts name
	 * @param pos central position of parts
	 */
	CParts(PartsType t, const char *name, const Position &pos)
		: Parts(t, name, pos) {}

	//! copy constructor
	CParts(const CParts &o)
		: Parts(o) {}

	SOCKET m_sock;

	// Object name which has the parts
	std::string m_owner;
private:
	void 	setQuaternion(dReal q0, dReal q1, dReal q2, dReal q3)
	{
		m_rot.setQuaternion(q0, q1, q2, q3);
	}
public:
	//! Destructor
	virtual ~CParts() {}
#ifdef CONTROLLER
	bool getPosition(Vector3d &v);
	Rotation & getRotation(Rotation &r);
#endif
	//! Cloning the same parts
	virtual CParts * clone() = 0;

	const dReal * getPosition();
	const dReal * getRotation();
	const dReal * getQuaternion();

	//! socket to server
	void setSocket(SOCKET sock)
	{
		m_sock = sock;
	}

	//! Set name of owner object
	void setOwner(const char *name)
	{
	  m_owner = name;
	}
	//! Grasp an object
	bool graspObj(std::string name);

	//! Release an object
	void releaseObj();

	//! Get a flag whether collision is happened to other objects
	bool getCollisionState();
	

public:
#ifdef SIGVERSE_OGRE_CLIENT
	static CParts * decode(char *data, int aid);
#elif (defined IRWAS_OGRE_CLIENT)
	static CParts * decode(char *data, int aid);
#else
	/**
	 * @brief Decode of binalized parts data
	 */
	static CParts * decode(char *);
#endif	// SIGVERSE_OGRE_CLIENT
};

class BoxPartsCmpnt;

/**
 * @brief Class of box parts for Client side
 */
class BoxParts : public CParts
{
private:
	BoxPartsCmpnt *m_cmpnt;
public:
	/**
	 * @brief Constructor
	 *
	 * @param name parts name
	 * @param pos  center position of parts
	 * @param sz   size of box (depth x width x height)
	 */
	BoxParts(const char *name, const Position &pos, const Size &sz);

	//! Destructor
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
 * @brief Class for cylinder parts in client side
 */
class CylinderParts : public CParts
{
private:
	CylinderPartsCmpnt *m_cmpnt;
public:
	/**
	 * @brief Constructor
	 *
	 * @param name parts name
	 * @param pos  center position of parts
	 * @param rad  radius of the cylinder
	 * @param len  length of the cylinder
	 */
	CylinderParts(const char *name, const Position &pos, dReal rad, dReal len);
	//! Destructor
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
 * @brief Class for spherical parts in client side
 */
class SphereParts : public CParts
{
private:
	SpherePartsCmpnt *m_cmpnt;
public:
	/**
	 * @brief Constructor
	 *
	 * @param name parts name
	 * @param pos  center position of the parts
	 * @param radius  radius of the sphere
	 */
	SphereParts(const char *name, const Position &pos, double radius);
	
	//! Destructor
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
 

