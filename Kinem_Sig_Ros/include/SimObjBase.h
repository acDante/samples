/* 
 * Written by okamoto on 2011-12-21
 * Modified by Tetsunari Inamura on 2013-12-29
 *   Add English comments (Translation from v2.2.0 is finished)
 *   Reduce warning in compilation
 * Modified by Tetsunari Inamura on 2013-12-30
 *   Change setVelocity to setLinearVelocity
 */

#ifndef SimObjBase_h
#define SimObjBase_h

#include <string>
#include <map>
#include <vector>

#include "systemdef.h"
#include "Attribute.h"
#include "Vector3d.h"
#include "Logger.h"


class Parts;
class Rotation;

/**
 * @brief Class for basic eneity in simulator
 */
class SimObjBase
{
public:
	//! Basic exeption class
	class Exception
	{
	protected:
		std::string m_msg;
	public:
		Exception() {}
		Exception(const char *msg_) : m_msg(msg_) {;}
		const char *msg() { return m_msg.c_str(); }
	};


	//! Exception when the required attribution is not set
	class NoAttributeException : public Exception
	{
	public:
		NoAttributeException(const char *attr) : Exception()
		{
			if (attr) {
				m_msg = "No attribute : \"";
				m_msg += attr;
				m_msg += "\"";
			}
		}
	};

	//! Exception when unknown attribution is set
	class AttributeReadOnlyException : public Exception
	{
	public:
		AttributeReadOnlyException(const char *attr) : Exception() {
			if (attr) {
				m_msg = "Read only attribute : \"";
				m_msg += attr;
				m_msg += "\"";
			}
		}
	};

	/**
	 * @brief Iterator of parts object
	 *
	 * Used in inherited class
	 */
	class PartsIterator {
	public:
		virtual ~PartsIterator() {}
		virtual Parts *next() = 0;
	};
#ifndef UNIT_TEST
protected:
#endif
	enum {
		OP_NOT_SET              = 0x0,  //!< No operation
		OP_SET_POSITION         = 0x1,  //!< Operation of position change
		OP_SET_ROTATION         = 0x2,  //!< Operation of orientation change (rotation)
		OP_SET_FORCE            = 0x4,  //!< Operation of force set
		OP_SET_ACCEL            = 0x8,  //!< Operation of acceleration set
		OP_SET_TORQUE           = 0x10, //!< Operation of torque set
		OP_SET_LINEAR_VELOCITY  = 0x20,	//!< Operation of linear velocity,  changed from SET_VELOCITY by inamura on 2013-12-30
		OP_SET_ANGULAR_VELOCITY = 0x40, //!< Operation of angular velocity
	};
	//! operation type for entity
	typedef unsigned Operation;
public:
	//! ID type for entity
	typedef unsigned Id;
protected:
	//! string type
	typedef std::string S;
	//! attribution map type
	typedef std::map<S, Attribute*> AttrM;
protected:
	bool                     m_attached; //!< flag whether controller is attached
	int                      m_id;       //!< unique entity ID
	AttrM                    m_attrs;    //!< attribution map
	Operation                m_ops;      //!< called operation
	std::vector<std::string> m_files;    //!< file name of shape
	std::vector<int>         m_ids;      //!< camera ID which this entity has

protected:
	//! Constructor
	SimObjBase();
public:
	//! Destructor
	virtual ~SimObjBase();

	//! Copy constructor
	void copy(const SimObjBase &o);

	//! Refer attribution map
	const AttrM &attrs() const { return m_attrs; }

	//! add file name of shape of entity
	void addFile(const char* name){ m_files.push_back(name);}

	//! refer the # of shape files
	int getFileNum(){ return m_files.size();}

	//! Refer camera ID which this entity has
	int getCameraNum(){return m_ids.size();}

	//! Add camera ID for the entity
	void addCameraID(int id)
	{
	  int size = m_ids.size();
	  for(int i = 0; i < size; i++){
	    if(id == m_ids[i]) {
	      LOG_ERR(("Camera ID %d is already exist.",id));
	      return;	      
	    }
	  }
	  m_ids.push_back(id);
	}
	//! Refer camera ID (vector) of the entity
	std::vector<int> getCameraIDs(){return m_ids;}

	//! Refer shape file
	std::string getFile(int num)
	{
	  if(num > m_files.size()){
	    return NULL;
	  }
	  else{
	    return m_files[num];
	  }
	}
	
	//! add attribution
	void	push(Attribute *attr)
	{
		m_attrs[attr->name()] = attr;
	}

	/**
	 * @brief Set attribution by string
	 * value shoul be given by (char *); it will be parsed according to type of attribution
	 *
	 * @param name name of attribution
	 * @param v    value of attribution by string
	 */	
	void	setAttrValue(const char *name, const char *v)
	{
		getAttr(name).value().setString(v);
	}

	/**
	 * @brief Set attribution by value
	 *
	 * @param name name of attribution
	 * @param v    value of attribution
	 */
	void	setAttrValue(const char *name, const Value &v)
	{
		getAttr(name).value().copy(v);
	}

	/**
	 * @brief Check whether the required attribution is existed in order to execute simulation
	 *
	 * @retval true  attribution is existed
	 * @retval false attribution is not existed
	 */
	bool	checkAttrs();

private:
	Attribute* hasAttr(const char *name) const;
public:
	//! Refer the target attribution
	Attribute & getAttr(const char *name) const;

	//! Whether the target attribution is existed
	bool isAttr(const char *name);

	//! Refer the entity ID
	Id    	id() const { return m_id; }

	//! Whether controller is attached?
	bool	isAttached() const { return m_attached; }

	//! Refer entity name
	const char *name() const
	{
		return getAttr("name").value().getString();
	}
	
	//! Refer class name
	const char *classname()
	{
		return getAttr("class").value().getString();
	}

	// added by yahara@tome (2011/02/23)
#define DEFINE_ATTR_STRING(NAME, TOKEN)					\
		const char *NAME() const { return getAttr(TOKEN).value().getString(); } \
	void  NAME(const char *v) {						\
	  getAttr(TOKEN).value().setString(v);				\
	}
	//okamoto (2010/12/7)
#define DEFINE_ATTR_DOUBLE(NAME, TOKEN)					\
        double NAME() const { return getAttr(TOKEN).value().getDouble(); } \
	void  NAME(double v) {						\
	  getAttr(TOKEN).value().setDouble(v);				\
	}
	
#define DEFINE_ATTR_BOOL(NAME, TOKEN)				       \
        bool NAME() const { return getAttr(TOKEN).value().getBool(); } \
        void NAME(bool b) { getAttr(TOKEN).value().setBool(b); }

#include "SimObjBaseAttrs.h"

#undef DEFINE_ATTR_DOUBLE
#undef DEFINE_ATTR_BOOL
	// added by yahara@tome (2011/03/03)
#undef DEFINE_ATTR_STRING

	// Operation methods must be declared as virtual.
	/**
	 * @brief Set position of the entity
	 * @param v 3D position
	 */
	virtual void setPosition(const Vector3d &v);

	/**
	 * @brief Set position of the entity
	 * @param x_ X element
	 * @param y_ Y element
	 * @param z_ Z element
	 */
	virtual void setPosition(double x_, double y_, double z_);

	/**
	 * @brief Set force added to the entity
	 * @param x_ X element
	 * @param y_ Y element
	 * @param z_ Z element
	 */
	virtual void setForce(double fx, double fy, double fz);

	/**
	 * @brief Set torque added to the entity
	 * @param x_ X element
	 * @param y_ Y element
	 * @param z_ Z element
	 */
	virtual void setTorque(double x, double y, double z);

	/**
	 * @brief Set linear velocity of the entity
	 * @param x_ X element
	 * @param y_ Y element
	 * @param z_ Z element
	 */// changed from setVelocity: by inamura on 2013-12-30
	virtual void setLinearVelocity(double vx_,double vy_,double vz_);

	/**
	 * @brief Set angular velocity of the entity
	 * @param x_ X element
	 * @param y_ Y element
	 * @param z_ Z element
	 */
	virtual void setAngularVelocity(double x_,double y_,double z_);

	/**
	 * @brief Refer position of the entity
	 */
	Vector3d &  getPosition(Vector3d &v)
	{
		v.set(x(), y(), z());
		return v;
	}

	/**
	 * @brief Set orientation of the entity
	 *
	 * @param ax x element of the axis of rotation
	 * @param ay y element of the axis of rotation
	 * @param az z element of the axis of rotation
	 * @param angle angle value of rotation
	 */
	virtual void setAxisAndAngle(double ax, double ay, double az, double angle);

	/**
	 * @brief It rotates for the specification of the relative angle.
	 * @param ax x-axis rotation weather(i of quaternion complex part)
	 * @param ay y-axis rotation weather(j of quaternion complex part)
	 * @param az z-axis rotation weather(k of quaternion complex part)
	 * @param direct flag for ansolute / relational (1.0=absolute, else=relational)
	 */
	virtual void setAxisAndAngle(double ax, double ay, double az, double angle, double direct);

	/**
	 * @brief Set orientation of entity
	 *
	 * @param r rotating matrix
	 */
	virtual void setRotation(const Rotation &r);

private:
	void 	setQ(const dReal *q);
public:

	/**
	 * @brief Binalize of the entity
	 * @param  Size of binalized data
	 * @return Binalized data
	 */
	char *toBinary(int &n);

protected:
	//! Refer parts iterator
	virtual PartsIterator * getPartsIterator() = 0;
private:
	void	free_();

public:
	void dump();
#ifdef UNIT_TEST
	Operation ops() { return m_ops; }
#endif	
#ifdef IMPLEMENT_DRAWER
	void draw(DrawContext &c);
#endif
};

#endif // SimObjBase_h
 

