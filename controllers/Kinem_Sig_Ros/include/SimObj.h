/*
 * Written by noma on  2012-03-27
 * Modified by Tetsunari Inamura on 2013-12-30
 *    Change get/setVelocity to get/setLinearVelocity
 *    change JointVelocity  to AngularVelocityToJoint
 *    change ObjectVelocity to AngularVelocityToParts
 */


#ifndef SimObj_h
#define SimObj_h

#include <map>
#include <iostream>
#include <vector>

#include "systemdef.h"
#include "SimObjBase.h"
#include "Attribute.h"
#include "CParts.h"
#include "Value.h"

class Vector3d;
#ifdef DEPRECATED
class Command;
#endif

class CommDataEncoder;
class JointForce;
class CommDataResult;

class SimObj : public SimObjBase
{
public:
	class RequestSender {
	public:
		virtual ~RequestSender() {}
		virtual bool send(CommDataEncoder &) = 0;
		virtual CommDataResult * recv(int bufsize) = 0;
	};
	typedef std::map<S, CParts*> PartsM;

	SimObj();
	virtual ~SimObj();

	// added by kawamoto@tome (2011/04/15)
#ifdef WIN32
	void setRequestSener(RequestSender *s) {
		m_sender = s;
	}
#else
	void setRequestSener(RequestSender *s);
	/* {
		m_sender = s;
	}*/
#endif

#ifdef WIN32
	// ++++++++++++++ for test only +++++++++++++++++
	// added by sekikawa(2007/11/30)
	void setId(int id);
	// +++++++ this should be deleted later +++++++++
#endif

	CParts  *getParts(const char *name);

	CParts  *getMainParts(){
	  return getParts("body");
	}

	// added by sekikaw(2007/11/30)
	std::map<std::string, CParts *>& getPartsCollection();

	void copy(const SimObj &o);

protected:
	void	free_();
	RequestSender * m_sender;

private:
	typedef SimObjBase Super;
	PartsM	m_parts;

  
	class Iterator : public PartsIterator {
	private:
		typedef std::map<std::string, CParts*> M;
	private:
		M &m_map;
		M::iterator m_i;
	public:
		Iterator(M &m) : m_map(m) {
			m_i = m_map.begin();
		}
	private:
		Parts *next() {
			if (m_i == m_map.end()) { return NULL; }
			CParts *p = m_i->second;
			m_i++;
			return p;
		}
	};

public:

	PartsIterator *getPartsIterator()
	{
		return new Iterator(m_parts);
	}

	void 	push(Attribute *attr)
	{
		Super::push(attr);
	}

	void	push(CParts *p);

	int setBinary(char *data, int n);
	
	enum{
		MODE_NOT_USE_WHEEL = 0,	
		MODE_USE_WHEEL		
	};
	int dynamicsMode; //TODO: naming is bad. It should be changed to wheelMode

	////// added by okamoto@tome(2012/08/02)
	std::string getCameraLinkName(int camID = 1);

	bool getCamPos(Vector3d &pos, int camID = 1, bool requestToServer = true);

	bool setCamPos(Vector3d pos, int camID = 1);

	bool getCamDir(Vector3d &v, int camID = 1, bool requestToServer = true);

	bool getCamQuaternion(double &qw,double &qx, double &qy, double &qz, int camID = 1);

	bool setCamDir(Vector3d v, int camID = 1);

	double getCamFOV(int camID = 1);

	bool setCamFOV(double fov, int camID = 1);

	double getCamAS(int camID = 1);

	bool setCamAS(double as, int camID = 1);

	std::string getCamLink(int camID = 1);

	//[ToDo]
	void setCamLink(std::string link, int camID = 1);

	bool getIsGrasped();

#ifdef CONTROLLER

public:

	void print();
	void setPosition(double x, double y, double z);

	void setPosition(const Vector3d &v);

	void setAxisAndAngle(double ax, double ay, double az, double angle);

	//for Okonomiyaki
        void setAxisAndAngle(double ax, double ay, double az, double angle, double direct);

	
	void setRotation(const Rotation &r);
	
	void setForce(double fx, double fy, double fz);

	void addForce(double fx, double fy, double fz);

	void addForceAtPos(double fx, double fy, double fz, double px, double py, double pz);

	void addForceAtRelPos(double fx, double fy, double fz, double px, double py, double pz);

	void addRelForce(double fx, double fy, double fz);
	
	void addRelForceAtPos(double fx, double fy, double fz, double px, double py, double pz);

	void addRelForceAtRelPos(double fx, double fy, double fz, double px, double py, double pz);

	void addForceToParts(const char* parts, double fx, double fy, double fz);

	void setGravityMode(bool gravity);

	int getGravityMode();

	void setDynamicsMode(bool dynamics);

	bool getDynamicsMode();

	void setMass(double mass);

	double getMass();

	void setAccel(double ax, double ay, double az);
	
	void setTorque(double x, double y, double z);

	// Changed from {s|g}etVelocity by inamura on 2013-12-30
	void setLinearVelocity(double vx,double vy,double vz);
	void getLinearVelocity(Vector3d &vec);

	void setAngularVelocity(double x,double y,double z);
	void getAngularVelocity(Vector3d &vec);

	void setJointAngle(const char *jointName, double angle);

	//added by okamoto@tome (2011/2/17)
	void setJointQuaternion(const char *jointName, double qw, double qx, double qy, double qz , bool offset = false);

	//added by okamoto@tome (2011/3/3)
	void addJointTorque(const char *jointName, double t);

	//added by okamoto@tome (2011/3/9)
	// modified by inamura on 2013-12-30
	void setAngularVelocityToJoint(const char *jointName, double v, double max);
	void setAngularVelocityToParts(const char *partsName, double v, double max);
	void setJointVelocity         (const char *partsName, double v, double max);

	//added by okamoto@tome (2011/3/9)
	double getJointAngle(const char *jointName);

	std::map<std::string, double> getAllJointAngles();

	bool getJointPosition(Vector3d &pos, const char *jointName);

	bool getPartsPosition(Vector3d &pos, const char *partsName);
	
	bool getJointForce(const char *jointName, JointForce &jf1, JointForce &jf2);

	void setCollisionEnable(bool flag);


	void	connectJoint(const char *jointName, const char *targetName)
	{
		connectJoint(jointName, NULL, targetName, NULL);
	}

	void	connectJoint(const char *jointName, const char *myParts,
			     const char *targetName, const char *targetParts);
	void	releaseJoint(const char *jointName);

	Vector3d & getPosition(Vector3d &v);

	Rotation & getRotation(Rotation &r);

	// fx, fy, fz
	Vector3d & getForce(Vector3d &v);

	Vector3d & getAccel(Vector3d &v);

	// tqx, tqy, tqz
	Vector3d & getTorque(Vector3d &v);

	// lepx, lepy, lepz
	//Vector3d & getLeftViewPoint(Vector3d &v);
	// repx, repy, repz
	//Vector3d & getRightViewPoint(Vector3d &v);


	////////
	////// added by okamoto@tome(2012/01/05)
	void setCameraViewPoint(Vector3d v, int camID = 1);

	Vector3d & getCameraViewPoint(Vector3d &v, int camID = 1);

	void setCameraViewVector(Vector3d v, int camID = 1);

	Vector3d & getCameraViewVector(Vector3d &v, int camID = 1);

	// epx1, epy1, epz1
	Vector3d & getCamera1ViewPoint(Vector3d &v);
	// epx2, epy2, epz2
	Vector3d & getCamera2ViewPoint(Vector3d &v);
	// epx3, epy3, epz3
	Vector3d & getCamera3ViewPoint(Vector3d &v);
	// epx4, epy4, epz4
	Vector3d & getCamera4ViewPoint(Vector3d &v);
	// epx5, epy5, epz5
	Vector3d & getCamera5ViewPoint(Vector3d &v);
	// epx6, epy6, epz6
	Vector3d & getCamera6ViewPoint(Vector3d &v);
	// epx7, epy7, epz7
	Vector3d & getCamera7ViewPoint(Vector3d &v);
	// epx8, epy8, epz8
	Vector3d & getCamera8ViewPoint(Vector3d &v);
	// epx9, epy9, epz9
	Vector3d & getCamera9ViewPoint(Vector3d &v);


	// levx, levy, levz
	//Vector3d & getLeftViewVector(Vector3d &v);
	// revx, revy, revz
	//Vector3d & getRightViewVector(Vector3d &v);
	// evx1, evy1, evz1
	Vector3d & getCamera1ViewVector(Vector3d &v);
	// evx2, evy2, evz2
	Vector3d & getCamera2ViewVector(Vector3d &v);
	// evx3, evy3, evz3
	Vector3d & getCamera3ViewVector(Vector3d &v);
	// evx4, evy4, evz4
	Vector3d & getCamera4ViewVector(Vector3d &v);
	// evx5, evy5, evz5
	Vector3d & getCamera5ViewVector(Vector3d &v);
	// evx6, evy6, evz6
	Vector3d & getCamera6ViewVector(Vector3d &v);
	// evx7, evy7, evz7
	Vector3d & getCamera7ViewVector(Vector3d &v);
	// evx8, evy8, evz8
	Vector3d & getCamera8ViewVector(Vector3d &v);
	// evx9, evy9, evz9
	Vector3d & getCamera9ViewVector(Vector3d &v);

	// levx, levy, levz
	//void setLeftViewVector(const Vector3d &v);
	// revx, revy, revz
	//void setRightViewVector(const Vector3d &v);
	// evx1, evy1, evz1



	void setCamera1ViewVector(const Vector3d &v);
	// evx2, evy2, evz2
	void setCamera2ViewVector(const Vector3d &v);
	// evx3, evy3, evz3
	void setCamera3ViewVector(const Vector3d &v);
	// evx4, evy4, evz4
	void setCamera4ViewVector(const Vector3d &v);
	// evx5, evy5, evz5
	void setCamera5ViewVector(const Vector3d &v);
	// evx6, evy6, evz6
	void setCamera6ViewVector(const Vector3d &v);
	// evx7, evy7, evz7
	void setCamera7ViewVector(const Vector3d &v);
	// evx8, evy8, evz8
	void setCamera8ViewVector(const Vector3d &v);
	// evx9, evy9, evz9
	void setCamera9ViewVector(const Vector3d &v);
	

	////// added by noma@tome(2012/02/20)
	std::vector<const char*> getPointedObject(const char* speakerName, int lrFlag, int lineID, int typicalType=1);

	////// added by okamoto@tome(2012/11/14)
	bool getPointingVector(Vector3d &vec, const char *joint1, const char *joint2);

	////// added by okamoto@tome(2012/11/14)
	bool getPointingVector(Vector3d &vec, int lrFlag = 0);

 private:
	bool sendEntityQuaternion(const dReal *rot, bool abs);

	bool sendRequest(std::string name, int requestNum);

 public:

	
#ifdef DEPRECATED
	Command * createJointControlCommand();
#endif // DEPRECATED

#endif // CONTROLLER


#ifdef DEPRECATED
private:
	typedef std::map<S, double> JointValueM;
	JointValueM	m_jointValues;
#endif // DEPRECATED


};



class RobotObj : public SimObj
{
 public:

 RobotObj() : 
  SimObj(), 
    m_wheelRadius(0.0),
    m_wheelDistance(0.0)
      {}


  bool setWheel(double wheelRadius, double wheelDistance);

  bool setWheelVelocity(double leftWheel, double rightWheel);
  
  

 private:

  double m_wheelRadius;
  double m_wheelDistance;
  
};
#endif // SimObj_h
 

