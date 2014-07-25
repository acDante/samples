/* $Id: Controller.h,v 1.19 2011-12-14 08:13:44 okamoto Exp $ */ 
#ifndef Controller_h
#define Controller_h

#include <string>
#include <vector>
#include <map>

#include "ControllerImpl.h"
#include "ViewImageInfo.h"
#include "SimObj.h"

//#include "DynamicsController.h"


class ViewImage;
class DepthImage;
class Text;
class RawSound;





class DynamicsController {
public:
	DynamicsController();
	void setWheelProperty(
		SimObj *my,
		char   *leftWheelName,
		double leftMotorConsumption,
		double leftWheelMaxSpeed,
		double leftWheelSpeedUnit,
		double leftSlipNoise,
		double leftEncoderResolution,
		double leftMaxForce,
		char   *rightWheelName,
		double rightMotorConsumption,
		double rightWheelMaxSpeed,
		double rightWheelSpeedUnit,
		double rightSlipNoise,
		double rightEncoderResolution,
		double rightMaxForce
	);

	void differentialWheelsSetSpeed(SimObj *my,double left,double right);

	double getAxleLength();

	double getLeftWheelRadius(SimObj *my);

	double getRightWheelRadius(SimObj *my);

	double getLeftEncoderNoise();

	double getRightEncoderNoise();

	double getLeftSlipNoise();

	double getRightSlipNoise();

	double getCurrentLeftWheelSpeed();

	double getCurrentRightWheelSpeed();

private:

	#define ACCURACY (0.00000001)

	char   *leftWheelName;
	double leftMotorConsumption;
	double leftWheelMaxSpeed;
	double leftWheelSpeedUnit;
	double leftSlipNoise;
	//double leftCumulativeNoise;
	double leftEncoderResolution;
	double leftMaxForce;
	char   *rightWheelName;
	double rightMotorConsumption;
	double rightWheelMaxSpeed;
	double rightWheelSpeedUnit;
	double rightSlipNoise;
	//double rightCumulativeNoise;
	double rightEncoderResolution;
	double rightMaxForce;


	double axleLength;
	double leftWheelRadius;
	double rightWheelRadius;
	double leftEncoderNoise;
	double rightEncoderNoise;

	double currentLeftWheelSpeed;
	double currentRightWheelSpeed;

};





class Controller : public ControllerImpl, public SimObj::RequestSender
{
private:
	typedef ControllerImpl Super;
	typedef std::string S;
	typedef std::map<S, SimObj*> M;
	
private:
	M		m_objs;
	bool		m_in;
private:
	SimObj *	find(const char *name)
	{
	  return m_objs[name];
	}
 public:
	void	updateObjs();
	void	clearObjs();
private:
	bool			send(CommDataEncoder &);
	CommDataResult *	recv(int bufsize);
	SOCKET getControllerSocket() { SOCKET sock; return sock; }

protected:
	Controller();

	double getRand();

	void setWheelProperty(
		char   *leftWheelName,
		double leftMotorConsumption,
		double leftWheelMaxSpeed,
		double leftWheelSpeedUnit,
		double leftSlipNoise,
		double leftEncoderResolution,
		double leftMaxForce,
		char   *rightWheelName,
		double rightMotorConsumption,
		double rightWheelMaxSpeed,
		double rightWheelSpeedUnit,
		double rightSlipNoise,
		double rightEncoderResolution,
		double rightMaxForce
	);

	void setWheelProperty(
		const char   *objectName,
		char   *leftWheelName,
		double leftMotorConsumption,
		double leftWheelMaxSpeed,
		double leftWheelSpeedUnit,
		double leftSlipNoise,
		double leftEncoderResolution,
		double leftMaxForce,
		char   *rightWheelName,
		double rightMotorConsumption,
		double rightWheelMaxSpeed,
		double rightWheelSpeedUnit,
		double rightSlipNoise,
		double rightEncoderResolution,
		double rightMaxForce
	);

	double getAxleLength();

	double getAxleLength(const char *simObjName);

	double getLeftWheelRadius();

	double getLeftWheelRadius(const char *simObjName);

	double getRightWheelRadius();

	double getRightWheelRadius(const char *simObjName);

	double getLeftEncoderNoise();

	double getLeftEncoderNoise(const char *simObjName);

	double getRightEncoderNoise();

	double getRightEncoderNoise(const char *simObjName);

	void differentialWheelsSetSpeed(double left,double right);

	void differentialWheelsSetSpeed(const char *simObjName,double left,double right);

	SimObj *	getObj(const char *name);

	double          getSimulationTime();

	RobotObj *	getRobotObj(const char *name);

	// old
	ViewImage * 	captureView(ColorBitType cbtype, ImageDataSize size);

	// old
	ViewImage * 	captureView(ColorBitType cbtype, ImageDataSize size, int id);

	// old
	bool	       	detectEntities(std::vector<std::string> &v);

	// old
	bool	       	detectEntities(std::vector<std::string> &v, int id);

	void            worldStep(double stepsize);

	void            worldQuickStep(double stepsize);

	Text *		getText(RawSound &sound);

	bool		getAllEntities(std::vector<std::string> &v);

	//bool		connectToService(std::string name);


	bool	isProcessing()
	{
		return m_in;
	}
	void	onPreEvent()
	{
		m_in = true;
	}
	void	onPostEvent()
	{
	  //updateObjs();
	  //clearObjs();
	  m_in = false;
	}

private:
	typedef std::map<std::string, DynamicsController*> DYNAMICS_CONTROLLER_LIST;
	DYNAMICS_CONTROLLER_LIST dynamicsDataList;


	void	add(SimObj *obj);

	enum{
		MODE_NOT_USE_WHEEL = 0,	
		MODE_USE_WHEEL		
	};
	int dynamicsMode;

	void slipWheel();

	//
	//
	//void slipLeftWheel();

	//
	//
	//void slipRightWheel();

	//
	//             T
	//             r
	//
	//
	//double getTorqueFromMotor(double motorConsumption,double radius,double wheelSpeed);

	//
	///
	//void moveMotor(char *jointName,double motorConsumption);

public:
	bool	attach(const char *server, int port, const char *myname);
	void	loopMain();

	static void init();
};


#endif // Controller_h
 

