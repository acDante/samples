/* $Id: Controller.h,v 1.1.1.1 2011-03-25 02:18:50 okamoto Exp $ */ 
#ifndef CommController_Controller_h
#define CommController_Controller_h

class ControllerEvent
{
protected:
	double m_currTime; 

public:
	double	time() { return m_currTime; }

	void setTime(double time) { m_currTime = time; } 
	
};

#endif // CommController_Controller_h
 

