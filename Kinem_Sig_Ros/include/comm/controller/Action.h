/* $Id: Action.h,v 1.1.1.1 2011-03-25 02:18:50 okamoto Exp $ */ 
#ifndef CommController_Action_h
#define CommController_Action_h

#include "Controller.h"

class ActionEvent : public ControllerEvent
{
public:
	bool	set(int packetNum, int seq, char *data, int n);
};


#endif // CommController_Action_h
 

