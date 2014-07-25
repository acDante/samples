/* $Id: RecvText.h,v 1.1.1.1 2011-03-25 02:18:50 okamoto Exp $ */ 
#ifndef CommController_RecvText_h
#define CommController_RecvText_h

#include <string>

#include "../../Encode.h"
#include "Controller.h"


class RecvTextEvent : public ControllerEvent
{
private:
	typedef std::string S;
private:
	S	m_caller;
	S	m_text;
	Encode  m_encode;
public:
	bool	set(int packetNum, int seq, char *data, int n);
	const char *	getCaller() { return m_caller.c_str(); }
	const char *	getText() { return m_text.c_str(); }
	Encode  	getEncode() { return m_encode; }
};


#endif // CommController_RecvText_h
 

