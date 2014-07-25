/* $Id: RecvSound.h,v 1.2 2011-03-31 08:16:17 okamoto Exp $ */ 
#ifndef RecvSound_h
#define RecvSound_h

#include <string>
#include <string.h>

#include "Controller.h"

class RawSound;


class RecvSoundEvent : public ControllerEvent
{
private:
	typedef std::string S;
private:
	S		m_caller;
	RawSound *	m_sound;
	RawSound *	m_soundTmp;
	int		m_prevSeq;
	char *		m_curr;
private:
	void	free_();
public:
	RecvSoundEvent() :
		ControllerEvent(),
		m_sound(0), m_soundTmp(0),
		m_prevSeq(-1), m_curr(0) {;}
	~RecvSoundEvent() { free_(); }
	bool	set(int packetNum, int seq, char *data, int n);
	const char *getCaller() { return m_caller.c_str(); }
	RawSound   *getRawSound() { return m_sound; }
	RawSound   *releaseRawSound() {
		RawSound *tmp = m_sound;
		m_sound = 0;
		return tmp;
	}
};

#endif // RecvSound_h
 

