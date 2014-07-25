/* $Id: RecvMessage.h,v 1.1.1.1 2011-03-25 02:18:50 okamoto Exp $ */ 
#ifndef RecvMessage_h
#define RecvMessage_h

#include <string>
#include <vector>


class RecvMessageEvent
{
private:
	typedef std::string S;
	typedef std::vector<S> C;
private:
	S	m_from;
	C	m_strs;
public:
	bool	set(int packetNum, int seq, char *data, int n);

	const char *getSender() { return m_from.c_str(); }

	int getSize() { return m_strs.size(); }
	const char *getString(int i) { return m_strs[i].c_str(); }
};



class RecvMsgEvent
{
private:
  std::string	m_from;
  std::string	m_msg;
public:
  bool	setData(std::string data, int size);
  
  const char *getSender() { return m_from.c_str(); }
  
  const char *getMsg() { return m_msg.c_str(); }
};


#endif // RecvMessage_h
 

