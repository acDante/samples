/* $Id: Logger.h,v 1.3 2011-08-10 02:52:03 okamoto Exp $ */ 
#ifndef Logger_h
#define Logger_h

#include "systemdef.h"

#include <vector>

enum {
	LOG_ERRMSG = 0,
	LOG_SYSTEM,
	LOG_MSG,
	LOG_DEBUG1,
	LOG_DEBUG2,
	LOG_DEBUG3,
	LOG_DISPLAY,   //add by okamoto@tome (2011/8/3)
	LOG_ALL,
};

class Logger
{
public:
	class OutStream
	{
	private:
		bool	m_release;
	protected:
		OutStream() : m_release(true) {;}
		OutStream(bool release) : m_release(release) {;}
	public:
		virtual ~OutStream() {;}

		virtual bool noHeader() = 0;
		virtual void print(int level, const char *msg) = 0;

		bool release() { return m_release; }
	};

	class Listener : public OutStream
	{
	protected:
		Listener() : OutStream(false) {;}
	private:
		bool noHeader() { return true; }
	};
	
private:
	typedef std::vector<OutStream*> C;
private:
	C	m_streams;
	int	m_level;
	int	m_outlevel;
private:
	void	push(OutStream *o) { m_streams.push_back(o); }
	void	free_();
private:
	Logger() : m_level(LOG_MSG), m_outlevel(LOG_MSG) {;}
public:
	~Logger() { free_(); }
public:
	void 	pushSTDOUT();
	void	pushFile(const char *fname);
	void	pushSocket(SOCKET);
	void    pushListener(Listener *l);
	void	setOutputLevel(int l) { m_outlevel = l; }
	void	setLevel(int l) { m_level = l; }
	void	print(const char *fmt, ...);
public:
	static Logger & get();
};

   
#define LOG_STDOUT() Logger::get().pushSTDOUT()
#define LOG_FILE(FNAME) Logger::get().pushFile(FNAME)
#define LOG_SOCKET(S) Logger::get().pushSocket(S)
#define LOG_LISTENER(S) Logger::get().pushListener(S)

#define LOG_OUTPUT_LEVEL(L) Logger::get().setOutputLevel(L)
#define LOG_LEVEL_MIN(A, B) ( (A) < (B)? (A): (B) )
#define LOG_LEVEL_INCR(L, INCR) L = LOG_LEVEL_MIN(L + INCR, LOG_ALL)


#define LOG_PRINT(L, MSG) { Logger &l_ = Logger::get(); l_.setLevel(L); l_.print MSG; }

#define LOG_ERR(MSG) 	LOG_PRINT(LOG_ERRMSG, MSG)
#define LOG_ERROR 	LOG_ERR

#define LOG_MSG(MSG) 	LOG_PRINT(LOG_MSG, MSG)
#define LOG_SYSTEM(MSG) LOG_PRINT(LOG_SYSTEM, MSG)
#define LOG_SYS 	LOG_SYSTEM

#define LOG_DEBUG1(MSG) LOG_PRINT(LOG_DEBUG1, MSG)
#define LOG_DEBUG2(MSG) LOG_PRINT(LOG_DEBUG2, MSG)
#define LOG_DEBUG3(MSG) LOG_PRINT(LOG_DEBUG3, MSG)

#define LOG_DISPLAY(MSG) LOG_PRINT(LOG_DISPLAY, MSG)

#endif // Logger_h
 

