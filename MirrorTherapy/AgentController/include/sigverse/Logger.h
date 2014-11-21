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

/**
 * @brief ���O�N���X
 *
 * �o�̓��x���A�o�͐�̐؂�ւ��A�����̏o�͐�̎w��\�ȃ��O�̎����B
 * �V�~�����[�V�����T�[�o�ւ̃��O�̑��M���\�B
 * ���̃N���X�𒼐ڎg�p�����A�}�N������Ďg�p���邱�ƁB
 */
class Logger
{
public:
	//! �o�͐撊�ۃN���X
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

	//! ���O���X�i�[�N���X
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
	//! �f�X�g���N�^
	~Logger() { free_(); }
public:
	//! �o�͐�ɕW���o�͂�ǉ�
	void 	pushSTDOUT();
	//! �o�͐�Ƀt�@�C����ǉ�
	void	pushFile(const char *fname);
	//! �o�͐�ɓ]����\�P�b�g��ǉ�
	void	pushSocket(SOCKET);
	/**
	 * @brief �o�͐�Ƀ��X�i�[��ǉ��B
	 *  ���X�i�[�̎������s�Ȃ����ƂŁA�C�ӂ̏o�͐�֏o�͂��\�ɂȂ�B
	 */
	void    pushListener(Listener *l);
	//! �o�̓��x���̎w��
	void	setOutputLevel(int l) { m_outlevel = l; }
	//! ���O���x���̎w��
	void	setLevel(int l) { m_level = l; }
	//! �t�H�[�}�b�g���g�p�����o�̓��\�b�h
	void	print(const char *fmt, ...);
public:
	//! Logger�N���X�C���X�^���X�̎擾(�V���O���g���p�^�[��)
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
 
