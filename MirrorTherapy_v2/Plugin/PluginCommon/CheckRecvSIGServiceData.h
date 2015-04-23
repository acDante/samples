#ifndef __CHECK_RECV_SIGSERVICE_DATA_H__
#define __CHECK_RECV_SIGSERVICE_DATA_H__

#include "SIGService.h"

/**
 * @brief checking received SIGService data class
 */
class CheckRecvSIGServiceData
{
private:

public:
	void run(sigverse::SIGService *m_srv);
};

#endif //__CHECK_RECV_SIGSERVICE_DATA_H__
