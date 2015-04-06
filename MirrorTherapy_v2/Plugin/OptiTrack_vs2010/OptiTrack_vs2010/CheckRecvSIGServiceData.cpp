/*
 * checking received SIGService data class
 */
#include "SIGService.h"
#include "CheckRecvSIGServiceData.h"

/*
 * checking received SIGService data at regular intervals.
 */
void CheckRecvSIGServiceData::run(sigverse::SIGService *m_srv)
{
	while(true)
	{
		if(!m_srv->checkRecvData(1))
		{
			std::cout << "SIGService::chkRecvData ERR." << std::endl;
		}

		//sleep 1 sec.
		Sleep(1000);
	}
};
