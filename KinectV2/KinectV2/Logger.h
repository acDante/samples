#ifndef _LOGGERHEADER_
#define _LOGGERHEADER_
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <stddef.h>
#include <time.h>
#include <Windows.h>
#include <string> 

class Logger
{
public:
	static void Initialize(const std::string& fileName);
	static void Write(const std::string& log);

protected:
	static std::string fileName;
	static FILE* file;
};

#endif