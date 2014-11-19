//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "Logger.h"

//initialize static variables
std::string Logger::fileName = "";
FILE* Logger::file = NULL;

void Logger::Initialize(const std::string& fileName)
{
	//save file name
	Logger::fileName = fileName;

	//open file and handle the pointer of it
	errno_t error;
	printf("Logger::fileName=%s", Logger::fileName.c_str());
	if (error = fopen_s(&(Logger::file), Logger::fileName.c_str(), "a") != 0){
		//error handler
		printf("cannot open log file: %s\n",fileName.c_str());
	}
	//can not open file
	if (file == NULL)
	{
		//display error message to exit program
		MessageBox(NULL, "Failed to open log file for writing.\nApplication will be closed now.", "Error Info", MB_ICONWARNING);
		exit(0);
	}
}

void Logger::Write(const std::string& log)
{
	//tm* newTime = nullptr;
	//__time64_t longTime;
	//_time64(&longTime);
	//_localtime64_s(newTime,&longTime);
	//fprintf(file, "[%02d %02d -%02d:%02d:%02d]%s\n",
	//newTime->tm_mon + 1, newTime->tm_mday,
	//newTime->tm_hour, newTime->tm_min, newTime->tm_sec,
	//log.c_str());
	//printf("%s\n", log.c_str());
	fprintf(file, "%s\n",log.c_str());
	//write to file
	fflush(file);
}