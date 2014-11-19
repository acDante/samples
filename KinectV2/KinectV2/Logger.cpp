//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "Logger.h"

// static変数の初期化 initialize static variables
std::string Logger::fileName = "";
FILE* Logger::file = NULL;

void Logger::Initialize(const std::string& fileName)
{
	// ファイルの名前を保存 save file
	Logger::fileName = fileName;

	// ファイルを開き、ポインタを保存 open file and handle the pointer of it
	errno_t error;
	printf("Logger::fileName=%s", Logger::fileName.c_str());
	if (error = fopen_s(&(Logger::file), Logger::fileName.c_str(), "a") != 0){
		//エラー処理 error process
		printf("cannot open log file: %s\n",fileName.c_str());
	}


	// ファイルが開けなかった場合、can not open file
	if (file == NULL)
	{
		// エラーメッセージを出し、プログラムを終了します。 display error message to exit program
		MessageBox(NULL, "Failed to open log file for writing.\nApplication will be closed now.", "Error Info", MB_ICONWARNING);
		exit(0);
	}
}

void Logger::Write(const std::string& log)
{
	//tm* newTime = nullptr;
	//__time64_t longTime;

	// 現在の時間を獲得し、ローカル時間（日本の時間）に変換する 
	//_time64(&longTime);
	//_localtime64_s(newTime,&longTime);

	// 書き込むログの内容を保存する
	//fprintf(file, "[%02d %02d -%02d:%02d:%02d]%s\n",
	//	newTime->tm_mon + 1, newTime->tm_mday,
	//	newTime->tm_hour, newTime->tm_min, newTime->tm_sec,
	//	log.c_str());
	//printf("%s\n", log.c_str());
	fprintf(file, "%s\n",log.c_str());
	// ファイルにを書き込む
	fflush(file);
}