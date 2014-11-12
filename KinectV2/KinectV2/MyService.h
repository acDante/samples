#ifndef _MYSERVICE_
#define _MYSERVICE_

#include "SIGService.h"  


// SIGServiceを継承したクラスを作成します  
class MyService : public sigverse::SIGService
{
private:
	std::string strMsg = "";
public:
	MyService(std::string name) : SIGService(name){};
	~MyService();
	// メッセージ受信時に呼び出される関数  
	void onRecvMsg(sigverse::RecvMsgEvent &evt,std::string str);
	void onRecvMsg(sigverse::RecvMsgEvent &evt);
	// 定期的に呼び出される関数  
	double onAction();
	//メインループ
	//void startLoop(double ltime);
	void setMessage(std::string str);
	void sendStrMessage(std::string str);

};
#endif