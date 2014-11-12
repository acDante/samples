//#include "stdafx.h"  
#include "MyService.h"

MyService::~MyService()
{
	// 切断します  
	this->disconnect();
}

double MyService::onAction()
{
	return 1.0;
}

/**
void MyService::onRecvMsg(sigverse::RecvMsgEvent &evt,std::string str)
{
	// メッセージ送信元とメッセージを取得します  
	std::string sender = evt.getSender();
	std::string msg = evt.getMsg();

	// メッセージが"Hello"だった場合コントローラにメッセージを返します  
	this->sendMsgToCtr(sender, str);
}
*/

void MyService::onRecvMsg(sigverse::RecvMsgEvent &evt)
{
	// メッセージ送信元とメッセージを取得します  
	std::string sender = evt.getSender();
	std::string msg = evt.getMsg();
	printf(" msg =%s @onRecvMsg()  MyService.cpp\n",msg);
	sendMsgToCtr(sender, strMsg);
}

/**
void MyService::startLoop(double ltime)
{
	m_start = true;
	static clock_t start;
	clock_t now;
	start = clock();

	m_onLoop = true;
	while (1){

		if (m_onLoop){
			if (ltime > 0){
				now = clock();
				double tmp = (double)(now - start) / CLOCKS_PER_SEC;
				if (tmp > ltime) {
					m_onLoop = false;
					break;
				}
			}
			if (!checkRecvData(100)) {
				m_onLoop = false;
				break;
			}
		}
		else break;
	}
	m_start = false;
}
*/


void MyService::setMessage(std::string str){
	this->strMsg = str;
}
void MyService::sendStrMessage(std::string strMsg){
	std::string sender = "";
	sendMsgToCtr(sender, strMsg);
}


