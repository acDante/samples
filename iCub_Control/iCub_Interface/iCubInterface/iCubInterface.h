#pragma once
#include "stdafx.h"
#include <iostream>
#include <conio.h>
#include <winsock2.h>
#include<stdio.h>
#include<stdlib.h>
#include <windows.h>


struct IPv4{

  int b1,b2,b3,b4;
 };

public ref class iCub_Interface : public sigverse::SIGService  
{  
	
public:
	iCub_Interface(System::String^ name) : SIGService(name)
	{
		tmp_score = gcnew System::Collections::Generic::List<int>();
		//tmp_msg  = gcnew System::Collections::Generic::List<System::String^>();
		m_total = 0;
        msg1 = "";
	};
	~iCub_Interface();
	int getScore();
	int getScoreSize();
	int getTotal();
	void setTotal(int total){ m_total = total; }
	System::String^ getMessage();
	virtual void onRecvMsg(sigverse::RecvMsgEvent ^evt) override;
	virtual double onAction() override;
	
private:
	System::Collections::Generic::List<int>^ tmp_score;
	System::Collections::Generic::List<System::String^>^ tmp_msg;

	int m_total;
	System::String^ msg1;

};

  
iCub_Interface::~iCub_Interface()  
{  
  this->disconnect();  
}  

int iCub_Interface::getScore()
{
	int score = tmp_score[0];
	tmp_score->RemoveAt(0);
	
	return score;
}

int iCub_Interface::getTotal()
{
	return m_total;
}
System::String^ iCub_Interface::getMessage()
{
	//System::String^ msg = tmp_msg[0];
	//tmp_msg->RemoveAt(0);
	
	return msg1;
}

int iCub_Interface::getScoreSize()
{
	return tmp_score->Count;
}

double iCub_Interface::onAction()  
{  
  return 10.0;  
}  

void iCub_Interface::onRecvMsg(sigverse::RecvMsgEvent ^evt)  
{  
	std::cout<<"Message receved :" <<std::endl;

	System::String ^name = evt->getSender();
	System::String ^msg  = evt->getMsg();
	
	msg1 = msg;
	/*
	array<System::String^>^ SepString={"/"};

	// split
	array<System::String^>^ split_msg = msg->Split(SepString, System::StringSplitOptions::None);
			tmp_msg->Add(split_msg[0]);
	if(split_msg[0] == "AvatarInformation"){
		// name
	//	tmp_msg->Add(split_msg[0]);
		m_total = int::Parse(split_msg[1]);
		// score
for(int j = 0; j< m_total;j++)
  {

  }
          
	//	int score = int::Parse(split_msg[1]);
		tmp_score->Add(m_total);
	//	m_total += score;
		

	}

	*/

	
}

void writeText(char* desc){
    FILE * pFile;
    pFile = fopen ("Capture.txt","a+");
    fputs (desc,pFile);
    fclose(pFile);
}

std::string getMyIP(struct IPv4* myIP )
{
    char szBuffer[1024];
    std::string str = "";

    WSADATA wsaData;
    WORD wVersionRequested = MAKEWORD(2, 0);
    if(WSAStartup(wVersionRequested, &wsaData) != 0)
        return false;



    if(gethostname(szBuffer, sizeof(szBuffer)) == SOCKET_ERROR)
    {

      WSACleanup();

      return false;
    }

   hostent *host = gethostbyname(szBuffer);
    if(host == NULL)
    {

      WSACleanup();

      return false;
    }
    in_addr addr;
    //Obtain the computer's IP
   // for (int i = 0; host->h_addr_list[i] != 0; ++i)
   // {

        memcpy(&addr, host->h_addr_list[0], sizeof(in_addr));
    //    std::cout << "IPv4 Address:  " << inet_ntoa(addr) << std::endl;



  //  }

    myIP->b1 = (int)addr.S_un.S_un_b.s_b1;
    myIP->b2 = (int)addr.S_un.S_un_b.s_b2;
    myIP->b3 = (int)addr.S_un.S_un_b.s_b3;
    myIP->b4 = (int)addr.S_un.S_un_b.s_b4;
	



	char tm[32];
	sprintf_s(tm, 32, "%d", myIP->b1);
	 str += std::string(tm);
	 str += ".";

	
	sprintf_s(tm, 32, "%d", myIP->b2);
    str += std::string(tm);
	str += ".";

	
	sprintf_s(tm, 32, "%d", myIP->b3);
    str += std::string(tm);
	str += ".";

	
	sprintf_s(tm, 32, "%d", myIP->b4);
	str += std::string(tm);
	str += "|";

//std::cout << "IPv4 Address:  " << str << std::endl;

    WSACleanup();
    return str;
}

