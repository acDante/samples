#pragma once
#include "stdafx.h"
#include <iostream>
#include <conio.h>
#include <winsock2.h>
#include<stdio.h>
#include<stdlib.h>
#include <windows.h>


public ref class iCub_Interface : public sigverse::SIGService  
{  
	
public:
	iCub_Interface(System::String^ name) : SIGService(name)
	{
		tmp_score = gcnew System::Collections::Generic::List<int>();
        msg1 = "";
	};
	~iCub_Interface();

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
	

	
}

void writeText(char* desc){
    FILE * pFile;
    pFile = fopen ("Capture.txt","a+");
    fputs (desc,pFile);
    fclose(pFile);
}

