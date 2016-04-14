#pragma once
#include "ViewClass.h"


using namespace System;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;
public ref class DistanceSensorView : public sigverse::SIGService
{

public:
	DistanceSensorView(System::String^ name) : SIGService(name)
	{
		this->view_class = gcnew Generic::List<ViewClass^>();
	};
	~DistanceSensorView(){
		this->disconnect();
	}

	int addDistanceSensor(System::String ^entity, int id){
		ViewClass ^vc = gcnew ViewClass();
		vc->initialize(this,entity, id, 500);
		view_class->Add(vc);
		return 0;
	}

	virtual void onRecvMsg(sigverse::RecvMsgEvent ^evt) override;
	virtual double onAction() override;
	Generic::List<ViewClass^>^ view_class;
};


double DistanceSensorView::onAction()
{
	for (int i = 0; i < this->view_class->Count; i++){
		if (this->view_class[i]->main_process()){
			this->view_class->RemoveAt(i);
		}
	}
	return 100.0;
}


void DistanceSensorView::onRecvMsg(sigverse::RecvMsgEvent ^evt)
{

	System::String ^name = evt->getSender();
	System::String ^msg = evt->getMsg();
}