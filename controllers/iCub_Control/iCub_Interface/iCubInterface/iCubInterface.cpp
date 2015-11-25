// RobocupReferee.cpp : main project file.

#include "stdafx.h"
#include "Form1.h"
#include "iCubInterface.h"
#include <iostream>
#include <sstream>

using namespace iCubInterface;

[STAThreadAttribute]
int main(array<System::String ^> ^args)
{

	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false); 

	// Create the main window and run it
	Form1 ^mform = gcnew Form1();
	Application::Run(mform);

	return 0;
}
