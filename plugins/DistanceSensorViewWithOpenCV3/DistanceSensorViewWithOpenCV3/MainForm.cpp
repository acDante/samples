#include "MainForm.h"


using namespace DistanceSensorViewWithOpenCV3;

[STAThreadAttribute]
int main(array<System::String ^> ^args)
{
	// Enabling Windows XP visual effects before any controls are created
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false);

	// Create the main window and run it
	MainForm ^mform = gcnew MainForm();
	Application::Run(mform);

	return 0;
}
