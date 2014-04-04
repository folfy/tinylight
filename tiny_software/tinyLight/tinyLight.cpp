// tinyLight.cpp: Hauptprojektdatei.

#include "stdafx.h"
#include "tiny_protocol.h"
#include "Serial.h"
#include "Form1.h"

using namespace tinyLight;

[STAThreadAttribute]
int main(array<System::String ^> ^args)
{
	// Aktivieren visueller Effekte von Windows XP, bevor Steuerelemente erstellt werden
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false); 

	// Hauptfenster erstellen und ausführen
	Application::Run(gcnew Form1());
	
	

	return 0;
}