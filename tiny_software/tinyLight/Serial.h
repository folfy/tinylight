//Serial

using namespace System;
using namespace System::IO::Ports;
using namespace System::Threading;


public ref class Serial
{
private:
	static class SerialPort^ _serialPort;
	static array<Byte>^ _response = {*response};

public:
	static String^ getDevice()
	{
		array<String^>^ serialPorts = SerialPort::GetPortNames();
		for each(String^ port in serialPorts)
		{
			SerialPort^ _test_port = gcnew SerialPort();
			if (Init(port, 115200) & pingDevice())
				return port;
		}
		return "N/A";
	}
	static bool pingDevice()
	{
		array<System::Byte>^ buff;
		array<Byte>^ ping;
		for(int x=0;x<=sizeof(preamble);x++)
		{
			ping->SetValue(preamble[x],x);
		}
		ping->SetValue(Byte(0x00),sizeof(preamble)+1);
		_serialPort->Write(ping,0,5);
		_serialPort->Read(buff,0,3);
		if(buff==_response)
			return true;
		else
			return false;
	}
	static bool Init(String^ Port, Int32 BaudRate)
	{
		//String^ data;
		//Thread^ readThread = gcnew Thread(gcnew ThreadStart(Serial::Read));

		_serialPort = gcnew SerialPort();

        // Set the appropriate properties.
        _serialPort->PortName = Port;
        _serialPort->BaudRate = BaudRate;
        _serialPort->Parity = Parity::None;
        _serialPort->DataBits = 8;
        _serialPort->StopBits = StopBits::One;
        _serialPort->Handshake = Handshake::None;

		if(_serialPort->IsOpen)
			_serialPort->Close();

		try
		{
			_serialPort->Open();
		}
		catch( const char * str )
		{
			return false;
		}
		
		pingDevice();
		//System::Windows::Forms::MessageBox::Show(ret, "Form Closing");
		//Msgbox(data);
		//_serialPort->DiscardInBuffer();
		return true;
	}
	//static void Read()
	//{

	//}
}