

#include "iCubInterface.h"

#include <iostream>
#include<sstream>
#include <cstdlib>





namespace iCubInterface {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for Form1
	/// </summary>
	public ref class Form1 : public System::Windows::Forms::Form
	{
	public:
		Form1(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
			m_srv = nullptr;
			m_connected = false;
			cli::array<System::String^>^ args = System::Environment::GetCommandLineArgs();
           
			// 接続を試みる
			m_srv = gcnew iCub_Interface("Configure_Robot");
			

			//if(m_srv->connect("socio4.iir.nii.ac.jp", 9333)){
		/*	if(m_srv->connect(args[1], System::Convert::ToInt32(args[2]))){
				m_connected = true;
				this->connect->Enabled = false;
					 
				m_srv->connectToViewer();
				m_srv->setAutoExitProc(true);
			}*/
		}
		void setText(System::String^ text)
		{
			this->label1->Text = text;
		}
	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~Form1()
		{
			m_srv = nullptr;
			if (components)
			{
				delete components;
			}
		}
	private: System::ComponentModel::IContainer^  components;
	protected: 

	protected: 

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>

	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Button^  connect;


	private: System::Windows::Forms::Timer^  timer1;
	private: System::Windows::Forms::Button^  button1;
	private: System::Windows::Forms::Button^  button2;




	private: System::Windows::Forms::ListBox^  listBox5;	
	//private: System::Windows::Forms::ListBox^  listBox5;
			 
			 //sigverse::SIGService ^srv = gcnew sigverse::SIGService("RobocupReferee");
			 iCub_Interface ^m_srv;
	private: System::Windows::Forms::Panel^  panel1;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::TrackBar^  trackBar7;
	private: System::Windows::Forms::TrackBar^  trackBar6;
	private: System::Windows::Forms::TrackBar^  trackBar5;
	private: System::Windows::Forms::TrackBar^  trackBar4;
	private: System::Windows::Forms::TrackBar^  trackBar3;
	private: System::Windows::Forms::TrackBar^  trackBar2;
	private: System::Windows::Forms::TrackBar^  trackBar1;











































private: System::Windows::Forms::Button^  button3;
private: System::Windows::Forms::TextBox^  textBox1;
private: System::Windows::Forms::TextBox^  textBox2;
private: System::Windows::Forms::Label^  label8;
private: System::Windows::Forms::TextBox^  textBox3;
private: System::Windows::Forms::TextBox^  textBox4;
private: System::Windows::Forms::TextBox^  textBox5;
private: System::Windows::Forms::TextBox^  textBox6;
private: System::Windows::Forms::TextBox^  textBox8;
private: System::Windows::Forms::TextBox^  textBox7;
private: System::Windows::Forms::Panel^  panel2;
private: System::Windows::Forms::TextBox^  textBox9;
private: System::Windows::Forms::TextBox^  textBox10;
private: System::Windows::Forms::TextBox^  textBox11;
private: System::Windows::Forms::TextBox^  textBox12;
private: System::Windows::Forms::TextBox^  textBox13;
private: System::Windows::Forms::TextBox^  textBox14;
private: System::Windows::Forms::TextBox^  textBox15;
private: System::Windows::Forms::Label^  label3;
private: System::Windows::Forms::TrackBar^  trackBar8;
private: System::Windows::Forms::TrackBar^  trackBar9;
private: System::Windows::Forms::TrackBar^  trackBar10;
private: System::Windows::Forms::TrackBar^  trackBar11;
private: System::Windows::Forms::TrackBar^  trackBar12;
private: System::Windows::Forms::TrackBar^  trackBar13;
private: System::Windows::Forms::TrackBar^  trackBar14;
private: System::Windows::Forms::Label^  label4;
private: System::Windows::Forms::Label^  label5;
private: System::Windows::Forms::Label^  label9;
private: System::Windows::Forms::Label^  label10;
private: System::Windows::Forms::Label^  label11;
private: System::Windows::Forms::Label^  label12;
private: System::Windows::Forms::Panel^  panel3;
private: System::Windows::Forms::TextBox^  textBox17;
private: System::Windows::Forms::TextBox^  textBox18;
private: System::Windows::Forms::TextBox^  textBox19;
private: System::Windows::Forms::TextBox^  textBox20;
private: System::Windows::Forms::TextBox^  textBox21;
private: System::Windows::Forms::TextBox^  textBox22;
private: System::Windows::Forms::Label^  label13;
private: System::Windows::Forms::TrackBar^  trackBar16;
private: System::Windows::Forms::TrackBar^  trackBar17;
private: System::Windows::Forms::TrackBar^  trackBar18;
private: System::Windows::Forms::TrackBar^  trackBar19;
private: System::Windows::Forms::TrackBar^  trackBar20;
private: System::Windows::Forms::TrackBar^  trackBar21;
private: System::Windows::Forms::Panel^  panel4;
private: System::Windows::Forms::TextBox^  textBox24;
private: System::Windows::Forms::TextBox^  textBox25;
private: System::Windows::Forms::TextBox^  textBox26;
private: System::Windows::Forms::TextBox^  textBox27;
private: System::Windows::Forms::TextBox^  textBox28;
private: System::Windows::Forms::TextBox^  textBox29;
private: System::Windows::Forms::Label^  label14;
private: System::Windows::Forms::TrackBar^  trackBar23;
private: System::Windows::Forms::TrackBar^  trackBar24;
private: System::Windows::Forms::TrackBar^  trackBar25;
private: System::Windows::Forms::TrackBar^  trackBar26;
private: System::Windows::Forms::TrackBar^  trackBar27;
private: System::Windows::Forms::TrackBar^  trackBar28;
private: System::Windows::Forms::Label^  label6;
private: System::Windows::Forms::Label^  label7;
private: System::Windows::Forms::Label^  label15;
private: System::Windows::Forms::Panel^  panel5;
private: System::Windows::Forms::TextBox^  textBox16;
private: System::Windows::Forms::TextBox^  textBox23;
private: System::Windows::Forms::TextBox^  textBox30;
private: System::Windows::Forms::TextBox^  textBox31;
private: System::Windows::Forms::TextBox^  textBox32;
private: System::Windows::Forms::TextBox^  textBox33;
private: System::Windows::Forms::Label^  label16;
private: System::Windows::Forms::TrackBar^  trackBar15;
private: System::Windows::Forms::TrackBar^  trackBar22;
private: System::Windows::Forms::TrackBar^  trackBar29;
private: System::Windows::Forms::TrackBar^  trackBar30;
private: System::Windows::Forms::TrackBar^  trackBar31;
private: System::Windows::Forms::TrackBar^  trackBar32;
private: System::Windows::Forms::Panel^  panel6;
private: System::Windows::Forms::TextBox^  textBox34;
private: System::Windows::Forms::TextBox^  textBox35;
private: System::Windows::Forms::TextBox^  textBox36;
private: System::Windows::Forms::TextBox^  textBox37;
private: System::Windows::Forms::TextBox^  textBox38;
private: System::Windows::Forms::TextBox^  textBox39;
private: System::Windows::Forms::Label^  label17;
private: System::Windows::Forms::TrackBar^  trackBar33;
private: System::Windows::Forms::TrackBar^  trackBar34;
private: System::Windows::Forms::TrackBar^  trackBar35;
private: System::Windows::Forms::TrackBar^  trackBar36;
private: System::Windows::Forms::TrackBar^  trackBar37;
private: System::Windows::Forms::TrackBar^  trackBar38;
private: System::Windows::Forms::Panel^  panel7;
private: System::Windows::Forms::TextBox^  textBox40;
private: System::Windows::Forms::TextBox^  textBox41;
private: System::Windows::Forms::TextBox^  textBox42;
private: System::Windows::Forms::TextBox^  textBox43;
private: System::Windows::Forms::TextBox^  textBox44;
private: System::Windows::Forms::TextBox^  textBox45;
private: System::Windows::Forms::TextBox^  textBox46;
private: System::Windows::Forms::Label^  label18;
private: System::Windows::Forms::TrackBar^  trackBar39;
private: System::Windows::Forms::TrackBar^  trackBar40;
private: System::Windows::Forms::TrackBar^  trackBar41;
private: System::Windows::Forms::TrackBar^  trackBar42;
private: System::Windows::Forms::TrackBar^  trackBar43;
private: System::Windows::Forms::TrackBar^  trackBar44;
private: System::Windows::Forms::TrackBar^  trackBar45;
private: System::Windows::Forms::TextBox^  textBox50;
private: System::Windows::Forms::TextBox^  textBox51;
private: System::Windows::Forms::TrackBar^  trackBar49;
private: System::Windows::Forms::TrackBar^  trackBar50;
private: System::Windows::Forms::TextBox^  textBox52;
private: System::Windows::Forms::TextBox^  textBox53;
private: System::Windows::Forms::TrackBar^  trackBar51;
private: System::Windows::Forms::TrackBar^  trackBar52;
private: System::Windows::Forms::TextBox^  textBox47;
private: System::Windows::Forms::TextBox^  textBox48;
private: System::Windows::Forms::TextBox^  textBox49;
private: System::Windows::Forms::TrackBar^  trackBar46;
private: System::Windows::Forms::TrackBar^  trackBar47;
private: System::Windows::Forms::TrackBar^  trackBar48;
private: System::Windows::Forms::TextBox^  textBox54;
private: System::Windows::Forms::TrackBar^  trackBar53;
private: System::Windows::Forms::TextBox^  textBox55;
private: System::Windows::Forms::TrackBar^  trackBar54;
private: System::Windows::Forms::TextBox^  textBox56;
private: System::Windows::Forms::TrackBar^  trackBar55;
private: System::Windows::Forms::TextBox^  textBox57;
private: System::Windows::Forms::TextBox^  textBox58;
private: System::Windows::Forms::TextBox^  textBox59;
private: System::Windows::Forms::TrackBar^  trackBar56;
private: System::Windows::Forms::TrackBar^  trackBar57;
private: System::Windows::Forms::TrackBar^  trackBar58;
private: System::Windows::Forms::Panel^  panel8;
private: System::Windows::Forms::TextBox^  textBox60;
private: System::Windows::Forms::TrackBar^  trackBar59;
private: System::Windows::Forms::TextBox^  textBox61;
private: System::Windows::Forms::TrackBar^  trackBar60;
private: System::Windows::Forms::TextBox^  textBox62;
private: System::Windows::Forms::TextBox^  textBox63;
private: System::Windows::Forms::TextBox^  textBox64;
private: System::Windows::Forms::TrackBar^  trackBar61;
private: System::Windows::Forms::TrackBar^  trackBar62;
private: System::Windows::Forms::TrackBar^  trackBar63;
private: System::Windows::Forms::TextBox^  textBox65;
private: System::Windows::Forms::TrackBar^  trackBar64;
private: System::Windows::Forms::TextBox^  textBox66;
private: System::Windows::Forms::TextBox^  textBox67;
private: System::Windows::Forms::TrackBar^  trackBar65;
private: System::Windows::Forms::TrackBar^  trackBar66;
private: System::Windows::Forms::TextBox^  textBox68;
private: System::Windows::Forms::TextBox^  textBox69;
private: System::Windows::Forms::TrackBar^  trackBar67;
private: System::Windows::Forms::TrackBar^  trackBar68;
private: System::Windows::Forms::TextBox^  textBox70;
private: System::Windows::Forms::TextBox^  textBox71;
private: System::Windows::Forms::TextBox^  textBox72;
private: System::Windows::Forms::TrackBar^  trackBar69;
private: System::Windows::Forms::TrackBar^  trackBar70;
private: System::Windows::Forms::TrackBar^  trackBar71;
private: System::Windows::Forms::TextBox^  textBox73;
private: System::Windows::Forms::TextBox^  textBox74;
private: System::Windows::Forms::TextBox^  textBox75;
private: System::Windows::Forms::TextBox^  textBox76;
private: System::Windows::Forms::TextBox^  textBox77;
private: System::Windows::Forms::TextBox^  textBox78;
private: System::Windows::Forms::TextBox^  textBox79;
private: System::Windows::Forms::Label^  label19;
private: System::Windows::Forms::TrackBar^  trackBar72;
private: System::Windows::Forms::TrackBar^  trackBar73;
private: System::Windows::Forms::TrackBar^  trackBar74;
private: System::Windows::Forms::TrackBar^  trackBar75;
private: System::Windows::Forms::TrackBar^  trackBar76;
private: System::Windows::Forms::TrackBar^  trackBar77;
private: System::Windows::Forms::TrackBar^  trackBar78;
private: System::Windows::Forms::Label^  label20;
private: System::Windows::Forms::Label^  label21;
private: System::Windows::Forms::Label^  label22;
private: System::Windows::Forms::Label^  label23;
private: System::Windows::Forms::Label^  label24;
private: System::Windows::Forms::Label^  label25;
private: System::Windows::Forms::Label^  label26;
private: System::Windows::Forms::Label^  label27;
private: System::Windows::Forms::Label^  label28;
private: System::Windows::Forms::Label^  label29;
private: System::Windows::Forms::Label^  label30;
private: System::Windows::Forms::TextBox^  textBox80;
private: System::Windows::Forms::Button^  button4;
private: System::Windows::Forms::Button^  button5;
















			 bool m_connected;
			 //srv->connect("socio4.iir.nii.ac.jp", 9333);
			 
#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->connect = (gcnew System::Windows::Forms::Button());
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->panel1 = (gcnew System::Windows::Forms::Panel());
			this->textBox8 = (gcnew System::Windows::Forms::TextBox());
			this->textBox7 = (gcnew System::Windows::Forms::TextBox());
			this->textBox6 = (gcnew System::Windows::Forms::TextBox());
			this->textBox5 = (gcnew System::Windows::Forms::TextBox());
			this->textBox4 = (gcnew System::Windows::Forms::TextBox());
			this->textBox3 = (gcnew System::Windows::Forms::TextBox());
			this->textBox1 = (gcnew System::Windows::Forms::TextBox());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->trackBar7 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar6 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar5 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar4 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar3 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar2 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar1 = (gcnew System::Windows::Forms::TrackBar());
			this->button3 = (gcnew System::Windows::Forms::Button());
			this->textBox2 = (gcnew System::Windows::Forms::TextBox());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->panel2 = (gcnew System::Windows::Forms::Panel());
			this->textBox9 = (gcnew System::Windows::Forms::TextBox());
			this->textBox10 = (gcnew System::Windows::Forms::TextBox());
			this->textBox11 = (gcnew System::Windows::Forms::TextBox());
			this->textBox12 = (gcnew System::Windows::Forms::TextBox());
			this->textBox13 = (gcnew System::Windows::Forms::TextBox());
			this->textBox14 = (gcnew System::Windows::Forms::TextBox());
			this->textBox15 = (gcnew System::Windows::Forms::TextBox());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->trackBar8 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar9 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar10 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar11 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar12 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar13 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar14 = (gcnew System::Windows::Forms::TrackBar());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->panel3 = (gcnew System::Windows::Forms::Panel());
			this->textBox17 = (gcnew System::Windows::Forms::TextBox());
			this->textBox18 = (gcnew System::Windows::Forms::TextBox());
			this->textBox19 = (gcnew System::Windows::Forms::TextBox());
			this->textBox20 = (gcnew System::Windows::Forms::TextBox());
			this->textBox21 = (gcnew System::Windows::Forms::TextBox());
			this->textBox22 = (gcnew System::Windows::Forms::TextBox());
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->trackBar16 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar17 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar18 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar19 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar20 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar21 = (gcnew System::Windows::Forms::TrackBar());
			this->panel4 = (gcnew System::Windows::Forms::Panel());
			this->textBox24 = (gcnew System::Windows::Forms::TextBox());
			this->textBox25 = (gcnew System::Windows::Forms::TextBox());
			this->textBox26 = (gcnew System::Windows::Forms::TextBox());
			this->textBox27 = (gcnew System::Windows::Forms::TextBox());
			this->textBox28 = (gcnew System::Windows::Forms::TextBox());
			this->textBox29 = (gcnew System::Windows::Forms::TextBox());
			this->label14 = (gcnew System::Windows::Forms::Label());
			this->trackBar23 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar24 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar25 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar26 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar27 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar28 = (gcnew System::Windows::Forms::TrackBar());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->label15 = (gcnew System::Windows::Forms::Label());
			this->panel5 = (gcnew System::Windows::Forms::Panel());
			this->textBox16 = (gcnew System::Windows::Forms::TextBox());
			this->textBox23 = (gcnew System::Windows::Forms::TextBox());
			this->textBox30 = (gcnew System::Windows::Forms::TextBox());
			this->textBox31 = (gcnew System::Windows::Forms::TextBox());
			this->textBox32 = (gcnew System::Windows::Forms::TextBox());
			this->textBox39 = (gcnew System::Windows::Forms::TextBox());
			this->textBox33 = (gcnew System::Windows::Forms::TextBox());
			this->label16 = (gcnew System::Windows::Forms::Label());
			this->trackBar15 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar34 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar22 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar29 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar30 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar31 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar32 = (gcnew System::Windows::Forms::TrackBar());
			this->panel6 = (gcnew System::Windows::Forms::Panel());
			this->textBox34 = (gcnew System::Windows::Forms::TextBox());
			this->textBox35 = (gcnew System::Windows::Forms::TextBox());
			this->textBox36 = (gcnew System::Windows::Forms::TextBox());
			this->textBox37 = (gcnew System::Windows::Forms::TextBox());
			this->textBox38 = (gcnew System::Windows::Forms::TextBox());
			this->label17 = (gcnew System::Windows::Forms::Label());
			this->trackBar33 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar35 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar36 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar37 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar38 = (gcnew System::Windows::Forms::TrackBar());
			this->panel7 = (gcnew System::Windows::Forms::Panel());
			this->textBox55 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar54 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox56 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar55 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox57 = (gcnew System::Windows::Forms::TextBox());
			this->textBox58 = (gcnew System::Windows::Forms::TextBox());
			this->textBox59 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar56 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar57 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar58 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox54 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar53 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox50 = (gcnew System::Windows::Forms::TextBox());
			this->textBox51 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar49 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar50 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox52 = (gcnew System::Windows::Forms::TextBox());
			this->textBox53 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar51 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar52 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox47 = (gcnew System::Windows::Forms::TextBox());
			this->textBox48 = (gcnew System::Windows::Forms::TextBox());
			this->textBox49 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar46 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar47 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar48 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox40 = (gcnew System::Windows::Forms::TextBox());
			this->textBox41 = (gcnew System::Windows::Forms::TextBox());
			this->textBox42 = (gcnew System::Windows::Forms::TextBox());
			this->textBox43 = (gcnew System::Windows::Forms::TextBox());
			this->textBox44 = (gcnew System::Windows::Forms::TextBox());
			this->textBox45 = (gcnew System::Windows::Forms::TextBox());
			this->textBox46 = (gcnew System::Windows::Forms::TextBox());
			this->label18 = (gcnew System::Windows::Forms::Label());
			this->trackBar39 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar40 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar41 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar42 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar43 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar44 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar45 = (gcnew System::Windows::Forms::TrackBar());
			this->panel8 = (gcnew System::Windows::Forms::Panel());
			this->textBox60 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar59 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox61 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar60 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox62 = (gcnew System::Windows::Forms::TextBox());
			this->textBox63 = (gcnew System::Windows::Forms::TextBox());
			this->textBox64 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar61 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar62 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar63 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox65 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar64 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox66 = (gcnew System::Windows::Forms::TextBox());
			this->textBox67 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar65 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar66 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox68 = (gcnew System::Windows::Forms::TextBox());
			this->textBox69 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar67 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar68 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox70 = (gcnew System::Windows::Forms::TextBox());
			this->textBox71 = (gcnew System::Windows::Forms::TextBox());
			this->textBox72 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar69 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar70 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar71 = (gcnew System::Windows::Forms::TrackBar());
			this->textBox73 = (gcnew System::Windows::Forms::TextBox());
			this->textBox74 = (gcnew System::Windows::Forms::TextBox());
			this->textBox75 = (gcnew System::Windows::Forms::TextBox());
			this->textBox76 = (gcnew System::Windows::Forms::TextBox());
			this->textBox77 = (gcnew System::Windows::Forms::TextBox());
			this->textBox78 = (gcnew System::Windows::Forms::TextBox());
			this->textBox79 = (gcnew System::Windows::Forms::TextBox());
			this->label19 = (gcnew System::Windows::Forms::Label());
			this->trackBar72 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar73 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar74 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar75 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar76 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar77 = (gcnew System::Windows::Forms::TrackBar());
			this->trackBar78 = (gcnew System::Windows::Forms::TrackBar());
			this->label20 = (gcnew System::Windows::Forms::Label());
			this->label21 = (gcnew System::Windows::Forms::Label());
			this->label22 = (gcnew System::Windows::Forms::Label());
			this->label23 = (gcnew System::Windows::Forms::Label());
			this->label24 = (gcnew System::Windows::Forms::Label());
			this->label25 = (gcnew System::Windows::Forms::Label());
			this->label26 = (gcnew System::Windows::Forms::Label());
			this->label27 = (gcnew System::Windows::Forms::Label());
			this->label28 = (gcnew System::Windows::Forms::Label());
			this->label29 = (gcnew System::Windows::Forms::Label());
			this->label30 = (gcnew System::Windows::Forms::Label());
			this->textBox80 = (gcnew System::Windows::Forms::TextBox());
			this->button4 = (gcnew System::Windows::Forms::Button());
			this->button5 = (gcnew System::Windows::Forms::Button());
			this->panel1->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar7))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar6))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar5))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar4))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar3))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar2))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar1))->BeginInit();
			this->panel2->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar8))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar9))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar10))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar11))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar12))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar13))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar14))->BeginInit();
			this->panel3->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar16))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar17))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar18))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar19))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar20))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar21))->BeginInit();
			this->panel4->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar23))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar24))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar25))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar26))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar27))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar28))->BeginInit();
			this->panel5->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar15))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar34))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar22))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar29))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar30))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar31))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar32))->BeginInit();
			this->panel6->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar33))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar35))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar36))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar37))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar38))->BeginInit();
			this->panel7->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar54))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar55))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar56))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar57))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar58))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar53))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar49))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar50))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar51))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar52))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar46))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar47))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar48))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar39))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar40))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar41))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar42))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar43))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar44))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar45))->BeginInit();
			this->panel8->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar59))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar60))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar61))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar62))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar63))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar64))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar65))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar66))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar67))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar68))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar69))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar70))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar71))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar72))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar73))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar74))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar75))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar76))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar77))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar78))->BeginInit();
			this->SuspendLayout();
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 20.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label1->Location = System::Drawing::Point(623, 9);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(184, 31);
			this->label1->TabIndex = 0;
			this->label1->Text = L"iCub Interface";
			this->label1->Click += gcnew System::EventHandler(this, &Form1::label1_Click);
			// 
			// connect
			// 
			this->connect->Enabled = false;
			this->connect->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->connect->Location = System::Drawing::Point(55, 707);
			this->connect->Name = L"connect";
			this->connect->Size = System::Drawing::Size(110, 38);
			this->connect->TabIndex = 1;
			this->connect->Text = L"Connect";
			this->connect->UseVisualStyleBackColor = true;
			this->connect->Click += gcnew System::EventHandler(this, &Form1::button1_Click_1);
			// 
			// timer1
			// 
			this->timer1->Enabled = true;
			this->timer1->Interval = 10;
			this->timer1->Tick += gcnew System::EventHandler(this, &Form1::timer1_Tick);
			// 
			// button1
			// 
			this->button1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->button1->Location = System::Drawing::Point(171, 709);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(116, 37);
			this->button1->TabIndex = 2;
			this->button1->Text = L"Disconnect";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &Form1::button1_Click_2);
			// 
			// button2
			// 
			this->button2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->button2->Location = System::Drawing::Point(319, 709);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(85, 38);
			this->button2->TabIndex = 3;
			this->button2->Text = L"Capture";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &Form1::button2_Click);
			// 
			// panel1
			// 
			this->panel1->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->panel1->Controls->Add(this->textBox8);
			this->panel1->Controls->Add(this->textBox7);
			this->panel1->Controls->Add(this->textBox6);
			this->panel1->Controls->Add(this->textBox5);
			this->panel1->Controls->Add(this->textBox4);
			this->panel1->Controls->Add(this->textBox3);
			this->panel1->Controls->Add(this->textBox1);
			this->panel1->Controls->Add(this->label2);
			this->panel1->Controls->Add(this->trackBar7);
			this->panel1->Controls->Add(this->trackBar6);
			this->panel1->Controls->Add(this->trackBar5);
			this->panel1->Controls->Add(this->trackBar4);
			this->panel1->Controls->Add(this->trackBar3);
			this->panel1->Controls->Add(this->trackBar2);
			this->panel1->Controls->Add(this->trackBar1);
			this->panel1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->panel1->Location = System::Drawing::Point(47, 67);
			this->panel1->Name = L"panel1";
			this->panel1->Size = System::Drawing::Size(372, 201);
			this->panel1->TabIndex = 4;
			// 
			// textBox8
			// 
			this->textBox8->Location = System::Drawing::Point(271, 161);
			this->textBox8->Name = L"textBox8";
			this->textBox8->Size = System::Drawing::Size(54, 29);
			this->textBox8->TabIndex = 17;
			this->textBox8->Text = L"0";
			this->textBox8->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox8->TextChanged += gcnew System::EventHandler(this, &Form1::textBox8_TextChanged);
			// 
			// textBox7
			// 
			this->textBox7->Location = System::Drawing::Point(44, 161);
			this->textBox7->Name = L"textBox7";
			this->textBox7->Size = System::Drawing::Size(54, 29);
			this->textBox7->TabIndex = 16;
			this->textBox7->Text = L"0";
			this->textBox7->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox7->TextChanged += gcnew System::EventHandler(this, &Form1::textBox7_TextChanged);
			// 
			// textBox6
			// 
			this->textBox6->Location = System::Drawing::Point(271, 101);
			this->textBox6->Name = L"textBox6";
			this->textBox6->Size = System::Drawing::Size(54, 29);
			this->textBox6->TabIndex = 15;
			this->textBox6->Text = L"0";
			this->textBox6->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox6->TextChanged += gcnew System::EventHandler(this, &Form1::textBox6_TextChanged);
			// 
			// textBox5
			// 
			this->textBox5->Location = System::Drawing::Point(44, 101);
			this->textBox5->Name = L"textBox5";
			this->textBox5->Size = System::Drawing::Size(54, 29);
			this->textBox5->TabIndex = 14;
			this->textBox5->Text = L"0";
			this->textBox5->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox5->TextChanged += gcnew System::EventHandler(this, &Form1::textBox5_TextChanged);
			// 
			// textBox4
			// 
			this->textBox4->Location = System::Drawing::Point(44, 35);
			this->textBox4->Name = L"textBox4";
			this->textBox4->Size = System::Drawing::Size(54, 29);
			this->textBox4->TabIndex = 13;
			this->textBox4->Text = L"0";
			this->textBox4->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox4->TextChanged += gcnew System::EventHandler(this, &Form1::textBox4_TextChanged);
			// 
			// textBox3
			// 
			this->textBox3->Location = System::Drawing::Point(155, 35);
			this->textBox3->Name = L"textBox3";
			this->textBox3->Size = System::Drawing::Size(54, 29);
			this->textBox3->TabIndex = 12;
			this->textBox3->Text = L"0";
			this->textBox3->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox3->TextChanged += gcnew System::EventHandler(this, &Form1::textBox3_TextChanged);
			// 
			// textBox1
			// 
			this->textBox1->Location = System::Drawing::Point(276, 35);
			this->textBox1->Name = L"textBox1";
			this->textBox1->Size = System::Drawing::Size(49, 29);
			this->textBox1->TabIndex = 11;
			this->textBox1->Text = L"0";
			this->textBox1->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox1->TextChanged += gcnew System::EventHandler(this, &Form1::textBox1_TextChanged);
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label2->Location = System::Drawing::Point(125, 105);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(117, 25);
			this->label2->TabIndex = 5;
			this->label2->Text = L"LEFT ARM";
			// 
			// trackBar7
			// 
			this->trackBar7->LargeChange = 1;
			this->trackBar7->Location = System::Drawing::Point(238, 133);
			this->trackBar7->Maximum = 180;
			this->trackBar7->Minimum = -180;
			this->trackBar7->Name = L"trackBar7";
			this->trackBar7->Size = System::Drawing::Size(129, 45);
			this->trackBar7->TabIndex = 6;
			this->trackBar7->Scroll += gcnew System::EventHandler(this, &Form1::trackBar7_Scroll);
			// 
			// trackBar6
			// 
			this->trackBar6->LargeChange = 1;
			this->trackBar6->Location = System::Drawing::Point(238, 70);
			this->trackBar6->Maximum = 180;
			this->trackBar6->Minimum = -180;
			this->trackBar6->Name = L"trackBar6";
			this->trackBar6->Size = System::Drawing::Size(129, 45);
			this->trackBar6->TabIndex = 5;
			this->trackBar6->Scroll += gcnew System::EventHandler(this, &Form1::trackBar6_Scroll);
			// 
			// trackBar5
			// 
			this->trackBar5->LargeChange = 1;
			this->trackBar5->Location = System::Drawing::Point(238, 3);
			this->trackBar5->Maximum = 180;
			this->trackBar5->Minimum = -180;
			this->trackBar5->Name = L"trackBar5";
			this->trackBar5->Size = System::Drawing::Size(129, 45);
			this->trackBar5->TabIndex = 4;
			this->trackBar5->Scroll += gcnew System::EventHandler(this, &Form1::trackBar5_Scroll);
			// 
			// trackBar4
			// 
			this->trackBar4->LargeChange = 1;
			this->trackBar4->Location = System::Drawing::Point(123, 3);
			this->trackBar4->Maximum = 180;
			this->trackBar4->Minimum = -180;
			this->trackBar4->Name = L"trackBar4";
			this->trackBar4->Size = System::Drawing::Size(119, 45);
			this->trackBar4->TabIndex = 3;
			this->trackBar4->Scroll += gcnew System::EventHandler(this, &Form1::trackBar4_Scroll);
			// 
			// trackBar3
			// 
			this->trackBar3->LargeChange = 1;
			this->trackBar3->Location = System::Drawing::Point(3, 133);
			this->trackBar3->Maximum = 180;
			this->trackBar3->Minimum = -180;
			this->trackBar3->Name = L"trackBar3";
			this->trackBar3->Size = System::Drawing::Size(130, 45);
			this->trackBar3->TabIndex = 2;
			this->trackBar3->Scroll += gcnew System::EventHandler(this, &Form1::trackBar3_Scroll);
			// 
			// trackBar2
			// 
			this->trackBar2->LargeChange = 1;
			this->trackBar2->Location = System::Drawing::Point(3, 70);
			this->trackBar2->Maximum = 180;
			this->trackBar2->Minimum = -180;
			this->trackBar2->Name = L"trackBar2";
			this->trackBar2->Size = System::Drawing::Size(130, 45);
			this->trackBar2->TabIndex = 1;
			this->trackBar2->Scroll += gcnew System::EventHandler(this, &Form1::trackBar2_Scroll);
			// 
			// trackBar1
			// 
			this->trackBar1->LargeChange = 1;
			this->trackBar1->Location = System::Drawing::Point(3, 3);
			this->trackBar1->Maximum = 180;
			this->trackBar1->Minimum = -180;
			this->trackBar1->Name = L"trackBar1";
			this->trackBar1->Size = System::Drawing::Size(121, 45);
			this->trackBar1->TabIndex = 0;
			this->trackBar1->Scroll += gcnew System::EventHandler(this, &Form1::trackBar1_Scroll);
			// 
			// button3
			// 
			this->button3->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->button3->Location = System::Drawing::Point(769, 708);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(85, 38);
			this->button3->TabIndex = 10;
			this->button3->Text = L"Update";
			this->button3->UseVisualStyleBackColor = true;
			this->button3->Click += gcnew System::EventHandler(this, &Form1::button3_Click_1);
			// 
			// textBox2
			// 
			this->textBox2->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->textBox2->Location = System::Drawing::Point(47, 813);
			this->textBox2->Name = L"textBox2";
			this->textBox2->Size = System::Drawing::Size(125, 26);
			this->textBox2->TabIndex = 11;
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label8->Location = System::Drawing::Point(33, 776);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(176, 19);
			this->label8->TabIndex = 12;
			this->label8->Text = L"Capture Sequence";
			// 
			// panel2
			// 
			this->panel2->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->panel2->Controls->Add(this->textBox9);
			this->panel2->Controls->Add(this->textBox10);
			this->panel2->Controls->Add(this->textBox11);
			this->panel2->Controls->Add(this->textBox12);
			this->panel2->Controls->Add(this->textBox13);
			this->panel2->Controls->Add(this->textBox14);
			this->panel2->Controls->Add(this->textBox15);
			this->panel2->Controls->Add(this->label3);
			this->panel2->Controls->Add(this->trackBar8);
			this->panel2->Controls->Add(this->trackBar9);
			this->panel2->Controls->Add(this->trackBar10);
			this->panel2->Controls->Add(this->trackBar11);
			this->panel2->Controls->Add(this->trackBar12);
			this->panel2->Controls->Add(this->trackBar13);
			this->panel2->Controls->Add(this->trackBar14);
			this->panel2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->panel2->Location = System::Drawing::Point(482, 67);
			this->panel2->Name = L"panel2";
			this->panel2->Size = System::Drawing::Size(372, 201);
			this->panel2->TabIndex = 13;
			// 
			// textBox9
			// 
			this->textBox9->Location = System::Drawing::Point(264, 161);
			this->textBox9->Name = L"textBox9";
			this->textBox9->Size = System::Drawing::Size(54, 29);
			this->textBox9->TabIndex = 17;
			this->textBox9->Text = L"0";
			this->textBox9->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox9->TextChanged += gcnew System::EventHandler(this, &Form1::textBox9_TextChanged);
			// 
			// textBox10
			// 
			this->textBox10->Location = System::Drawing::Point(44, 161);
			this->textBox10->Name = L"textBox10";
			this->textBox10->Size = System::Drawing::Size(54, 29);
			this->textBox10->TabIndex = 16;
			this->textBox10->Text = L"0";
			this->textBox10->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox10->TextChanged += gcnew System::EventHandler(this, &Form1::textBox10_TextChanged);
			// 
			// textBox11
			// 
			this->textBox11->Location = System::Drawing::Point(264, 101);
			this->textBox11->Name = L"textBox11";
			this->textBox11->Size = System::Drawing::Size(54, 29);
			this->textBox11->TabIndex = 15;
			this->textBox11->Text = L"0";
			this->textBox11->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox11->TextChanged += gcnew System::EventHandler(this, &Form1::textBox11_TextChanged);
			// 
			// textBox12
			// 
			this->textBox12->Location = System::Drawing::Point(44, 101);
			this->textBox12->Name = L"textBox12";
			this->textBox12->Size = System::Drawing::Size(54, 29);
			this->textBox12->TabIndex = 14;
			this->textBox12->Text = L"0";
			this->textBox12->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox12->TextChanged += gcnew System::EventHandler(this, &Form1::textBox12_TextChanged);
			// 
			// textBox13
			// 
			this->textBox13->Location = System::Drawing::Point(44, 35);
			this->textBox13->Name = L"textBox13";
			this->textBox13->Size = System::Drawing::Size(54, 29);
			this->textBox13->TabIndex = 13;
			this->textBox13->Text = L"0";
			this->textBox13->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox13->TextChanged += gcnew System::EventHandler(this, &Form1::textBox13_TextChanged);
			// 
			// textBox14
			// 
			this->textBox14->Location = System::Drawing::Point(157, 35);
			this->textBox14->Name = L"textBox14";
			this->textBox14->Size = System::Drawing::Size(54, 29);
			this->textBox14->TabIndex = 12;
			this->textBox14->Text = L"0";
			this->textBox14->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox14->TextChanged += gcnew System::EventHandler(this, &Form1::textBox14_TextChanged);
			// 
			// textBox15
			// 
			this->textBox15->Location = System::Drawing::Point(269, 35);
			this->textBox15->Name = L"textBox15";
			this->textBox15->Size = System::Drawing::Size(49, 29);
			this->textBox15->TabIndex = 11;
			this->textBox15->Text = L"0";
			this->textBox15->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox15->TextChanged += gcnew System::EventHandler(this, &Form1::textBox15_TextChanged);
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label3->Location = System::Drawing::Point(125, 105);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(129, 25);
			this->label3->TabIndex = 5;
			this->label3->Text = L"RIGHT ARM";
			// 
			// trackBar8
			// 
			this->trackBar8->Location = System::Drawing::Point(240, 133);
			this->trackBar8->Maximum = 180;
			this->trackBar8->Minimum = -180;
			this->trackBar8->Name = L"trackBar8";
			this->trackBar8->Size = System::Drawing::Size(104, 45);
			this->trackBar8->TabIndex = 6;
			this->trackBar8->Scroll += gcnew System::EventHandler(this, &Form1::trackBar8_Scroll);
			// 
			// trackBar9
			// 
			this->trackBar9->Location = System::Drawing::Point(240, 70);
			this->trackBar9->Maximum = 180;
			this->trackBar9->Minimum = -180;
			this->trackBar9->Name = L"trackBar9";
			this->trackBar9->Size = System::Drawing::Size(104, 45);
			this->trackBar9->TabIndex = 5;
			this->trackBar9->Scroll += gcnew System::EventHandler(this, &Form1::trackBar9_Scroll);
			// 
			// trackBar10
			// 
			this->trackBar10->Location = System::Drawing::Point(240, 3);
			this->trackBar10->Maximum = 180;
			this->trackBar10->Minimum = -180;
			this->trackBar10->Name = L"trackBar10";
			this->trackBar10->Size = System::Drawing::Size(104, 45);
			this->trackBar10->TabIndex = 4;
			this->trackBar10->Scroll += gcnew System::EventHandler(this, &Form1::trackBar10_Scroll);
			// 
			// trackBar11
			// 
			this->trackBar11->Location = System::Drawing::Point(130, 3);
			this->trackBar11->Maximum = 180;
			this->trackBar11->Minimum = -180;
			this->trackBar11->Name = L"trackBar11";
			this->trackBar11->Size = System::Drawing::Size(104, 45);
			this->trackBar11->TabIndex = 3;
			this->trackBar11->Scroll += gcnew System::EventHandler(this, &Form1::trackBar11_Scroll);
			// 
			// trackBar12
			// 
			this->trackBar12->Location = System::Drawing::Point(20, 133);
			this->trackBar12->Maximum = 180;
			this->trackBar12->Minimum = -180;
			this->trackBar12->Name = L"trackBar12";
			this->trackBar12->Size = System::Drawing::Size(104, 45);
			this->trackBar12->TabIndex = 2;
			this->trackBar12->Scroll += gcnew System::EventHandler(this, &Form1::trackBar12_Scroll);
			// 
			// trackBar13
			// 
			this->trackBar13->Location = System::Drawing::Point(20, 70);
			this->trackBar13->Maximum = 180;
			this->trackBar13->Minimum = -180;
			this->trackBar13->Name = L"trackBar13";
			this->trackBar13->Size = System::Drawing::Size(104, 45);
			this->trackBar13->TabIndex = 1;
			this->trackBar13->Scroll += gcnew System::EventHandler(this, &Form1::trackBar13_Scroll);
			// 
			// trackBar14
			// 
			this->trackBar14->Location = System::Drawing::Point(20, 3);
			this->trackBar14->Maximum = 180;
			this->trackBar14->Minimum = -180;
			this->trackBar14->Name = L"trackBar14";
			this->trackBar14->Size = System::Drawing::Size(104, 45);
			this->trackBar14->TabIndex = 0;
			this->trackBar14->Scroll += gcnew System::EventHandler(this, &Form1::trackBar14_Scroll);
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label4->Location = System::Drawing::Point(437, 95);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(24, 21);
			this->label4->TabIndex = 14;
			this->label4->Text = L"X";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label5->Location = System::Drawing::Point(437, 169);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(23, 21);
			this->label5->TabIndex = 15;
			this->label5->Text = L"Y";
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 15.75F, System::Drawing::FontStyle::Italic, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label9->Location = System::Drawing::Point(437, 233);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(22, 21);
			this->label9->TabIndex = 16;
			this->label9->Text = L"Z";
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 15.75F, System::Drawing::FontStyle::Italic, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label10->Location = System::Drawing::Point(437, 443);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(22, 21);
			this->label10->TabIndex = 21;
			this->label10->Text = L"Z";
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label11->Location = System::Drawing::Point(437, 379);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(23, 21);
			this->label11->TabIndex = 20;
			this->label11->Text = L"Y";
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label12->Location = System::Drawing::Point(437, 305);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(24, 21);
			this->label12->TabIndex = 19;
			this->label12->Text = L"X";
			// 
			// panel3
			// 
			this->panel3->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->panel3->Controls->Add(this->textBox17);
			this->panel3->Controls->Add(this->textBox18);
			this->panel3->Controls->Add(this->textBox19);
			this->panel3->Controls->Add(this->textBox20);
			this->panel3->Controls->Add(this->textBox21);
			this->panel3->Controls->Add(this->textBox22);
			this->panel3->Controls->Add(this->label13);
			this->panel3->Controls->Add(this->trackBar16);
			this->panel3->Controls->Add(this->trackBar17);
			this->panel3->Controls->Add(this->trackBar18);
			this->panel3->Controls->Add(this->trackBar19);
			this->panel3->Controls->Add(this->trackBar20);
			this->panel3->Controls->Add(this->trackBar21);
			this->panel3->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->panel3->Location = System::Drawing::Point(482, 277);
			this->panel3->Name = L"panel3";
			this->panel3->Size = System::Drawing::Size(372, 201);
			this->panel3->TabIndex = 18;
			// 
			// textBox17
			// 
			this->textBox17->Location = System::Drawing::Point(44, 161);
			this->textBox17->Name = L"textBox17";
			this->textBox17->Size = System::Drawing::Size(54, 29);
			this->textBox17->TabIndex = 16;
			this->textBox17->Text = L"0";
			this->textBox17->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox17->TextChanged += gcnew System::EventHandler(this, &Form1::textBox17_TextChanged);
			// 
			// textBox18
			// 
			this->textBox18->Location = System::Drawing::Point(270, 161);
			this->textBox18->Name = L"textBox18";
			this->textBox18->Size = System::Drawing::Size(54, 29);
			this->textBox18->TabIndex = 15;
			this->textBox18->Text = L"0";
			this->textBox18->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox18->TextChanged += gcnew System::EventHandler(this, &Form1::textBox18_TextChanged);
			// 
			// textBox19
			// 
			this->textBox19->Location = System::Drawing::Point(44, 101);
			this->textBox19->Name = L"textBox19";
			this->textBox19->Size = System::Drawing::Size(54, 29);
			this->textBox19->TabIndex = 14;
			this->textBox19->Text = L"0";
			this->textBox19->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox19->TextChanged += gcnew System::EventHandler(this, &Form1::textBox19_TextChanged);
			// 
			// textBox20
			// 
			this->textBox20->Location = System::Drawing::Point(44, 35);
			this->textBox20->Name = L"textBox20";
			this->textBox20->Size = System::Drawing::Size(54, 29);
			this->textBox20->TabIndex = 13;
			this->textBox20->Text = L"0";
			this->textBox20->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox20->TextChanged += gcnew System::EventHandler(this, &Form1::textBox20_TextChanged);
			// 
			// textBox21
			// 
			this->textBox21->Location = System::Drawing::Point(157, 35);
			this->textBox21->Name = L"textBox21";
			this->textBox21->Size = System::Drawing::Size(54, 29);
			this->textBox21->TabIndex = 12;
			this->textBox21->Text = L"0";
			this->textBox21->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox21->TextChanged += gcnew System::EventHandler(this, &Form1::textBox21_TextChanged);
			// 
			// textBox22
			// 
			this->textBox22->Location = System::Drawing::Point(269, 35);
			this->textBox22->Name = L"textBox22";
			this->textBox22->Size = System::Drawing::Size(49, 29);
			this->textBox22->TabIndex = 11;
			this->textBox22->Text = L"0";
			this->textBox22->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox22->TextChanged += gcnew System::EventHandler(this, &Form1::textBox22_TextChanged);
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label13->Location = System::Drawing::Point(141, 94);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(124, 25);
			this->label13->TabIndex = 5;
			this->label13->Text = L"RIGHT LEG";
			// 
			// trackBar16
			// 
			this->trackBar16->Location = System::Drawing::Point(229, 133);
			this->trackBar16->Maximum = 180;
			this->trackBar16->Minimum = -180;
			this->trackBar16->Name = L"trackBar16";
			this->trackBar16->Size = System::Drawing::Size(127, 45);
			this->trackBar16->TabIndex = 5;
			this->trackBar16->Scroll += gcnew System::EventHandler(this, &Form1::trackBar16_Scroll);
			// 
			// trackBar17
			// 
			this->trackBar17->Location = System::Drawing::Point(240, 3);
			this->trackBar17->Maximum = 180;
			this->trackBar17->Minimum = -180;
			this->trackBar17->Name = L"trackBar17";
			this->trackBar17->Size = System::Drawing::Size(116, 45);
			this->trackBar17->TabIndex = 4;
			this->trackBar17->Scroll += gcnew System::EventHandler(this, &Form1::trackBar17_Scroll);
			// 
			// trackBar18
			// 
			this->trackBar18->Location = System::Drawing::Point(130, 3);
			this->trackBar18->Maximum = 180;
			this->trackBar18->Minimum = -180;
			this->trackBar18->Name = L"trackBar18";
			this->trackBar18->Size = System::Drawing::Size(112, 45);
			this->trackBar18->TabIndex = 3;
			this->trackBar18->Scroll += gcnew System::EventHandler(this, &Form1::trackBar18_Scroll);
			// 
			// trackBar19
			// 
			this->trackBar19->Location = System::Drawing::Point(20, 133);
			this->trackBar19->Maximum = 180;
			this->trackBar19->Minimum = -180;
			this->trackBar19->Name = L"trackBar19";
			this->trackBar19->Size = System::Drawing::Size(111, 45);
			this->trackBar19->TabIndex = 2;
			this->trackBar19->Scroll += gcnew System::EventHandler(this, &Form1::trackBar19_Scroll);
			// 
			// trackBar20
			// 
			this->trackBar20->Location = System::Drawing::Point(20, 70);
			this->trackBar20->Maximum = 180;
			this->trackBar20->Minimum = -180;
			this->trackBar20->Name = L"trackBar20";
			this->trackBar20->Size = System::Drawing::Size(111, 45);
			this->trackBar20->TabIndex = 1;
			this->trackBar20->Scroll += gcnew System::EventHandler(this, &Form1::trackBar20_Scroll);
			// 
			// trackBar21
			// 
			this->trackBar21->Location = System::Drawing::Point(20, 3);
			this->trackBar21->Maximum = 180;
			this->trackBar21->Minimum = -180;
			this->trackBar21->Name = L"trackBar21";
			this->trackBar21->Size = System::Drawing::Size(111, 45);
			this->trackBar21->TabIndex = 0;
			this->trackBar21->Scroll += gcnew System::EventHandler(this, &Form1::trackBar21_Scroll);
			// 
			// panel4
			// 
			this->panel4->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->panel4->Controls->Add(this->textBox24);
			this->panel4->Controls->Add(this->textBox25);
			this->panel4->Controls->Add(this->textBox26);
			this->panel4->Controls->Add(this->textBox27);
			this->panel4->Controls->Add(this->textBox28);
			this->panel4->Controls->Add(this->textBox29);
			this->panel4->Controls->Add(this->label14);
			this->panel4->Controls->Add(this->trackBar23);
			this->panel4->Controls->Add(this->trackBar24);
			this->panel4->Controls->Add(this->trackBar25);
			this->panel4->Controls->Add(this->trackBar26);
			this->panel4->Controls->Add(this->trackBar27);
			this->panel4->Controls->Add(this->trackBar28);
			this->panel4->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->panel4->Location = System::Drawing::Point(47, 277);
			this->panel4->Name = L"panel4";
			this->panel4->Size = System::Drawing::Size(372, 201);
			this->panel4->TabIndex = 17;
			// 
			// textBox24
			// 
			this->textBox24->Location = System::Drawing::Point(44, 161);
			this->textBox24->Name = L"textBox24";
			this->textBox24->Size = System::Drawing::Size(54, 29);
			this->textBox24->TabIndex = 16;
			this->textBox24->Text = L"0";
			this->textBox24->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox24->TextChanged += gcnew System::EventHandler(this, &Form1::textBox24_TextChanged);
			// 
			// textBox25
			// 
			this->textBox25->Location = System::Drawing::Point(271, 161);
			this->textBox25->Name = L"textBox25";
			this->textBox25->Size = System::Drawing::Size(54, 29);
			this->textBox25->TabIndex = 15;
			this->textBox25->Text = L"0";
			this->textBox25->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox25->TextChanged += gcnew System::EventHandler(this, &Form1::textBox25_TextChanged);
			// 
			// textBox26
			// 
			this->textBox26->Location = System::Drawing::Point(44, 101);
			this->textBox26->Name = L"textBox26";
			this->textBox26->Size = System::Drawing::Size(54, 29);
			this->textBox26->TabIndex = 14;
			this->textBox26->Text = L"0";
			this->textBox26->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox26->TextChanged += gcnew System::EventHandler(this, &Form1::textBox26_TextChanged);
			// 
			// textBox27
			// 
			this->textBox27->Location = System::Drawing::Point(44, 35);
			this->textBox27->Name = L"textBox27";
			this->textBox27->Size = System::Drawing::Size(54, 29);
			this->textBox27->TabIndex = 13;
			this->textBox27->Text = L"0";
			this->textBox27->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox27->TextChanged += gcnew System::EventHandler(this, &Form1::textBox27_TextChanged);
			// 
			// textBox28
			// 
			this->textBox28->Location = System::Drawing::Point(155, 35);
			this->textBox28->Name = L"textBox28";
			this->textBox28->Size = System::Drawing::Size(54, 29);
			this->textBox28->TabIndex = 12;
			this->textBox28->Text = L"0";
			this->textBox28->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox28->TextChanged += gcnew System::EventHandler(this, &Form1::textBox28_TextChanged);
			// 
			// textBox29
			// 
			this->textBox29->Location = System::Drawing::Point(276, 35);
			this->textBox29->Name = L"textBox29";
			this->textBox29->Size = System::Drawing::Size(49, 29);
			this->textBox29->TabIndex = 11;
			this->textBox29->Text = L"0";
			this->textBox29->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox29->TextChanged += gcnew System::EventHandler(this, &Form1::textBox29_TextChanged);
			// 
			// label14
			// 
			this->label14->AutoSize = true;
			this->label14->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label14->Location = System::Drawing::Point(139, 90);
			this->label14->Name = L"label14";
			this->label14->Size = System::Drawing::Size(112, 25);
			this->label14->TabIndex = 5;
			this->label14->Text = L"LEFT LEG";
			// 
			// trackBar23
			// 
			this->trackBar23->Location = System::Drawing::Point(238, 133);
			this->trackBar23->Maximum = 180;
			this->trackBar23->Minimum = -180;
			this->trackBar23->Name = L"trackBar23";
			this->trackBar23->Size = System::Drawing::Size(129, 45);
			this->trackBar23->TabIndex = 5;
			this->trackBar23->Scroll += gcnew System::EventHandler(this, &Form1::trackBar23_Scroll);
			// 
			// trackBar24
			// 
			this->trackBar24->Location = System::Drawing::Point(238, 3);
			this->trackBar24->Maximum = 180;
			this->trackBar24->Minimum = -180;
			this->trackBar24->Name = L"trackBar24";
			this->trackBar24->Size = System::Drawing::Size(129, 45);
			this->trackBar24->TabIndex = 4;
			this->trackBar24->Scroll += gcnew System::EventHandler(this, &Form1::trackBar24_Scroll);
			// 
			// trackBar25
			// 
			this->trackBar25->Location = System::Drawing::Point(123, 3);
			this->trackBar25->Maximum = 180;
			this->trackBar25->Minimum = -180;
			this->trackBar25->Name = L"trackBar25";
			this->trackBar25->Size = System::Drawing::Size(119, 45);
			this->trackBar25->TabIndex = 3;
			this->trackBar25->Scroll += gcnew System::EventHandler(this, &Form1::trackBar25_Scroll);
			// 
			// trackBar26
			// 
			this->trackBar26->Location = System::Drawing::Point(3, 133);
			this->trackBar26->Maximum = 180;
			this->trackBar26->Minimum = -180;
			this->trackBar26->Name = L"trackBar26";
			this->trackBar26->Size = System::Drawing::Size(130, 45);
			this->trackBar26->TabIndex = 2;
			this->trackBar26->Scroll += gcnew System::EventHandler(this, &Form1::trackBar26_Scroll);
			// 
			// trackBar27
			// 
			this->trackBar27->Location = System::Drawing::Point(3, 70);
			this->trackBar27->Maximum = 180;
			this->trackBar27->Minimum = -180;
			this->trackBar27->Name = L"trackBar27";
			this->trackBar27->Size = System::Drawing::Size(130, 45);
			this->trackBar27->TabIndex = 1;
			this->trackBar27->Scroll += gcnew System::EventHandler(this, &Form1::trackBar27_Scroll);
			// 
			// trackBar28
			// 
			this->trackBar28->Location = System::Drawing::Point(3, 3);
			this->trackBar28->Maximum = 180;
			this->trackBar28->Minimum = -180;
			this->trackBar28->Name = L"trackBar28";
			this->trackBar28->Size = System::Drawing::Size(121, 45);
			this->trackBar28->TabIndex = 0;
			this->trackBar28->Scroll += gcnew System::EventHandler(this, &Form1::trackBar28_Scroll);
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 15.75F, System::Drawing::FontStyle::Italic, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label6->Location = System::Drawing::Point(437, 652);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(22, 21);
			this->label6->TabIndex = 26;
			this->label6->Text = L"Z";
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label7->Location = System::Drawing::Point(437, 588);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(23, 21);
			this->label7->TabIndex = 25;
			this->label7->Text = L"Y";
			// 
			// label15
			// 
			this->label15->AutoSize = true;
			this->label15->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label15->Location = System::Drawing::Point(437, 514);
			this->label15->Name = L"label15";
			this->label15->Size = System::Drawing::Size(24, 21);
			this->label15->TabIndex = 24;
			this->label15->Text = L"X";
			// 
			// panel5
			// 
			this->panel5->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->panel5->Controls->Add(this->textBox16);
			this->panel5->Controls->Add(this->textBox23);
			this->panel5->Controls->Add(this->textBox30);
			this->panel5->Controls->Add(this->textBox31);
			this->panel5->Controls->Add(this->textBox32);
			this->panel5->Controls->Add(this->textBox39);
			this->panel5->Controls->Add(this->textBox33);
			this->panel5->Controls->Add(this->label16);
			this->panel5->Controls->Add(this->trackBar15);
			this->panel5->Controls->Add(this->trackBar34);
			this->panel5->Controls->Add(this->trackBar22);
			this->panel5->Controls->Add(this->trackBar29);
			this->panel5->Controls->Add(this->trackBar30);
			this->panel5->Controls->Add(this->trackBar31);
			this->panel5->Controls->Add(this->trackBar32);
			this->panel5->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->panel5->Location = System::Drawing::Point(482, 484);
			this->panel5->Name = L"panel5";
			this->panel5->Size = System::Drawing::Size(372, 201);
			this->panel5->TabIndex = 23;
			// 
			// textBox16
			// 
			this->textBox16->Location = System::Drawing::Point(44, 161);
			this->textBox16->Name = L"textBox16";
			this->textBox16->Size = System::Drawing::Size(54, 29);
			this->textBox16->TabIndex = 16;
			this->textBox16->Text = L"0";
			this->textBox16->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox16->TextChanged += gcnew System::EventHandler(this, &Form1::textBox16_TextChanged);
			// 
			// textBox23
			// 
			this->textBox23->Location = System::Drawing::Point(157, 103);
			this->textBox23->Name = L"textBox23";
			this->textBox23->Size = System::Drawing::Size(54, 29);
			this->textBox23->TabIndex = 15;
			this->textBox23->Text = L"0";
			this->textBox23->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox23->TextChanged += gcnew System::EventHandler(this, &Form1::textBox23_TextChanged);
			// 
			// textBox30
			// 
			this->textBox30->Location = System::Drawing::Point(44, 101);
			this->textBox30->Name = L"textBox30";
			this->textBox30->Size = System::Drawing::Size(54, 29);
			this->textBox30->TabIndex = 14;
			this->textBox30->Text = L"0";
			this->textBox30->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox30->TextChanged += gcnew System::EventHandler(this, &Form1::textBox30_TextChanged);
			// 
			// textBox31
			// 
			this->textBox31->Location = System::Drawing::Point(44, 35);
			this->textBox31->Name = L"textBox31";
			this->textBox31->Size = System::Drawing::Size(54, 29);
			this->textBox31->TabIndex = 13;
			this->textBox31->Text = L"0";
			this->textBox31->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox31->TextChanged += gcnew System::EventHandler(this, &Form1::textBox31_TextChanged);
			// 
			// textBox32
			// 
			this->textBox32->Location = System::Drawing::Point(157, 35);
			this->textBox32->Name = L"textBox32";
			this->textBox32->Size = System::Drawing::Size(54, 29);
			this->textBox32->TabIndex = 12;
			this->textBox32->Text = L"0";
			this->textBox32->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox32->TextChanged += gcnew System::EventHandler(this, &Form1::textBox32_TextChanged);
			// 
			// textBox39
			// 
			this->textBox39->Location = System::Drawing::Point(264, 105);
			this->textBox39->Name = L"textBox39";
			this->textBox39->Size = System::Drawing::Size(49, 29);
			this->textBox39->TabIndex = 11;
			this->textBox39->Text = L"0";
			this->textBox39->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox39->TextChanged += gcnew System::EventHandler(this, &Form1::textBox39_TextChanged);
			// 
			// textBox33
			// 
			this->textBox33->Location = System::Drawing::Point(269, 35);
			this->textBox33->Name = L"textBox33";
			this->textBox33->Size = System::Drawing::Size(49, 29);
			this->textBox33->TabIndex = 11;
			this->textBox33->Text = L"0";
			this->textBox33->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox33->TextChanged += gcnew System::EventHandler(this, &Form1::textBox33_TextChanged);
			// 
			// label16
			// 
			this->label16->AutoSize = true;
			this->label16->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label16->Location = System::Drawing::Point(169, 155);
			this->label16->Name = L"label16";
			this->label16->Size = System::Drawing::Size(175, 25);
			this->label16->TabIndex = 5;
			this->label16->Text = L"HEAD and EYES";
			// 
			// trackBar15
			// 
			this->trackBar15->Location = System::Drawing::Point(130, 72);
			this->trackBar15->Maximum = 180;
			this->trackBar15->Minimum = -180;
			this->trackBar15->Name = L"trackBar15";
			this->trackBar15->Size = System::Drawing::Size(104, 45);
			this->trackBar15->TabIndex = 5;
			this->trackBar15->Scroll += gcnew System::EventHandler(this, &Form1::trackBar15_Scroll);
			// 
			// trackBar34
			// 
			this->trackBar34->Location = System::Drawing::Point(240, 72);
			this->trackBar34->Maximum = 180;
			this->trackBar34->Minimum = -180;
			this->trackBar34->Name = L"trackBar34";
			this->trackBar34->Size = System::Drawing::Size(104, 45);
			this->trackBar34->TabIndex = 4;
			this->trackBar34->Scroll += gcnew System::EventHandler(this, &Form1::trackBar34_Scroll);
			// 
			// trackBar22
			// 
			this->trackBar22->Location = System::Drawing::Point(240, 3);
			this->trackBar22->Maximum = 180;
			this->trackBar22->Minimum = -180;
			this->trackBar22->Name = L"trackBar22";
			this->trackBar22->Size = System::Drawing::Size(104, 45);
			this->trackBar22->TabIndex = 4;
			this->trackBar22->Scroll += gcnew System::EventHandler(this, &Form1::trackBar22_Scroll);
			// 
			// trackBar29
			// 
			this->trackBar29->Location = System::Drawing::Point(130, 3);
			this->trackBar29->Maximum = 180;
			this->trackBar29->Minimum = -180;
			this->trackBar29->Name = L"trackBar29";
			this->trackBar29->Size = System::Drawing::Size(104, 45);
			this->trackBar29->TabIndex = 3;
			this->trackBar29->Scroll += gcnew System::EventHandler(this, &Form1::trackBar29_Scroll);
			// 
			// trackBar30
			// 
			this->trackBar30->Location = System::Drawing::Point(20, 133);
			this->trackBar30->Maximum = 180;
			this->trackBar30->Minimum = -180;
			this->trackBar30->Name = L"trackBar30";
			this->trackBar30->Size = System::Drawing::Size(104, 45);
			this->trackBar30->TabIndex = 2;
			this->trackBar30->Scroll += gcnew System::EventHandler(this, &Form1::trackBar30_Scroll);
			// 
			// trackBar31
			// 
			this->trackBar31->Location = System::Drawing::Point(20, 70);
			this->trackBar31->Maximum = 180;
			this->trackBar31->Minimum = -180;
			this->trackBar31->Name = L"trackBar31";
			this->trackBar31->Size = System::Drawing::Size(104, 45);
			this->trackBar31->TabIndex = 1;
			this->trackBar31->Scroll += gcnew System::EventHandler(this, &Form1::trackBar31_Scroll);
			// 
			// trackBar32
			// 
			this->trackBar32->Location = System::Drawing::Point(20, 3);
			this->trackBar32->Maximum = 180;
			this->trackBar32->Minimum = -180;
			this->trackBar32->Name = L"trackBar32";
			this->trackBar32->Size = System::Drawing::Size(104, 45);
			this->trackBar32->TabIndex = 0;
			this->trackBar32->Scroll += gcnew System::EventHandler(this, &Form1::trackBar32_Scroll);
			// 
			// panel6
			// 
			this->panel6->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->panel6->Controls->Add(this->textBox34);
			this->panel6->Controls->Add(this->textBox35);
			this->panel6->Controls->Add(this->textBox36);
			this->panel6->Controls->Add(this->textBox37);
			this->panel6->Controls->Add(this->textBox38);
			this->panel6->Controls->Add(this->label17);
			this->panel6->Controls->Add(this->trackBar33);
			this->panel6->Controls->Add(this->trackBar35);
			this->panel6->Controls->Add(this->trackBar36);
			this->panel6->Controls->Add(this->trackBar37);
			this->panel6->Controls->Add(this->trackBar38);
			this->panel6->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->panel6->Location = System::Drawing::Point(47, 488);
			this->panel6->Name = L"panel6";
			this->panel6->Size = System::Drawing::Size(372, 201);
			this->panel6->TabIndex = 22;
			// 
			// textBox34
			// 
			this->textBox34->Location = System::Drawing::Point(44, 161);
			this->textBox34->Name = L"textBox34";
			this->textBox34->Size = System::Drawing::Size(54, 29);
			this->textBox34->TabIndex = 16;
			this->textBox34->Text = L"0";
			this->textBox34->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox34->TextChanged += gcnew System::EventHandler(this, &Form1::textBox34_TextChanged);
			// 
			// textBox35
			// 
			this->textBox35->Location = System::Drawing::Point(185, 49);
			this->textBox35->Name = L"textBox35";
			this->textBox35->Size = System::Drawing::Size(54, 29);
			this->textBox35->TabIndex = 15;
			this->textBox35->Text = L"0";
			this->textBox35->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox35->TextChanged += gcnew System::EventHandler(this, &Form1::textBox35_TextChanged);
			// 
			// textBox36
			// 
			this->textBox36->Location = System::Drawing::Point(44, 101);
			this->textBox36->Name = L"textBox36";
			this->textBox36->Size = System::Drawing::Size(54, 29);
			this->textBox36->TabIndex = 14;
			this->textBox36->Text = L"0";
			this->textBox36->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox36->TextChanged += gcnew System::EventHandler(this, &Form1::textBox36_TextChanged);
			// 
			// textBox37
			// 
			this->textBox37->Location = System::Drawing::Point(44, 35);
			this->textBox37->Name = L"textBox37";
			this->textBox37->Size = System::Drawing::Size(54, 29);
			this->textBox37->TabIndex = 13;
			this->textBox37->Text = L"0";
			this->textBox37->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox37->TextChanged += gcnew System::EventHandler(this, &Form1::textBox37_TextChanged);
			// 
			// textBox38
			// 
			this->textBox38->Location = System::Drawing::Point(271, 49);
			this->textBox38->Name = L"textBox38";
			this->textBox38->Size = System::Drawing::Size(54, 29);
			this->textBox38->TabIndex = 12;
			this->textBox38->Text = L"0";
			this->textBox38->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox38->TextChanged += gcnew System::EventHandler(this, &Form1::textBox38_TextChanged);
			// 
			// label17
			// 
			this->label17->AutoSize = true;
			this->label17->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label17->Location = System::Drawing::Point(185, 153);
			this->label17->Name = L"label17";
			this->label17->Size = System::Drawing::Size(150, 25);
			this->label17->TabIndex = 5;
			this->label17->Text = L"UPPER BODY";
			// 
			// trackBar33
			// 
			this->trackBar33->Location = System::Drawing::Point(139, 3);
			this->trackBar33->Maximum = 180;
			this->trackBar33->Minimum = -180;
			this->trackBar33->Name = L"trackBar33";
			this->trackBar33->Orientation = System::Windows::Forms::Orientation::Vertical;
			this->trackBar33->Size = System::Drawing::Size(45, 129);
			this->trackBar33->TabIndex = 5;
			this->trackBar33->Scroll += gcnew System::EventHandler(this, &Form1::trackBar33_Scroll);
			// 
			// trackBar35
			// 
			this->trackBar35->Location = System::Drawing::Point(229, 15);
			this->trackBar35->Maximum = 180;
			this->trackBar35->Minimum = -180;
			this->trackBar35->Name = L"trackBar35";
			this->trackBar35->Size = System::Drawing::Size(138, 45);
			this->trackBar35->TabIndex = 3;
			this->trackBar35->Scroll += gcnew System::EventHandler(this, &Form1::trackBar35_Scroll);
			// 
			// trackBar36
			// 
			this->trackBar36->Location = System::Drawing::Point(3, 133);
			this->trackBar36->Maximum = 180;
			this->trackBar36->Minimum = -180;
			this->trackBar36->Name = L"trackBar36";
			this->trackBar36->Size = System::Drawing::Size(130, 45);
			this->trackBar36->TabIndex = 2;
			this->trackBar36->Scroll += gcnew System::EventHandler(this, &Form1::trackBar36_Scroll);
			// 
			// trackBar37
			// 
			this->trackBar37->Location = System::Drawing::Point(3, 70);
			this->trackBar37->Maximum = 180;
			this->trackBar37->Minimum = -180;
			this->trackBar37->Name = L"trackBar37";
			this->trackBar37->Size = System::Drawing::Size(130, 45);
			this->trackBar37->TabIndex = 1;
			this->trackBar37->Scroll += gcnew System::EventHandler(this, &Form1::trackBar37_Scroll);
			// 
			// trackBar38
			// 
			this->trackBar38->Location = System::Drawing::Point(3, 3);
			this->trackBar38->Maximum = 180;
			this->trackBar38->Minimum = -180;
			this->trackBar38->Name = L"trackBar38";
			this->trackBar38->Size = System::Drawing::Size(121, 45);
			this->trackBar38->TabIndex = 0;
			this->trackBar38->Scroll += gcnew System::EventHandler(this, &Form1::trackBar38_Scroll);
			// 
			// panel7
			// 
			this->panel7->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->panel7->Controls->Add(this->textBox55);
			this->panel7->Controls->Add(this->trackBar54);
			this->panel7->Controls->Add(this->textBox56);
			this->panel7->Controls->Add(this->trackBar55);
			this->panel7->Controls->Add(this->textBox57);
			this->panel7->Controls->Add(this->textBox58);
			this->panel7->Controls->Add(this->textBox59);
			this->panel7->Controls->Add(this->trackBar56);
			this->panel7->Controls->Add(this->trackBar57);
			this->panel7->Controls->Add(this->trackBar58);
			this->panel7->Controls->Add(this->textBox54);
			this->panel7->Controls->Add(this->trackBar53);
			this->panel7->Controls->Add(this->textBox50);
			this->panel7->Controls->Add(this->textBox51);
			this->panel7->Controls->Add(this->trackBar49);
			this->panel7->Controls->Add(this->trackBar50);
			this->panel7->Controls->Add(this->textBox52);
			this->panel7->Controls->Add(this->textBox53);
			this->panel7->Controls->Add(this->trackBar51);
			this->panel7->Controls->Add(this->trackBar52);
			this->panel7->Controls->Add(this->textBox47);
			this->panel7->Controls->Add(this->textBox48);
			this->panel7->Controls->Add(this->textBox49);
			this->panel7->Controls->Add(this->trackBar46);
			this->panel7->Controls->Add(this->trackBar47);
			this->panel7->Controls->Add(this->trackBar48);
			this->panel7->Controls->Add(this->textBox40);
			this->panel7->Controls->Add(this->textBox41);
			this->panel7->Controls->Add(this->textBox42);
			this->panel7->Controls->Add(this->textBox43);
			this->panel7->Controls->Add(this->textBox44);
			this->panel7->Controls->Add(this->textBox45);
			this->panel7->Controls->Add(this->textBox46);
			this->panel7->Controls->Add(this->label18);
			this->panel7->Controls->Add(this->trackBar39);
			this->panel7->Controls->Add(this->trackBar40);
			this->panel7->Controls->Add(this->trackBar41);
			this->panel7->Controls->Add(this->trackBar42);
			this->panel7->Controls->Add(this->trackBar43);
			this->panel7->Controls->Add(this->trackBar44);
			this->panel7->Controls->Add(this->trackBar45);
			this->panel7->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->panel7->Location = System::Drawing::Point(901, 67);
			this->panel7->Name = L"panel7";
			this->panel7->Size = System::Drawing::Size(430, 378);
			this->panel7->TabIndex = 27;
			// 
			// textBox55
			// 
			this->textBox55->Location = System::Drawing::Point(357, 292);
			this->textBox55->Name = L"textBox55";
			this->textBox55->Size = System::Drawing::Size(54, 29);
			this->textBox55->TabIndex = 43;
			this->textBox55->Text = L"0";
			this->textBox55->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox55->TextChanged += gcnew System::EventHandler(this, &Form1::textBox55_TextChanged);
			// 
			// trackBar54
			// 
			this->trackBar54->Location = System::Drawing::Point(333, 261);
			this->trackBar54->Maximum = 180;
			this->trackBar54->Minimum = -180;
			this->trackBar54->Name = L"trackBar54";
			this->trackBar54->Size = System::Drawing::Size(104, 45);
			this->trackBar54->TabIndex = 42;
			this->trackBar54->Scroll += gcnew System::EventHandler(this, &Form1::trackBar54_Scroll);
			// 
			// textBox56
			// 
			this->textBox56->Location = System::Drawing::Point(357, 229);
			this->textBox56->Name = L"textBox56";
			this->textBox56->Size = System::Drawing::Size(54, 29);
			this->textBox56->TabIndex = 41;
			this->textBox56->Text = L"0";
			this->textBox56->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox56->TextChanged += gcnew System::EventHandler(this, &Form1::textBox56_TextChanged);
			// 
			// trackBar55
			// 
			this->trackBar55->Location = System::Drawing::Point(333, 200);
			this->trackBar55->Maximum = 180;
			this->trackBar55->Minimum = -180;
			this->trackBar55->Name = L"trackBar55";
			this->trackBar55->Size = System::Drawing::Size(104, 45);
			this->trackBar55->TabIndex = 40;
			this->trackBar55->Scroll += gcnew System::EventHandler(this, &Form1::trackBar55_Scroll);
			// 
			// textBox57
			// 
			this->textBox57->Location = System::Drawing::Point(357, 161);
			this->textBox57->Name = L"textBox57";
			this->textBox57->Size = System::Drawing::Size(54, 29);
			this->textBox57->TabIndex = 39;
			this->textBox57->Text = L"0";
			this->textBox57->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox57->TextChanged += gcnew System::EventHandler(this, &Form1::textBox57_TextChanged);
			// 
			// textBox58
			// 
			this->textBox58->Location = System::Drawing::Point(357, 101);
			this->textBox58->Name = L"textBox58";
			this->textBox58->Size = System::Drawing::Size(54, 29);
			this->textBox58->TabIndex = 38;
			this->textBox58->Text = L"0";
			this->textBox58->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox58->TextChanged += gcnew System::EventHandler(this, &Form1::textBox58_TextChanged);
			// 
			// textBox59
			// 
			this->textBox59->Location = System::Drawing::Point(362, 35);
			this->textBox59->Name = L"textBox59";
			this->textBox59->Size = System::Drawing::Size(49, 29);
			this->textBox59->TabIndex = 37;
			this->textBox59->Text = L"0";
			this->textBox59->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox59->TextChanged += gcnew System::EventHandler(this, &Form1::textBox59_TextChanged);
			// 
			// trackBar56
			// 
			this->trackBar56->Location = System::Drawing::Point(333, 133);
			this->trackBar56->Maximum = 180;
			this->trackBar56->Minimum = -180;
			this->trackBar56->Name = L"trackBar56";
			this->trackBar56->Size = System::Drawing::Size(104, 45);
			this->trackBar56->TabIndex = 36;
			this->trackBar56->Scroll += gcnew System::EventHandler(this, &Form1::trackBar56_Scroll);
			// 
			// trackBar57
			// 
			this->trackBar57->Location = System::Drawing::Point(333, 70);
			this->trackBar57->Maximum = 180;
			this->trackBar57->Minimum = -180;
			this->trackBar57->Name = L"trackBar57";
			this->trackBar57->Size = System::Drawing::Size(104, 45);
			this->trackBar57->TabIndex = 35;
			this->trackBar57->Scroll += gcnew System::EventHandler(this, &Form1::trackBar57_Scroll);
			// 
			// trackBar58
			// 
			this->trackBar58->Location = System::Drawing::Point(333, 3);
			this->trackBar58->Maximum = 180;
			this->trackBar58->Minimum = -180;
			this->trackBar58->Name = L"trackBar58";
			this->trackBar58->Size = System::Drawing::Size(104, 45);
			this->trackBar58->TabIndex = 34;
			this->trackBar58->Scroll += gcnew System::EventHandler(this, &Form1::trackBar58_Scroll);
			// 
			// textBox54
			// 
			this->textBox54->Location = System::Drawing::Point(264, 292);
			this->textBox54->Name = L"textBox54";
			this->textBox54->Size = System::Drawing::Size(54, 29);
			this->textBox54->TabIndex = 33;
			this->textBox54->Text = L"0";
			this->textBox54->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox54->TextChanged += gcnew System::EventHandler(this, &Form1::textBox54_TextChanged);
			// 
			// trackBar53
			// 
			this->trackBar53->Location = System::Drawing::Point(240, 261);
			this->trackBar53->Maximum = 180;
			this->trackBar53->Minimum = -180;
			this->trackBar53->Name = L"trackBar53";
			this->trackBar53->Size = System::Drawing::Size(104, 45);
			this->trackBar53->TabIndex = 32;
			this->trackBar53->Scroll += gcnew System::EventHandler(this, &Form1::trackBar53_Scroll);
			// 
			// textBox50
			// 
			this->textBox50->Location = System::Drawing::Point(44, 229);
			this->textBox50->Name = L"textBox50";
			this->textBox50->Size = System::Drawing::Size(54, 29);
			this->textBox50->TabIndex = 31;
			this->textBox50->Text = L"0";
			this->textBox50->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox50->TextChanged += gcnew System::EventHandler(this, &Form1::textBox50_TextChanged);
			// 
			// textBox51
			// 
			this->textBox51->Location = System::Drawing::Point(154, 292);
			this->textBox51->Name = L"textBox51";
			this->textBox51->Size = System::Drawing::Size(54, 29);
			this->textBox51->TabIndex = 30;
			this->textBox51->Text = L"0";
			this->textBox51->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox51->TextChanged += gcnew System::EventHandler(this, &Form1::textBox51_TextChanged);
			// 
			// trackBar49
			// 
			this->trackBar49->Location = System::Drawing::Point(20, 200);
			this->trackBar49->Maximum = 180;
			this->trackBar49->Minimum = -180;
			this->trackBar49->Name = L"trackBar49";
			this->trackBar49->Size = System::Drawing::Size(104, 45);
			this->trackBar49->TabIndex = 29;
			this->trackBar49->Scroll += gcnew System::EventHandler(this, &Form1::trackBar49_Scroll);
			// 
			// trackBar50
			// 
			this->trackBar50->Location = System::Drawing::Point(130, 261);
			this->trackBar50->Maximum = 180;
			this->trackBar50->Minimum = -180;
			this->trackBar50->Name = L"trackBar50";
			this->trackBar50->Size = System::Drawing::Size(104, 45);
			this->trackBar50->TabIndex = 28;
			this->trackBar50->Scroll += gcnew System::EventHandler(this, &Form1::trackBar50_Scroll);
			// 
			// textBox52
			// 
			this->textBox52->Location = System::Drawing::Point(264, 229);
			this->textBox52->Name = L"textBox52";
			this->textBox52->Size = System::Drawing::Size(54, 29);
			this->textBox52->TabIndex = 27;
			this->textBox52->Text = L"0";
			this->textBox52->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox52->TextChanged += gcnew System::EventHandler(this, &Form1::textBox52_TextChanged);
			// 
			// textBox53
			// 
			this->textBox53->Location = System::Drawing::Point(44, 292);
			this->textBox53->Name = L"textBox53";
			this->textBox53->Size = System::Drawing::Size(54, 29);
			this->textBox53->TabIndex = 26;
			this->textBox53->Text = L"0";
			this->textBox53->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox53->TextChanged += gcnew System::EventHandler(this, &Form1::textBox53_TextChanged);
			// 
			// trackBar51
			// 
			this->trackBar51->Location = System::Drawing::Point(240, 200);
			this->trackBar51->Maximum = 180;
			this->trackBar51->Minimum = -180;
			this->trackBar51->Name = L"trackBar51";
			this->trackBar51->Size = System::Drawing::Size(104, 45);
			this->trackBar51->TabIndex = 25;
			this->trackBar51->Scroll += gcnew System::EventHandler(this, &Form1::trackBar51_Scroll);
			// 
			// trackBar52
			// 
			this->trackBar52->Location = System::Drawing::Point(20, 264);
			this->trackBar52->Maximum = 180;
			this->trackBar52->Minimum = -180;
			this->trackBar52->Name = L"trackBar52";
			this->trackBar52->Size = System::Drawing::Size(104, 45);
			this->trackBar52->TabIndex = 24;
			this->trackBar52->Scroll += gcnew System::EventHandler(this, &Form1::trackBar52_Scroll);
			// 
			// textBox47
			// 
			this->textBox47->Location = System::Drawing::Point(154, 228);
			this->textBox47->Name = L"textBox47";
			this->textBox47->Size = System::Drawing::Size(54, 29);
			this->textBox47->TabIndex = 23;
			this->textBox47->Text = L"0";
			this->textBox47->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox47->TextChanged += gcnew System::EventHandler(this, &Form1::textBox47_TextChanged);
			// 
			// textBox48
			// 
			this->textBox48->Location = System::Drawing::Point(154, 161);
			this->textBox48->Name = L"textBox48";
			this->textBox48->Size = System::Drawing::Size(54, 29);
			this->textBox48->TabIndex = 22;
			this->textBox48->Text = L"0";
			this->textBox48->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox48->TextChanged += gcnew System::EventHandler(this, &Form1::textBox48_TextChanged);
			// 
			// textBox49
			// 
			this->textBox49->Location = System::Drawing::Point(159, 102);
			this->textBox49->Name = L"textBox49";
			this->textBox49->Size = System::Drawing::Size(49, 29);
			this->textBox49->TabIndex = 21;
			this->textBox49->Text = L"0";
			this->textBox49->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox49->TextChanged += gcnew System::EventHandler(this, &Form1::textBox49_TextChanged);
			// 
			// trackBar46
			// 
			this->trackBar46->Location = System::Drawing::Point(130, 200);
			this->trackBar46->Maximum = 180;
			this->trackBar46->Minimum = -180;
			this->trackBar46->Name = L"trackBar46";
			this->trackBar46->Size = System::Drawing::Size(104, 45);
			this->trackBar46->TabIndex = 20;
			this->trackBar46->Scroll += gcnew System::EventHandler(this, &Form1::trackBar46_Scroll);
			// 
			// trackBar47
			// 
			this->trackBar47->Location = System::Drawing::Point(130, 133);
			this->trackBar47->Maximum = 180;
			this->trackBar47->Minimum = -180;
			this->trackBar47->Name = L"trackBar47";
			this->trackBar47->Size = System::Drawing::Size(104, 45);
			this->trackBar47->TabIndex = 19;
			this->trackBar47->Scroll += gcnew System::EventHandler(this, &Form1::trackBar47_Scroll);
			// 
			// trackBar48
			// 
			this->trackBar48->Location = System::Drawing::Point(130, 70);
			this->trackBar48->Maximum = 180;
			this->trackBar48->Minimum = -180;
			this->trackBar48->Name = L"trackBar48";
			this->trackBar48->Size = System::Drawing::Size(104, 45);
			this->trackBar48->TabIndex = 18;
			this->trackBar48->Scroll += gcnew System::EventHandler(this, &Form1::trackBar48_Scroll);
			// 
			// textBox40
			// 
			this->textBox40->Location = System::Drawing::Point(264, 161);
			this->textBox40->Name = L"textBox40";
			this->textBox40->Size = System::Drawing::Size(54, 29);
			this->textBox40->TabIndex = 17;
			this->textBox40->Text = L"0";
			this->textBox40->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox40->TextChanged += gcnew System::EventHandler(this, &Form1::textBox40_TextChanged);
			// 
			// textBox41
			// 
			this->textBox41->Location = System::Drawing::Point(44, 161);
			this->textBox41->Name = L"textBox41";
			this->textBox41->Size = System::Drawing::Size(54, 29);
			this->textBox41->TabIndex = 16;
			this->textBox41->Text = L"0";
			this->textBox41->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox41->TextChanged += gcnew System::EventHandler(this, &Form1::textBox41_TextChanged);
			// 
			// textBox42
			// 
			this->textBox42->Location = System::Drawing::Point(264, 101);
			this->textBox42->Name = L"textBox42";
			this->textBox42->Size = System::Drawing::Size(54, 29);
			this->textBox42->TabIndex = 15;
			this->textBox42->Text = L"0";
			this->textBox42->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox42->TextChanged += gcnew System::EventHandler(this, &Form1::textBox42_TextChanged);
			// 
			// textBox43
			// 
			this->textBox43->Location = System::Drawing::Point(44, 101);
			this->textBox43->Name = L"textBox43";
			this->textBox43->Size = System::Drawing::Size(54, 29);
			this->textBox43->TabIndex = 14;
			this->textBox43->Text = L"0";
			this->textBox43->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox43->TextChanged += gcnew System::EventHandler(this, &Form1::textBox43_TextChanged);
			// 
			// textBox44
			// 
			this->textBox44->Location = System::Drawing::Point(44, 35);
			this->textBox44->Name = L"textBox44";
			this->textBox44->Size = System::Drawing::Size(54, 29);
			this->textBox44->TabIndex = 13;
			this->textBox44->Text = L"0";
			this->textBox44->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox44->TextChanged += gcnew System::EventHandler(this, &Form1::textBox44_TextChanged);
			// 
			// textBox45
			// 
			this->textBox45->Location = System::Drawing::Point(157, 35);
			this->textBox45->Name = L"textBox45";
			this->textBox45->Size = System::Drawing::Size(54, 29);
			this->textBox45->TabIndex = 12;
			this->textBox45->Text = L"0";
			this->textBox45->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox45->TextChanged += gcnew System::EventHandler(this, &Form1::textBox45_TextChanged);
			// 
			// textBox46
			// 
			this->textBox46->Location = System::Drawing::Point(269, 35);
			this->textBox46->Name = L"textBox46";
			this->textBox46->Size = System::Drawing::Size(49, 29);
			this->textBox46->TabIndex = 11;
			this->textBox46->Text = L"0";
			this->textBox46->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox46->TextChanged += gcnew System::EventHandler(this, &Form1::textBox46_TextChanged);
			// 
			// label18
			// 
			this->label18->AutoSize = true;
			this->label18->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label18->Location = System::Drawing::Point(167, 343);
			this->label18->Name = L"label18";
			this->label18->Size = System::Drawing::Size(129, 25);
			this->label18->TabIndex = 5;
			this->label18->Text = L"LEFT HAND";
			// 
			// trackBar39
			// 
			this->trackBar39->Location = System::Drawing::Point(240, 133);
			this->trackBar39->Maximum = 180;
			this->trackBar39->Minimum = -180;
			this->trackBar39->Name = L"trackBar39";
			this->trackBar39->Size = System::Drawing::Size(104, 45);
			this->trackBar39->TabIndex = 6;
			this->trackBar39->Scroll += gcnew System::EventHandler(this, &Form1::trackBar39_Scroll);
			// 
			// trackBar40
			// 
			this->trackBar40->Location = System::Drawing::Point(240, 70);
			this->trackBar40->Maximum = 180;
			this->trackBar40->Minimum = -180;
			this->trackBar40->Name = L"trackBar40";
			this->trackBar40->Size = System::Drawing::Size(104, 45);
			this->trackBar40->TabIndex = 5;
			this->trackBar40->Scroll += gcnew System::EventHandler(this, &Form1::trackBar40_Scroll);
			// 
			// trackBar41
			// 
			this->trackBar41->Location = System::Drawing::Point(240, 3);
			this->trackBar41->Maximum = 180;
			this->trackBar41->Minimum = -180;
			this->trackBar41->Name = L"trackBar41";
			this->trackBar41->Size = System::Drawing::Size(104, 45);
			this->trackBar41->TabIndex = 4;
			this->trackBar41->Scroll += gcnew System::EventHandler(this, &Form1::trackBar41_Scroll);
			// 
			// trackBar42
			// 
			this->trackBar42->Location = System::Drawing::Point(130, 3);
			this->trackBar42->Maximum = 180;
			this->trackBar42->Minimum = -180;
			this->trackBar42->Name = L"trackBar42";
			this->trackBar42->Size = System::Drawing::Size(104, 45);
			this->trackBar42->TabIndex = 3;
			this->trackBar42->Scroll += gcnew System::EventHandler(this, &Form1::trackBar42_Scroll);
			// 
			// trackBar43
			// 
			this->trackBar43->Location = System::Drawing::Point(20, 133);
			this->trackBar43->Maximum = 180;
			this->trackBar43->Minimum = -180;
			this->trackBar43->Name = L"trackBar43";
			this->trackBar43->Size = System::Drawing::Size(104, 45);
			this->trackBar43->TabIndex = 2;
			this->trackBar43->Scroll += gcnew System::EventHandler(this, &Form1::trackBar43_Scroll);
			// 
			// trackBar44
			// 
			this->trackBar44->Location = System::Drawing::Point(20, 70);
			this->trackBar44->Maximum = 180;
			this->trackBar44->Minimum = -180;
			this->trackBar44->Name = L"trackBar44";
			this->trackBar44->Size = System::Drawing::Size(104, 45);
			this->trackBar44->TabIndex = 1;
			this->trackBar44->Scroll += gcnew System::EventHandler(this, &Form1::trackBar44_Scroll);
			// 
			// trackBar45
			// 
			this->trackBar45->Location = System::Drawing::Point(20, 3);
			this->trackBar45->Maximum = 180;
			this->trackBar45->Minimum = -180;
			this->trackBar45->Name = L"trackBar45";
			this->trackBar45->Size = System::Drawing::Size(104, 45);
			this->trackBar45->TabIndex = 0;
			this->trackBar45->Scroll += gcnew System::EventHandler(this, &Form1::trackBar45_Scroll);
			// 
			// panel8
			// 
			this->panel8->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->panel8->Controls->Add(this->textBox60);
			this->panel8->Controls->Add(this->trackBar59);
			this->panel8->Controls->Add(this->textBox61);
			this->panel8->Controls->Add(this->trackBar60);
			this->panel8->Controls->Add(this->textBox62);
			this->panel8->Controls->Add(this->textBox63);
			this->panel8->Controls->Add(this->textBox64);
			this->panel8->Controls->Add(this->trackBar61);
			this->panel8->Controls->Add(this->trackBar62);
			this->panel8->Controls->Add(this->trackBar63);
			this->panel8->Controls->Add(this->textBox65);
			this->panel8->Controls->Add(this->trackBar64);
			this->panel8->Controls->Add(this->textBox66);
			this->panel8->Controls->Add(this->textBox67);
			this->panel8->Controls->Add(this->trackBar65);
			this->panel8->Controls->Add(this->trackBar66);
			this->panel8->Controls->Add(this->textBox68);
			this->panel8->Controls->Add(this->textBox69);
			this->panel8->Controls->Add(this->trackBar67);
			this->panel8->Controls->Add(this->trackBar68);
			this->panel8->Controls->Add(this->textBox70);
			this->panel8->Controls->Add(this->textBox71);
			this->panel8->Controls->Add(this->textBox72);
			this->panel8->Controls->Add(this->trackBar69);
			this->panel8->Controls->Add(this->trackBar70);
			this->panel8->Controls->Add(this->trackBar71);
			this->panel8->Controls->Add(this->textBox73);
			this->panel8->Controls->Add(this->textBox74);
			this->panel8->Controls->Add(this->textBox75);
			this->panel8->Controls->Add(this->textBox76);
			this->panel8->Controls->Add(this->textBox77);
			this->panel8->Controls->Add(this->textBox78);
			this->panel8->Controls->Add(this->textBox79);
			this->panel8->Controls->Add(this->label19);
			this->panel8->Controls->Add(this->trackBar72);
			this->panel8->Controls->Add(this->trackBar73);
			this->panel8->Controls->Add(this->trackBar74);
			this->panel8->Controls->Add(this->trackBar75);
			this->panel8->Controls->Add(this->trackBar76);
			this->panel8->Controls->Add(this->trackBar77);
			this->panel8->Controls->Add(this->trackBar78);
			this->panel8->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->panel8->Location = System::Drawing::Point(901, 451);
			this->panel8->Name = L"panel8";
			this->panel8->Size = System::Drawing::Size(430, 378);
			this->panel8->TabIndex = 28;
			// 
			// textBox60
			// 
			this->textBox60->Location = System::Drawing::Point(357, 292);
			this->textBox60->Name = L"textBox60";
			this->textBox60->Size = System::Drawing::Size(54, 29);
			this->textBox60->TabIndex = 43;
			this->textBox60->Text = L"0";
			this->textBox60->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox60->TextChanged += gcnew System::EventHandler(this, &Form1::textBox60_TextChanged);
			// 
			// trackBar59
			// 
			this->trackBar59->Location = System::Drawing::Point(333, 261);
			this->trackBar59->Maximum = 180;
			this->trackBar59->Minimum = -180;
			this->trackBar59->Name = L"trackBar59";
			this->trackBar59->Size = System::Drawing::Size(104, 45);
			this->trackBar59->TabIndex = 42;
			this->trackBar59->Scroll += gcnew System::EventHandler(this, &Form1::trackBar59_Scroll);
			// 
			// textBox61
			// 
			this->textBox61->Location = System::Drawing::Point(357, 229);
			this->textBox61->Name = L"textBox61";
			this->textBox61->Size = System::Drawing::Size(54, 29);
			this->textBox61->TabIndex = 41;
			this->textBox61->Text = L"0";
			this->textBox61->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox61->TextChanged += gcnew System::EventHandler(this, &Form1::textBox61_TextChanged);
			// 
			// trackBar60
			// 
			this->trackBar60->Location = System::Drawing::Point(333, 200);
			this->trackBar60->Maximum = 180;
			this->trackBar60->Minimum = -180;
			this->trackBar60->Name = L"trackBar60";
			this->trackBar60->Size = System::Drawing::Size(104, 45);
			this->trackBar60->TabIndex = 40;
			this->trackBar60->Scroll += gcnew System::EventHandler(this, &Form1::trackBar60_Scroll);
			// 
			// textBox62
			// 
			this->textBox62->Location = System::Drawing::Point(357, 161);
			this->textBox62->Name = L"textBox62";
			this->textBox62->Size = System::Drawing::Size(54, 29);
			this->textBox62->TabIndex = 39;
			this->textBox62->Text = L"0";
			this->textBox62->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox62->TextChanged += gcnew System::EventHandler(this, &Form1::textBox62_TextChanged);
			// 
			// textBox63
			// 
			this->textBox63->Location = System::Drawing::Point(357, 101);
			this->textBox63->Name = L"textBox63";
			this->textBox63->Size = System::Drawing::Size(54, 29);
			this->textBox63->TabIndex = 38;
			this->textBox63->Text = L"0";
			this->textBox63->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox63->TextChanged += gcnew System::EventHandler(this, &Form1::textBox63_TextChanged);
			// 
			// textBox64
			// 
			this->textBox64->Location = System::Drawing::Point(362, 35);
			this->textBox64->Name = L"textBox64";
			this->textBox64->Size = System::Drawing::Size(49, 29);
			this->textBox64->TabIndex = 37;
			this->textBox64->Text = L"0";
			this->textBox64->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox64->TextChanged += gcnew System::EventHandler(this, &Form1::textBox64_TextChanged);
			// 
			// trackBar61
			// 
			this->trackBar61->Location = System::Drawing::Point(333, 133);
			this->trackBar61->Maximum = 180;
			this->trackBar61->Minimum = -180;
			this->trackBar61->Name = L"trackBar61";
			this->trackBar61->Size = System::Drawing::Size(104, 45);
			this->trackBar61->TabIndex = 36;
			this->trackBar61->Scroll += gcnew System::EventHandler(this, &Form1::trackBar61_Scroll);
			// 
			// trackBar62
			// 
			this->trackBar62->Location = System::Drawing::Point(333, 70);
			this->trackBar62->Maximum = 180;
			this->trackBar62->Minimum = -180;
			this->trackBar62->Name = L"trackBar62";
			this->trackBar62->Size = System::Drawing::Size(104, 45);
			this->trackBar62->TabIndex = 35;
			this->trackBar62->Scroll += gcnew System::EventHandler(this, &Form1::trackBar62_Scroll);
			// 
			// trackBar63
			// 
			this->trackBar63->Location = System::Drawing::Point(333, 3);
			this->trackBar63->Maximum = 180;
			this->trackBar63->Minimum = -180;
			this->trackBar63->Name = L"trackBar63";
			this->trackBar63->Size = System::Drawing::Size(104, 45);
			this->trackBar63->TabIndex = 34;
			this->trackBar63->Scroll += gcnew System::EventHandler(this, &Form1::trackBar63_Scroll);
			// 
			// textBox65
			// 
			this->textBox65->Location = System::Drawing::Point(264, 292);
			this->textBox65->Name = L"textBox65";
			this->textBox65->Size = System::Drawing::Size(54, 29);
			this->textBox65->TabIndex = 33;
			this->textBox65->Text = L"0";
			this->textBox65->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox65->TextChanged += gcnew System::EventHandler(this, &Form1::textBox65_TextChanged);
			// 
			// trackBar64
			// 
			this->trackBar64->Location = System::Drawing::Point(240, 261);
			this->trackBar64->Maximum = 180;
			this->trackBar64->Minimum = -180;
			this->trackBar64->Name = L"trackBar64";
			this->trackBar64->Size = System::Drawing::Size(104, 45);
			this->trackBar64->TabIndex = 32;
			this->trackBar64->Scroll += gcnew System::EventHandler(this, &Form1::trackBar64_Scroll);
			// 
			// textBox66
			// 
			this->textBox66->Location = System::Drawing::Point(44, 229);
			this->textBox66->Name = L"textBox66";
			this->textBox66->Size = System::Drawing::Size(54, 29);
			this->textBox66->TabIndex = 31;
			this->textBox66->Text = L"0";
			this->textBox66->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox66->TextChanged += gcnew System::EventHandler(this, &Form1::textBox66_TextChanged);
			// 
			// textBox67
			// 
			this->textBox67->Location = System::Drawing::Point(154, 292);
			this->textBox67->Name = L"textBox67";
			this->textBox67->Size = System::Drawing::Size(54, 29);
			this->textBox67->TabIndex = 30;
			this->textBox67->Text = L"0";
			this->textBox67->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox67->TextChanged += gcnew System::EventHandler(this, &Form1::textBox67_TextChanged);
			// 
			// trackBar65
			// 
			this->trackBar65->Location = System::Drawing::Point(20, 200);
			this->trackBar65->Maximum = 180;
			this->trackBar65->Minimum = -180;
			this->trackBar65->Name = L"trackBar65";
			this->trackBar65->Size = System::Drawing::Size(104, 45);
			this->trackBar65->TabIndex = 29;
			this->trackBar65->Scroll += gcnew System::EventHandler(this, &Form1::trackBar65_Scroll);
			// 
			// trackBar66
			// 
			this->trackBar66->Location = System::Drawing::Point(130, 261);
			this->trackBar66->Maximum = 180;
			this->trackBar66->Minimum = -180;
			this->trackBar66->Name = L"trackBar66";
			this->trackBar66->Size = System::Drawing::Size(104, 45);
			this->trackBar66->TabIndex = 28;
			this->trackBar66->Scroll += gcnew System::EventHandler(this, &Form1::trackBar66_Scroll);
			// 
			// textBox68
			// 
			this->textBox68->Location = System::Drawing::Point(264, 229);
			this->textBox68->Name = L"textBox68";
			this->textBox68->Size = System::Drawing::Size(54, 29);
			this->textBox68->TabIndex = 27;
			this->textBox68->Text = L"0";
			this->textBox68->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox68->TextChanged += gcnew System::EventHandler(this, &Form1::textBox68_TextChanged);
			// 
			// textBox69
			// 
			this->textBox69->Location = System::Drawing::Point(44, 292);
			this->textBox69->Name = L"textBox69";
			this->textBox69->Size = System::Drawing::Size(54, 29);
			this->textBox69->TabIndex = 26;
			this->textBox69->Text = L"0";
			this->textBox69->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox69->TextChanged += gcnew System::EventHandler(this, &Form1::textBox69_TextChanged);
			// 
			// trackBar67
			// 
			this->trackBar67->Location = System::Drawing::Point(240, 200);
			this->trackBar67->Maximum = 180;
			this->trackBar67->Minimum = -180;
			this->trackBar67->Name = L"trackBar67";
			this->trackBar67->Size = System::Drawing::Size(104, 45);
			this->trackBar67->TabIndex = 25;
			this->trackBar67->Scroll += gcnew System::EventHandler(this, &Form1::trackBar67_Scroll);
			// 
			// trackBar68
			// 
			this->trackBar68->Location = System::Drawing::Point(20, 257);
			this->trackBar68->Maximum = 180;
			this->trackBar68->Minimum = -180;
			this->trackBar68->Name = L"trackBar68";
			this->trackBar68->Size = System::Drawing::Size(104, 45);
			this->trackBar68->TabIndex = 24;
			this->trackBar68->Scroll += gcnew System::EventHandler(this, &Form1::trackBar68_Scroll);
			// 
			// textBox70
			// 
			this->textBox70->Location = System::Drawing::Point(154, 228);
			this->textBox70->Name = L"textBox70";
			this->textBox70->Size = System::Drawing::Size(54, 29);
			this->textBox70->TabIndex = 23;
			this->textBox70->Text = L"0";
			this->textBox70->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox70->TextChanged += gcnew System::EventHandler(this, &Form1::textBox70_TextChanged);
			// 
			// textBox71
			// 
			this->textBox71->Location = System::Drawing::Point(159, 165);
			this->textBox71->Name = L"textBox71";
			this->textBox71->Size = System::Drawing::Size(54, 29);
			this->textBox71->TabIndex = 22;
			this->textBox71->Text = L"0";
			this->textBox71->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox71->TextChanged += gcnew System::EventHandler(this, &Form1::textBox71_TextChanged);
			// 
			// textBox72
			// 
			this->textBox72->Location = System::Drawing::Point(159, 102);
			this->textBox72->Name = L"textBox72";
			this->textBox72->Size = System::Drawing::Size(49, 29);
			this->textBox72->TabIndex = 21;
			this->textBox72->Text = L"0";
			this->textBox72->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox72->TextChanged += gcnew System::EventHandler(this, &Form1::textBox72_TextChanged);
			// 
			// trackBar69
			// 
			this->trackBar69->Location = System::Drawing::Point(130, 200);
			this->trackBar69->Maximum = 180;
			this->trackBar69->Minimum = -180;
			this->trackBar69->Name = L"trackBar69";
			this->trackBar69->Size = System::Drawing::Size(104, 45);
			this->trackBar69->TabIndex = 20;
			this->trackBar69->Scroll += gcnew System::EventHandler(this, &Form1::trackBar69_Scroll);
			// 
			// trackBar70
			// 
			this->trackBar70->Location = System::Drawing::Point(130, 137);
			this->trackBar70->Maximum = 180;
			this->trackBar70->Minimum = -180;
			this->trackBar70->Name = L"trackBar70";
			this->trackBar70->Size = System::Drawing::Size(104, 45);
			this->trackBar70->TabIndex = 19;
			this->trackBar70->Scroll += gcnew System::EventHandler(this, &Form1::trackBar70_Scroll);
			// 
			// trackBar71
			// 
			this->trackBar71->Location = System::Drawing::Point(130, 70);
			this->trackBar71->Maximum = 180;
			this->trackBar71->Minimum = -180;
			this->trackBar71->Name = L"trackBar71";
			this->trackBar71->Size = System::Drawing::Size(104, 45);
			this->trackBar71->TabIndex = 18;
			this->trackBar71->Scroll += gcnew System::EventHandler(this, &Form1::trackBar71_Scroll);
			// 
			// textBox73
			// 
			this->textBox73->Location = System::Drawing::Point(264, 161);
			this->textBox73->Name = L"textBox73";
			this->textBox73->Size = System::Drawing::Size(54, 29);
			this->textBox73->TabIndex = 17;
			this->textBox73->Text = L"0";
			this->textBox73->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox73->TextChanged += gcnew System::EventHandler(this, &Form1::textBox73_TextChanged);
			// 
			// textBox74
			// 
			this->textBox74->Location = System::Drawing::Point(44, 161);
			this->textBox74->Name = L"textBox74";
			this->textBox74->Size = System::Drawing::Size(54, 29);
			this->textBox74->TabIndex = 16;
			this->textBox74->Text = L"0";
			this->textBox74->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox74->TextChanged += gcnew System::EventHandler(this, &Form1::textBox74_TextChanged);
			// 
			// textBox75
			// 
			this->textBox75->Location = System::Drawing::Point(264, 101);
			this->textBox75->Name = L"textBox75";
			this->textBox75->Size = System::Drawing::Size(54, 29);
			this->textBox75->TabIndex = 15;
			this->textBox75->Text = L"0";
			this->textBox75->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox75->TextChanged += gcnew System::EventHandler(this, &Form1::textBox75_TextChanged);
			// 
			// textBox76
			// 
			this->textBox76->Location = System::Drawing::Point(44, 101);
			this->textBox76->Name = L"textBox76";
			this->textBox76->Size = System::Drawing::Size(54, 29);
			this->textBox76->TabIndex = 14;
			this->textBox76->Text = L"0";
			this->textBox76->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox76->TextChanged += gcnew System::EventHandler(this, &Form1::textBox76_TextChanged);
			// 
			// textBox77
			// 
			this->textBox77->Location = System::Drawing::Point(44, 35);
			this->textBox77->Name = L"textBox77";
			this->textBox77->Size = System::Drawing::Size(54, 29);
			this->textBox77->TabIndex = 13;
			this->textBox77->Text = L"0";
			this->textBox77->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox77->TextChanged += gcnew System::EventHandler(this, &Form1::textBox77_TextChanged);
			// 
			// textBox78
			// 
			this->textBox78->Location = System::Drawing::Point(157, 35);
			this->textBox78->Name = L"textBox78";
			this->textBox78->Size = System::Drawing::Size(54, 29);
			this->textBox78->TabIndex = 12;
			this->textBox78->Text = L"0";
			this->textBox78->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox78->TextChanged += gcnew System::EventHandler(this, &Form1::textBox78_TextChanged);
			// 
			// textBox79
			// 
			this->textBox79->Location = System::Drawing::Point(269, 35);
			this->textBox79->Name = L"textBox79";
			this->textBox79->Size = System::Drawing::Size(49, 29);
			this->textBox79->TabIndex = 11;
			this->textBox79->Text = L"0";
			this->textBox79->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			this->textBox79->TextChanged += gcnew System::EventHandler(this, &Form1::textBox79_TextChanged);
			// 
			// label19
			// 
			this->label19->AutoSize = true;
			this->label19->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label19->Location = System::Drawing::Point(167, 343);
			this->label19->Name = L"label19";
			this->label19->Size = System::Drawing::Size(141, 25);
			this->label19->TabIndex = 5;
			this->label19->Text = L"RIGHT HAND";
			// 
			// trackBar72
			// 
			this->trackBar72->Location = System::Drawing::Point(240, 133);
			this->trackBar72->Maximum = 180;
			this->trackBar72->Minimum = -180;
			this->trackBar72->Name = L"trackBar72";
			this->trackBar72->Size = System::Drawing::Size(104, 45);
			this->trackBar72->TabIndex = 6;
			this->trackBar72->Scroll += gcnew System::EventHandler(this, &Form1::trackBar72_Scroll);
			// 
			// trackBar73
			// 
			this->trackBar73->Location = System::Drawing::Point(240, 70);
			this->trackBar73->Maximum = 180;
			this->trackBar73->Minimum = -180;
			this->trackBar73->Name = L"trackBar73";
			this->trackBar73->Size = System::Drawing::Size(104, 45);
			this->trackBar73->TabIndex = 5;
			this->trackBar73->Scroll += gcnew System::EventHandler(this, &Form1::trackBar73_Scroll);
			// 
			// trackBar74
			// 
			this->trackBar74->Location = System::Drawing::Point(240, 3);
			this->trackBar74->Maximum = 180;
			this->trackBar74->Minimum = -180;
			this->trackBar74->Name = L"trackBar74";
			this->trackBar74->Size = System::Drawing::Size(104, 45);
			this->trackBar74->TabIndex = 4;
			this->trackBar74->Scroll += gcnew System::EventHandler(this, &Form1::trackBar74_Scroll);
			// 
			// trackBar75
			// 
			this->trackBar75->Location = System::Drawing::Point(130, 3);
			this->trackBar75->Maximum = 180;
			this->trackBar75->Minimum = -180;
			this->trackBar75->Name = L"trackBar75";
			this->trackBar75->Size = System::Drawing::Size(104, 45);
			this->trackBar75->TabIndex = 3;
			this->trackBar75->Scroll += gcnew System::EventHandler(this, &Form1::trackBar75_Scroll);
			// 
			// trackBar76
			// 
			this->trackBar76->Location = System::Drawing::Point(20, 133);
			this->trackBar76->Maximum = 180;
			this->trackBar76->Minimum = -180;
			this->trackBar76->Name = L"trackBar76";
			this->trackBar76->Size = System::Drawing::Size(104, 45);
			this->trackBar76->TabIndex = 2;
			this->trackBar76->Scroll += gcnew System::EventHandler(this, &Form1::trackBar76_Scroll);
			// 
			// trackBar77
			// 
			this->trackBar77->Location = System::Drawing::Point(20, 70);
			this->trackBar77->Maximum = 180;
			this->trackBar77->Minimum = -180;
			this->trackBar77->Name = L"trackBar77";
			this->trackBar77->Size = System::Drawing::Size(104, 45);
			this->trackBar77->TabIndex = 1;
			this->trackBar77->Scroll += gcnew System::EventHandler(this, &Form1::trackBar77_Scroll);
			// 
			// trackBar78
			// 
			this->trackBar78->Location = System::Drawing::Point(20, 3);
			this->trackBar78->Maximum = 180;
			this->trackBar78->Minimum = -180;
			this->trackBar78->Name = L"trackBar78";
			this->trackBar78->Size = System::Drawing::Size(104, 45);
			this->trackBar78->TabIndex = 0;
			this->trackBar78->Scroll += gcnew System::EventHandler(this, &Form1::trackBar78_Scroll);
			// 
			// label20
			// 
			this->label20->AutoSize = true;
			this->label20->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label20->Location = System::Drawing::Point(872, 103);
			this->label20->Name = L"label20";
			this->label20->Size = System::Drawing::Size(23, 24);
			this->label20->TabIndex = 29;
			this->label20->Text = L"T";
			// 
			// label21
			// 
			this->label21->AutoSize = true;
			this->label21->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label21->Location = System::Drawing::Point(872, 363);
			this->label21->Name = L"label21";
			this->label21->Size = System::Drawing::Size(21, 24);
			this->label21->TabIndex = 30;
			this->label21->Text = L"1";
			// 
			// label22
			// 
			this->label22->AutoSize = true;
			this->label22->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label22->Location = System::Drawing::Point(872, 170);
			this->label22->Name = L"label22";
			this->label22->Size = System::Drawing::Size(21, 24);
			this->label22->TabIndex = 31;
			this->label22->Text = L"4";
			// 
			// label23
			// 
			this->label23->AutoSize = true;
			this->label23->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label23->Location = System::Drawing::Point(872, 296);
			this->label23->Name = L"label23";
			this->label23->Size = System::Drawing::Size(21, 24);
			this->label23->TabIndex = 32;
			this->label23->Text = L"2";
			// 
			// label24
			// 
			this->label24->AutoSize = true;
			this->label24->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label24->Location = System::Drawing::Point(872, 233);
			this->label24->Name = L"label24";
			this->label24->Size = System::Drawing::Size(21, 24);
			this->label24->TabIndex = 33;
			this->label24->Text = L"3";
			// 
			// label25
			// 
			this->label25->AutoSize = true;
			this->label25->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label25->Location = System::Drawing::Point(872, 484);
			this->label25->Name = L"label25";
			this->label25->Size = System::Drawing::Size(23, 24);
			this->label25->TabIndex = 34;
			this->label25->Text = L"T";
			// 
			// label26
			// 
			this->label26->AutoSize = true;
			this->label26->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label26->Location = System::Drawing::Point(872, 553);
			this->label26->Name = L"label26";
			this->label26->Size = System::Drawing::Size(21, 24);
			this->label26->TabIndex = 30;
			this->label26->Text = L"4";
			// 
			// label27
			// 
			this->label27->AutoSize = true;
			this->label27->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label27->Location = System::Drawing::Point(872, 616);
			this->label27->Name = L"label27";
			this->label27->Size = System::Drawing::Size(21, 24);
			this->label27->TabIndex = 30;
			this->label27->Text = L"3";
			// 
			// label28
			// 
			this->label28->AutoSize = true;
			this->label28->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label28->Location = System::Drawing::Point(872, 684);
			this->label28->Name = L"label28";
			this->label28->Size = System::Drawing::Size(21, 24);
			this->label28->TabIndex = 35;
			this->label28->Text = L"2";
			// 
			// label29
			// 
			this->label29->AutoSize = true;
			this->label29->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label29->Location = System::Drawing::Point(872, 749);
			this->label29->Name = L"label29";
			this->label29->Size = System::Drawing::Size(21, 24);
			this->label29->TabIndex = 30;
			this->label29->Text = L"1";
			// 
			// label30
			// 
			this->label30->AutoSize = true;
			this->label30->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label30->Location = System::Drawing::Point(551, 707);
			this->label30->Name = L"label30";
			this->label30->Size = System::Drawing::Size(130, 25);
			this->label30->TabIndex = 36;
			this->label30->Text = L"Joint Name";
			// 
			// textBox80
			// 
			this->textBox80->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->textBox80->Location = System::Drawing::Point(468, 743);
			this->textBox80->Name = L"textBox80";
			this->textBox80->Size = System::Drawing::Size(280, 31);
			this->textBox80->TabIndex = 37;
			this->textBox80->Text = L"--";
			this->textBox80->TextAlign = System::Windows::Forms::HorizontalAlignment::Center;
			// 
			// button4
			// 
			this->button4->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->button4->Location = System::Drawing::Point(264, 783);
			this->button4->Name = L"button4";
			this->button4->Size = System::Drawing::Size(257, 37);
			this->button4->TabIndex = 38;
			this->button4->Text = L"Launch Sequence";
			this->button4->UseVisualStyleBackColor = true;
			this->button4->Click += gcnew System::EventHandler(this, &Form1::button4_Click);
			// 
			// button5
			// 
			this->button5->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->button5->Location = System::Drawing::Point(544, 783);
			this->button5->Name = L"button5";
			this->button5->Size = System::Drawing::Size(257, 37);
			this->button5->TabIndex = 39;
			this->button5->Text = L"Save Sequence";
			this->button5->UseVisualStyleBackColor = true;
			// 
			// Form1
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->BackColor = System::Drawing::SystemColors::ButtonFace;
			this->ClientSize = System::Drawing::Size(1369, 874);
			this->Controls->Add(this->button5);
			this->Controls->Add(this->button4);
			this->Controls->Add(this->textBox80);
			this->Controls->Add(this->label30);
			this->Controls->Add(this->label29);
			this->Controls->Add(this->label28);
			this->Controls->Add(this->label27);
			this->Controls->Add(this->label26);
			this->Controls->Add(this->label25);
			this->Controls->Add(this->label24);
			this->Controls->Add(this->label23);
			this->Controls->Add(this->label22);
			this->Controls->Add(this->label21);
			this->Controls->Add(this->label20);
			this->Controls->Add(this->panel8);
			this->Controls->Add(this->panel7);
			this->Controls->Add(this->label6);
			this->Controls->Add(this->label7);
			this->Controls->Add(this->label15);
			this->Controls->Add(this->panel5);
			this->Controls->Add(this->panel6);
			this->Controls->Add(this->label10);
			this->Controls->Add(this->label11);
			this->Controls->Add(this->label12);
			this->Controls->Add(this->panel3);
			this->Controls->Add(this->panel4);
			this->Controls->Add(this->label9);
			this->Controls->Add(this->label5);
			this->Controls->Add(this->label4);
			this->Controls->Add(this->panel2);
			this->Controls->Add(this->label8);
			this->Controls->Add(this->textBox2);
			this->Controls->Add(this->button3);
			this->Controls->Add(this->panel1);
			this->Controls->Add(this->button2);
			this->Controls->Add(this->button1);
			this->Controls->Add(this->connect);
			this->Controls->Add(this->label1);
			this->Name = L"Form1";
			this->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->Text = L"iCub Interface";
			this->Load += gcnew System::EventHandler(this, &Form1::Form1_Load);
			this->panel1->ResumeLayout(false);
			this->panel1->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar7))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar6))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar5))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar4))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar3))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar2))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar1))->EndInit();
			this->panel2->ResumeLayout(false);
			this->panel2->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar8))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar9))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar10))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar11))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar12))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar13))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar14))->EndInit();
			this->panel3->ResumeLayout(false);
			this->panel3->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar16))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar17))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar18))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar19))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar20))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar21))->EndInit();
			this->panel4->ResumeLayout(false);
			this->panel4->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar23))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar24))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar25))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar26))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar27))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar28))->EndInit();
			this->panel5->ResumeLayout(false);
			this->panel5->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar15))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar34))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar22))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar29))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar30))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar31))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar32))->EndInit();
			this->panel6->ResumeLayout(false);
			this->panel6->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar33))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar35))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar36))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar37))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar38))->EndInit();
			this->panel7->ResumeLayout(false);
			this->panel7->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar54))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar55))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar56))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar57))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar58))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar53))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar49))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar50))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar51))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar52))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar46))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar47))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar48))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar39))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar40))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar41))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar42))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar43))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar44))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar45))->EndInit();
			this->panel8->ResumeLayout(false);
			this->panel8->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar59))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar60))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar61))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar62))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar63))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar64))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar65))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar66))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar67))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar68))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar69))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar70))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar71))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar72))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar73))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar74))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar75))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar76))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar77))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar78))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}

#pragma endregion
	private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {


				 //
			 }
	private: System::Void label1_Click(System::Object^  sender, System::EventArgs^  e) {
			 }
	private: System::Void button1_Click_1(System::Object^  sender, System::EventArgs^  e) {
				 if(m_srv == nullptr){

					   std::string Service_name ="";
//                       IPv4 ip;
                       std::string str = "";
                   //    str = getMyIP(&ip);

							/*	if( str == "" )
									{
									  std::cout <<"Theres been an error !" << std::endl;
      
									}
                        */

					   
								   Service_name = "Configure_Robot";
								//   Service_name += str;
                                 

                             String^ ipping = gcnew String(Service_name.c_str());

					 m_srv = gcnew iCub_Interface(ipping);
					 cli::array<System::String^>^ args = System::Environment::GetCommandLineArgs();
					 if(m_srv->connect(args[1], System::Convert::ToInt32(args[2]))){
						 m_connected = true;
						 m_srv->connectToViewer();
						 m_srv->setAutoExitProc(true);
					//	 m_srv->sendMsg("Robot_000",ipping);

					 }
				 }
			 }
private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
			 if(m_srv != nullptr){
				 m_srv->checkRecvData(100);
				 int ssize = m_srv->getScoreSize();
				  System::String^ msg = m_srv->getMessage();
//				  this->textBox1->Text = msg;
				 // メッセージが来た
				 for(int i = 0; i < ssize; i++){
					 int score = m_srv->getScore();
					// System::String^ msg = m_srv->getMessage();
				//	 System::String^ listitem_msg = msg;// + "                    " + score.ToString();
				//	 System::String^ listitem_score = score.ToString();
					// this->listBox1->Items->Add(listitem_msg);
					// this->listBox2->Items->Add(listitem_score);
				//	 int total = m_srv->getTotal();
					// this->textBox1->Text = total.ToString();

//					 this->textBox1->Text = msg;

				 }
			 }
		 }
private: System::Void button1_Click_2(System::Object^  sender, System::EventArgs^  e) {
			 m_srv->disconnect();
			 this->connect->Enabled = true;
			 m_connected = false;
			 m_srv = nullptr;
		 }
private: System::Void listBox1_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void Form1_Load(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) {
  //			 m_srv->setTotal(0);
  //			 this->textBox1->Text = L"0";
  //			 this->listBox1->Items->Clear();
			// this->listBox2->Items->Clear();
		//	 System::Windows::Forms::ListBox^  listBox5;
		//	 this->listBox5 = (gcnew System::Windows::Forms::ListBox());
			// this->Controls->Add(this->listBox5);
			 m_srv->sendMsg("Robot_000","Capture!Tester");

        /*
            for (size_t z = 0; z < container.size(); z++){
                listBox5->Items->Add(container[z]);
          }
         */


		 }
private: System::Void button3_Click(System::Object^  sender, System::EventArgs^  e) {

 m_srv->sendMsg("multiuserctr_0","1");

		 }
private: System::Void trackBar5_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 //X3
			 //this->trackBar5
     this->textBox1->Text = this->trackBar5->Value.ToString();
	//  m_srv->sendMsg("robot_000",ipping);
		 }

private: System::Void trackBar4_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 //X2
				 this->textBox3->Text = this->trackBar4->Value.ToString();
				 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar1_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 //X1
			 this->textBox4->Text = this->trackBar1->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar2_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 //Y1
			 this->textBox5->Text = this->trackBar2->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar6_Scroll(System::Object^  sender, System::EventArgs^  e) {

			 this->textBox6->Text = this->trackBar6->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);

		 }
private: System::Void trackBar3_Scroll(System::Object^  sender, System::EventArgs^  e) {
			//Z1
			 this->textBox7->Text = this->trackBar3->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar7_Scroll(System::Object^  sender, System::EventArgs^  e) {

			 this->textBox8->Text = this->trackBar7->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar14_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox13->Text = this->trackBar14->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar13_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox12->Text = this->trackBar13->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar12_Scroll(System::Object^  sender, System::EventArgs^  e) {
             this->textBox10->Text = this->trackBar12->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar11_Scroll(System::Object^  sender, System::EventArgs^  e) {
                 this->textBox14->Text = this->trackBar11->Value.ToString();
				 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar10_Scroll(System::Object^  sender, System::EventArgs^  e) {
			    this->textBox15->Text = this->trackBar10->Value.ToString();
				//  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar9_Scroll(System::Object^  sender, System::EventArgs^  e) {
			    this->textBox11->Text = this->trackBar9->Value.ToString();
				//  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar8_Scroll(System::Object^  sender, System::EventArgs^  e) {
			    this->textBox9->Text = this->trackBar8->Value.ToString();
				//  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar28_Scroll(System::Object^  sender, System::EventArgs^  e) {
			    this->textBox27->Text = this->trackBar28->Value.ToString();
				//  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar27_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox26->Text = this->trackBar27->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar26_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox24->Text = this->trackBar26->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar25_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox28->Text = this->trackBar25->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar24_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox29->Text = this->trackBar24->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar23_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox25->Text = this->trackBar23->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar21_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox20->Text = this->trackBar21->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar20_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox19->Text = this->trackBar20->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar19_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox17->Text = this->trackBar19->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar18_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox21->Text = this->trackBar18->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar17_Scroll(System::Object^  sender, System::EventArgs^  e) {

			 this->textBox22->Text = this->trackBar17->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar16_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox18->Text = this->trackBar16->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar38_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox37->Text = this->trackBar38->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar37_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox36->Text = this->trackBar37->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar36_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox34->Text = this->trackBar36->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar35_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox38->Text = this->trackBar35->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar33_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox35->Text = this->trackBar33->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar34_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox39->Text = this->trackBar34->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar32_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox31->Text = this->trackBar32->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar31_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox30->Text = this->trackBar31->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar30_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox16->Text = this->trackBar30->Value.ToString();
			 //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar29_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox32->Text = this->trackBar29->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar15_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox23->Text = this->trackBar15->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar22_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox33->Text = this->trackBar22->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar45_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox44->Text = this->trackBar45->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar44_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 	this->textBox43->Text = this->trackBar44->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar43_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 	this->textBox41->Text = this->trackBar43->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar49_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox50->Text = this->trackBar49->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar52_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 			 this->textBox53->Text = this->trackBar52->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar42_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 			 this->textBox45->Text = this->trackBar42->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar48_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 			 this->textBox49->Text = this->trackBar48->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar47_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 			 this->textBox48->Text = this->trackBar47->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar46_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 			 this->textBox47->Text = this->trackBar46->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar50_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 			 this->textBox51->Text = this->trackBar50->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar41_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 	 this->textBox46->Text = this->trackBar41->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);

		 }
private: System::Void trackBar40_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 	this->textBox42->Text = this->trackBar40->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar39_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox40->Text = this->trackBar39->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar51_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox52->Text = this->trackBar51->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar53_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox54->Text = this->trackBar53->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar58_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox59->Text = this->trackBar58->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar57_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox58->Text = this->trackBar57->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar56_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox57->Text = this->trackBar56->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar55_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox56->Text = this->trackBar55->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar54_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox55->Text = this->trackBar54->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }

private: System::Void trackBar77_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox76->Text = this->trackBar77->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar78_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox77->Text = this->trackBar78->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }

private: System::Void trackBar76_Scroll(System::Object^  sender, System::EventArgs^  e) {
			this->textBox74->Text = this->trackBar76->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar65_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox66->Text = this->trackBar65->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar68_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox69->Text = this->trackBar68->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar75_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox78->Text = this->trackBar75->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar71_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox72->Text = this->trackBar71->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar70_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 	this->textBox71->Text = this->trackBar70->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar69_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 	this->textBox70->Text = this->trackBar69->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar66_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox67->Text = this->trackBar66->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar74_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox79->Text = this->trackBar74->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar73_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox75->Text = this->trackBar73->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar72_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 	this->textBox73->Text = this->trackBar72->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar67_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 	this->textBox68->Text = this->trackBar67->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar64_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox65->Text = this->trackBar64->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar63_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox64->Text = this->trackBar63->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar62_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 	this->textBox63->Text = this->trackBar62->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar61_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 	this->textBox62->Text = this->trackBar61->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar60_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->textBox61->Text = this->trackBar60->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
private: System::Void trackBar59_Scroll(System::Object^  sender, System::EventArgs^  e) {
			  this->textBox60->Text = this->trackBar59->Value.ToString();
			  //  m_srv->sendMsg("robot_000",ipping);
		 }
// Left Arm
private: System::Void textBox4_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			  String^ str = "LARM_JOINT0:";
			  this->textBox80->Text = str ;
			  str += this->textBox4->Text;
			 m_srv->sendMsg("Robot_000",str);
			 // left arm joint 0 on X
		 }

private: System::Void textBox5_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			  String^ str = "LARM_JOINT1:";
			  this->textBox80->Text = str ;
			  str += this->textBox5->Text;
			 m_srv->sendMsg("Robot_000",str);
            // left arm joint 1 on y
		 }
private: System::Void textBox7_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			  String^ str = "LARM_JOINT2:";
			  this->textBox80->Text = str ;
			  str += this->textBox7->Text;
			 m_srv->sendMsg("Robot_000",str);
		   // left arm joint 2 on z
		 }
private: System::Void textBox3_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		   	String^ str = "LARM_JOINT3:";
			this->textBox80->Text = str ;
			 str += this->textBox3->Text;
			 m_srv->sendMsg("Robot_000",str);
		   // left arm joint 3 on x
		 }
private: System::Void textBox1_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		   	 String^ str = "LARM_JOINT4:";
			 this->textBox80->Text = str ;
			 str += this->textBox1->Text;
			 m_srv->sendMsg("Robot_000",str);
		   // left arm joint 4 on x
		 }
private: System::Void textBox6_TextChanged(System::Object^  sender, System::EventArgs^  e) {
				String^ str = "LARM_JOINT5:";
				this->textBox80->Text = str ;
			 str += this->textBox6->Text;
			 m_srv->sendMsg("Robot_000",str);
		   // left arm joint 5 on y
		 }
private: System::Void textBox8_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 String^ str = "LARM_JOINT6:";
			 this->textBox80->Text = str ;
			 str += this->textBox8->Text;
			 m_srv->sendMsg("Robot_000",str);
		   // left arm joint 6 on z
		 }


// Right Arm

private: System::Void textBox13_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 String^ str = "RARM_JOINT0:";
			 this->textBox80->Text = str ;
			 str += this->textBox13->Text;
			 m_srv->sendMsg("Robot_000",str);
		       // Right arm joint 0 on x
		 }
private: System::Void textBox12_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RARM_JOINT1:";
				this->textBox80->Text = str ;
			    str += this->textBox12->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right arm joint 1 on y

		 }
private: System::Void textBox10_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RARM_JOINT2:";
				this->textBox80->Text = str ;
			    str += this->textBox10->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right arm joint 2 on z
		 }

private: System::Void textBox14_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RARM_JOINT3:";
				this->textBox80->Text = str ;
			    str += this->textBox14->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right arm joint 3 on x
		 }
private: System::Void textBox15_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RARM_JOINT4:";
				this->textBox80->Text = str ;
			    str += this->textBox15->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right arm joint 4 on x
		 }
private: System::Void textBox11_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RARM_JOINT5:";
				this->textBox80->Text = str ;
			    str += this->textBox11->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right arm joint 5 on y
		 }
private: System::Void textBox9_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RARM_JOINT6:";
				this->textBox80->Text = str ;
			    str += this->textBox9->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right arm joint 6 on z
		 }

// Left Leg

private: System::Void textBox27_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LLEG_JOINT0:";
				this->textBox80->Text = str ;
			    str += this->textBox27->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Left leg joint 0 on x
		 }

private: System::Void textBox26_TextChanged(System::Object^  sender, System::EventArgs^  e) {			 
			 	String^ str = "LLEG_JOINT1:";
				this->textBox80->Text = str ;
			    str += this->textBox26->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Left leg joint 1 on y

		 }
private: System::Void textBox24_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LLEG_JOINT2:";
				this->textBox80->Text = str ;
			    str += this->textBox24->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Left leg joint 2 on z
		 }

private: System::Void textBox28_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LLEG_JOINT3:";
				this->textBox80->Text = str ;
			    str += this->textBox28->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Left leg joint 3 on x
		 }
private: System::Void textBox29_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	 String^ str = "LLEG_JOINT4:";
				 this->textBox80->Text = str ;
			    str += this->textBox29->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Left leg joint 4 on x
		 }
private: System::Void textBox25_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			   String^ str = "LLEG_JOINT5:";
			   this->textBox80->Text = str ;
			    str += this->textBox25->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Left leg joint 5 on z
		 }

// Right Leg
private: System::Void textBox20_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RLEG_JOINT0:";
				this->textBox80->Text = str ;
			    str += this->textBox20->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right leg joint 0 on x
		 }
private: System::Void textBox19_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RLEG_JOINT1:";
				this->textBox80->Text = str ;
			    str += this->textBox19->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right leg joint 1 on y
		 }
private: System::Void textBox17_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RLEG_JOINT2:";
				this->textBox80->Text = str ;
			    str += this->textBox17->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right leg joint 2 on z
		 }
private: System::Void textBox21_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RLEG_JOINT3:";
				this->textBox80->Text = str ;
			    str += this->textBox21->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right leg joint 3 on x
		 }
private: System::Void textBox22_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RLEG_JOINT4:";
				this->textBox80->Text = str ;
			    str += this->textBox22->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right leg joint 4 on x
		 }
private: System::Void textBox18_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RLEG_JOINT5:";
				this->textBox80->Text = str ;
			    str += this->textBox18->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right leg joint 5 on z
		 }



private: System::Void textBox37_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "ROOT_JOINT0:";
				this->textBox80->Text = str ;
			    str += this->textBox37->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Root joint 0 on x
		 }
private: System::Void textBox36_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "ROOT_JOINT1:";
				this->textBox80->Text = str ;
			    str += this->textBox36->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Root joint 1 on y
		 }
private: System::Void textBox34_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "ROOT_JOINT2:";
				this->textBox80->Text = str ;
			    str += this->textBox34->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Root joint 2 on z
		 }
private: System::Void textBox31_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "NECK_JOINT0:";
				this->textBox80->Text = str ;
			    str += this->textBox31->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Neck joint 0 on x
		 }
private: System::Void textBox30_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "NECK_JOINT1:";
				this->textBox80->Text = str ;
			    str += this->textBox30->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Neck joint 1 on y
		 }
private: System::Void textBox16_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "NECK_JOINT2:";
				this->textBox80->Text = str ;
			    str += this->textBox16->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Neck joint 2 on z
		 }
private: System::Void textBox32_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LEYE_JOINT0:";
				this->textBox80->Text = str ;
			    str += this->textBox32->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Left eye joint 0 on x
		 }
private: System::Void textBox23_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LEYE_JOINT1:";
				this->textBox80->Text = str ;
			    str += this->textBox23->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Left eye joint 1 on y
		 }
private: System::Void textBox33_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "REYE_JOINT0:";
				this->textBox80->Text = str ;
			    str += this->textBox33->Text;
			    m_srv->sendMsg("Robot_000",str);
		       //Right eye joint 0 on x
		 }
private: System::Void textBox39_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "REYE_JOINT1:";
				this->textBox80->Text = str ;
			    str += this->textBox39->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right eye joint 1 on y
		 }
// Left Hand 
// thumb



private: System::Void textBox44_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			    String^ str = "LHAND_THUMB_BASE:";
				this->textBox80->Text = str ;
			    str += this->textBox44->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Thumb joint base 
		 }
private: System::Void textBox45_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_THUMB_0:";
				this->textBox80->Text = str ;
			    str += this->textBox45->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Thumb joint 0 
		 }
private: System::Void textBox46_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_THUMB_1:";
				this->textBox80->Text = str ;
			    str += this->textBox46->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Thumb joint 1
		 }
private: System::Void textBox59_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_THUMB_2:";
				this->textBox80->Text = str ;
			    str += this->textBox59->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Thumb joint 2 
		 }

// Finger 4

private: System::Void textBox43_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER4_BASE:";
				this->textBox80->Text = str ;
			    str += this->textBox43->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 4 base
		 }
private: System::Void textBox49_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER4_0:";
				this->textBox80->Text = str ;
			    str += this->textBox49->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 4 joint 0
		 }
private: System::Void textBox42_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER4_1:";
				this->textBox80->Text = str ;
			    str += this->textBox42->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 4 joint 1
		 }
private: System::Void textBox58_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER4_2:";
				this->textBox80->Text = str ;
			    str += this->textBox58->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 4 joint 2
		 }

// finger 3 
private: System::Void textBox41_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER3_BASE:";
				this->textBox80->Text = str ;
			    str += this->textBox41->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 3 base
		 }
private: System::Void textBox48_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER3_0:";
				this->textBox80->Text = str ;
			    str += this->textBox48->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 3 joint 0
		 }
private: System::Void textBox40_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER3_1:";
				this->textBox80->Text = str ;
			    str += this->textBox40->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 3 joint 1
		 }
private: System::Void textBox57_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER3_2:";
				this->textBox80->Text = str ;
			    str += this->textBox57->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 3 joint 2
		 }

// Finger 2
private: System::Void textBox50_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER2_BASE:";
				this->textBox80->Text = str ;
			    str += this->textBox50->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 2 base
		 }
private: System::Void textBox47_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER2_0:";
				this->textBox80->Text = str ;
			    str += this->textBox47->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 2 joint 0
		 }


private: System::Void textBox52_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER2_1:";
				this->textBox80->Text = str ;
			    str += this->textBox52->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 2 joint 1
		 }
private: System::Void textBox56_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER2_2:";
				this->textBox80->Text = str ;
			    str += this->textBox56->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 2 joint 2
		 }


// Fnger 1
private: System::Void textBox53_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER1_BASE:";
				this->textBox80->Text = str ;
			    str += this->textBox53->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 1 base
		 }

private: System::Void textBox51_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER1_0:";
				this->textBox80->Text = str ;
			    str += this->textBox51->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 1 joint 0
		 }
private: System::Void textBox54_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER1_1:";
				this->textBox80->Text = str ;
			    str += this->textBox54->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 1 joint 1
		 }

private: System::Void textBox55_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "LHAND_FINGER1_2:";
				this->textBox80->Text = str ;
			    str += this->textBox55->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Finger 1 joint 2
		 }


// Right Hand
// Thumb
private: System::Void textBox77_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			   String^ str = "RHAND_THUMB_BASE:";
			   this->textBox80->Text = str ;
			    str += this->textBox77->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Thumb base
		 }

private: System::Void textBox78_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			   String^ str = "RHAND_THUMB_0:";
			   this->textBox80->Text = str ;
			    str += this->textBox78->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Thumb joint 0
		 }
private: System::Void textBox79_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RHAND_THUMB_1:";
				this->textBox80->Text = str ;
			    str += this->textBox79->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Thumb joint 1
		 }
private: System::Void textBox64_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RHAND_THUMB_2:";
				this->textBox80->Text = str ;
			    str += this->textBox64->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Thumb joint 2
		 }

// Finger 4

private: System::Void textBox76_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			   String^ str = "RHAND_FINGER4_BASE:";
			   this->textBox80->Text = str ;
			    str += this->textBox76->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 4 base
		 }
private: System::Void textBox72_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RHAND_FINGER4_0:";
				this->textBox80->Text = str ;
			    str += this->textBox72->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 4 joint 0
		 }

private: System::Void textBox75_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RHAND_FINGER4_1:";
				this->textBox80->Text = str ;
			    str += this->textBox75->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 4 joint 1
		 }
private: System::Void textBox63_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			    String^ str = "RHAND_FINGER4_2:";
				this->textBox80->Text = str ;
			    str += this->textBox63->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 4 joint 2
		 }
private: System::Void textBox74_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RHAND_FINGER3_BASE:";
				this->textBox80->Text = str ;
			    str += this->textBox74->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 3 base
		 }
private: System::Void textBox71_TextChanged(System::Object^  sender, System::EventArgs^  e) {
				String^ str = "RHAND_FINGER3_0:";
				this->textBox80->Text = str ;
			    str += this->textBox71->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 3 joint 0
		 }
private: System::Void textBox73_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RHAND_FINGER3_1:";
				this->textBox80->Text = str ;
			    str += this->textBox73->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 3 joint 1
		 }
private: System::Void textBox62_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RHAND_FINGER3_2:";
				this->textBox80->Text = str ;
			    str += this->textBox62->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 3 joint 2
		 }
private: System::Void textBox66_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RHAND_FINGER2_BASE:";
				this->textBox80->Text = str ;
			    str += this->textBox66->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 2 base
		 }
private: System::Void textBox70_TextChanged(System::Object^  sender, System::EventArgs^  e) {		
			    String^ str = "RHAND_FINGER2_0:";
				this->textBox80->Text = str ;
			    str += this->textBox70->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 2 joint 0
		 }
private: System::Void textBox68_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RHAND_FINGER2_1:";
				this->textBox80->Text = str ;
			    str += this->textBox68->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 2 joint 1
		 }
private: System::Void textBox61_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RHAND_FINGER2_2:";
				this->textBox80->Text = str ;
			    str += this->textBox61->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 2 joint 2
		 }
private: System::Void textBox69_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RHAND_FINGER1_BASE:";
				this->textBox80->Text = str ;
			    str += this->textBox69->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 1 base
		 }
private: System::Void textBox67_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RHAND_FINGER1_0:";
				this->textBox80->Text = str ;
			    str += this->textBox67->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 1 joint 0
		 }
private: System::Void textBox65_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			    String^ str = "RHAND_FINGER1_1:";
				this->textBox80->Text = str ;
			    str += this->textBox65->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger 1 joint 1
		 }
private: System::Void textBox60_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 	String^ str = "RHAND_FINGER1_2:";
				this->textBox80->Text = str ;
			    str += this->textBox60->Text;
			    m_srv->sendMsg("Robot_000",str);
		       // Right Finger q joint 0
		 }
private: System::Void button3_Click_1(System::Object^  sender, System::EventArgs^  e) {
			  m_srv->sendMsg("Robot_000","Update??kk");
		 }
private: System::Void button4_Click(System::Object^  sender, System::EventArgs^  e) {
			 m_srv->sendMsg("Robot_000","LaunchSequence#Tester");
		 }
private: System::Void textBox38_TextChanged(System::Object^  sender, System::EventArgs^  e) {
           	    String^ str = "PositionX=";
				this->textBox80->Text = str ;
			    str += this->textBox38->Text;
			    m_srv->sendMsg("Robot_000",str);
		 }
private: System::Void textBox35_TextChanged(System::Object^  sender, System::EventArgs^  e) {
                String^ str = "PositionY=";
				this->textBox80->Text = str ;
			    str += this->textBox35->Text;
			    m_srv->sendMsg("Robot_000",str);

		 }
};
}

