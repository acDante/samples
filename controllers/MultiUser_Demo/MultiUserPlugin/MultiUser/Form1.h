

#include "MultiUsers.h"

#include <iostream>
#include<sstream>
#include <cstdlib>





namespace MultiUserInterface {

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
			m_srv = gcnew MultiUsersInterface("MultiUsers");
			

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
	private: System::Windows::Forms::ListBox^  listBox1;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::TextBox^  textBox1;
	private: System::Windows::Forms::ListBox^  listBox2;
	private: System::Windows::Forms::ListBox^  listBox5;	
	//private: System::Windows::Forms::ListBox^  listBox5;
			 
			 //sigverse::SIGService ^srv = gcnew sigverse::SIGService("RobocupReferee");
			 MultiUsersInterface ^m_srv;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Button^  button3;
	private: System::Windows::Forms::TextBox^  textBox2;
	private: System::Windows::Forms::TextBox^  textBox3;
	private: System::Windows::Forms::TextBox^  textBox4;
	private: System::Windows::Forms::TextBox^  textBox5;
	private: System::Windows::Forms::Button^  button4;









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
			this->listBox1 = (gcnew System::Windows::Forms::ListBox());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->textBox1 = (gcnew System::Windows::Forms::TextBox());
			this->listBox2 = (gcnew System::Windows::Forms::ListBox());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->button3 = (gcnew System::Windows::Forms::Button());
			this->textBox2 = (gcnew System::Windows::Forms::TextBox());
			this->textBox3 = (gcnew System::Windows::Forms::TextBox());
			this->textBox4 = (gcnew System::Windows::Forms::TextBox());
			this->textBox5 = (gcnew System::Windows::Forms::TextBox());
			this->button4 = (gcnew System::Windows::Forms::Button());
			this->SuspendLayout();
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label1->Location = System::Drawing::Point(81, 126);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(188, 25);
			this->label1->TabIndex = 0;
			this->label1->Text = L"Connection Status";
			this->label1->Click += gcnew System::EventHandler(this, &Form1::label1_Click);
			// 
			// connect
			// 
			this->connect->Enabled = false;
			this->connect->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->connect->Location = System::Drawing::Point(12, 281);
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
			this->button1->Location = System::Drawing::Point(128, 282);
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
			this->button2->Location = System::Drawing::Point(280, 282);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(85, 38);
			this->button2->TabIndex = 3;
			this->button2->Text = L"Update";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &Form1::button2_Click);
			// 
			// listBox1
			// 
			this->listBox1->BackColor = System::Drawing::SystemColors::InfoText;
			this->listBox1->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->listBox1->ForeColor = System::Drawing::SystemColors::Menu;
			this->listBox1->FormattingEnabled = true;
			this->listBox1->HorizontalScrollbar = true;
			this->listBox1->ItemHeight = 21;
			this->listBox1->Location = System::Drawing::Point(12, 244);
			this->listBox1->Name = L"listBox1";
			this->listBox1->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->listBox1->Size = System::Drawing::Size(175, 4);
			this->listBox1->TabIndex = 4;
			this->listBox1->SelectedIndexChanged += gcnew System::EventHandler(this, &Form1::listBox1_SelectedIndexChanged);
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label2->Location = System::Drawing::Point(188, 26);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(56, 20);
			this->label2->TabIndex = 5;
			this->label2->Text = L"Status";
			// 
			// textBox1
			// 
			this->textBox1->BackColor = System::Drawing::SystemColors::InfoText;
			this->textBox1->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 18, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->textBox1->ForeColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBox1->Location = System::Drawing::Point(12, 178);
			this->textBox1->Name = L"textBox1";
			this->textBox1->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->textBox1->Size = System::Drawing::Size(353, 31);
			this->textBox1->TabIndex = 6;
			this->textBox1->Text = L"0";
			this->textBox1->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			// 
			// listBox2
			// 
			this->listBox2->BackColor = System::Drawing::SystemColors::InfoText;
			this->listBox2->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->listBox2->ForeColor = System::Drawing::SystemColors::Menu;
			this->listBox2->FormattingEnabled = true;
			this->listBox2->HorizontalScrollbar = true;
			this->listBox2->ItemHeight = 21;
			this->listBox2->Location = System::Drawing::Point(262, 244);
			this->listBox2->Name = L"listBox2";
			this->listBox2->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->listBox2->Size = System::Drawing::Size(103, 4);
			this->listBox2->TabIndex = 7;
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->label3->Location = System::Drawing::Point(46, 26);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(76, 20);
			this->label3->TabIndex = 8;
			this->label3->Text = L"Avatar ID";
			// 
			// button3
			// 
			this->button3->Location = System::Drawing::Point(293, 52);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(72, 23);
			this->button3->TabIndex = 9;
			this->button3->Text = L"Connect";
			this->button3->UseVisualStyleBackColor = true;
			this->button3->Click += gcnew System::EventHandler(this, &Form1::button3_Click);
			// 
			// textBox2
			// 
			this->textBox2->Location = System::Drawing::Point(17, 52);
			this->textBox2->Multiline = true;
			this->textBox2->Name = L"textBox2";
			this->textBox2->Size = System::Drawing::Size(152, 23);
			this->textBox2->TabIndex = 10;
			this->textBox2->TextChanged += gcnew System::EventHandler(this, &Form1::textBox2_TextChanged);
			// 
			// textBox3
			// 
			this->textBox3->Location = System::Drawing::Point(185, 52);
			this->textBox3->Multiline = true;
			this->textBox3->Name = L"textBox3";
			this->textBox3->Size = System::Drawing::Size(84, 23);
			this->textBox3->TabIndex = 11;
			// 
			// textBox4
			// 
			this->textBox4->Location = System::Drawing::Point(185, 80);
			this->textBox4->Multiline = true;
			this->textBox4->Name = L"textBox4";
			this->textBox4->Size = System::Drawing::Size(84, 23);
			this->textBox4->TabIndex = 14;
			// 
			// textBox5
			// 
			this->textBox5->Location = System::Drawing::Point(17, 80);
			this->textBox5->Multiline = true;
			this->textBox5->Name = L"textBox5";
			this->textBox5->Size = System::Drawing::Size(152, 23);
			this->textBox5->TabIndex = 13;
			// 
			// button4
			// 
			this->button4->Location = System::Drawing::Point(293, 80);
			this->button4->Name = L"button4";
			this->button4->Size = System::Drawing::Size(72, 23);
			this->button4->TabIndex = 12;
			this->button4->Text = L"Connect";
			this->button4->UseVisualStyleBackColor = true;
			// 
			// Form1
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->BackColor = System::Drawing::SystemColors::ButtonFace;
			this->ClientSize = System::Drawing::Size(390, 352);
			this->Controls->Add(this->textBox4);
			this->Controls->Add(this->textBox5);
			this->Controls->Add(this->button4);
			this->Controls->Add(this->textBox3);
			this->Controls->Add(this->textBox2);
			this->Controls->Add(this->button3);
			this->Controls->Add(this->label3);
			this->Controls->Add(this->listBox2);
			this->Controls->Add(this->textBox1);
			this->Controls->Add(this->label2);
			this->Controls->Add(this->listBox1);
			this->Controls->Add(this->button2);
			this->Controls->Add(this->button1);
			this->Controls->Add(this->connect);
			this->Controls->Add(this->label1);
			this->Name = L"Form1";
			this->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->Text = L"MultiUser Interface";
			this->Load += gcnew System::EventHandler(this, &Form1::Form1_Load);
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
                       IPv4 ip;
                       std::string str = "";
                       str = getMyIP(&ip);

							/*	if( str == "" )
									{
									  std::cout <<"Theres been an error !" << std::endl;
      
									}
                        */

					   
								   Service_name = "MultiUsersMenu ";
								   Service_name += str;
                                 

                             String^ ipping = gcnew String(Service_name.c_str());

					 m_srv = gcnew MultiUsersInterface(ipping);
					 cli::array<System::String^>^ args = System::Environment::GetCommandLineArgs();
					 if(m_srv->connect(args[1], System::Convert::ToInt32(args[2]))){
						 m_connected = true;
						 m_srv->connectToViewer();
						 m_srv->setAutoExitProc(true);
						 m_srv->sendMsg("multiuserctr_0",ipping);

					 }
				 }
			 }
private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
			 if(m_srv != nullptr){
				 m_srv->checkRecvData(100);
				 int ssize = m_srv->getScoreSize();
				  System::String^ msg = m_srv->getMessage();
				  this->textBox1->Text = msg;
				 // メッセージが来た
				 for(int i = 0; i < ssize; i++){
					 int score = m_srv->getScore();
					 System::String^ msg = m_srv->getMessage();
					 System::String^ listitem_msg = msg;// + "                    " + score.ToString();
					 System::String^ listitem_score = score.ToString();
					// this->listBox1->Items->Add(listitem_msg);
					// this->listBox2->Items->Add(listitem_score);
					 int total = m_srv->getTotal();
					// this->textBox1->Text = total.ToString();

					 this->textBox1->Text = msg;

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
			 m_srv->setTotal(0);
			 this->textBox1->Text = L"0";
			 this->listBox1->Items->Clear();
			// this->listBox2->Items->Clear();
		//	 System::Windows::Forms::ListBox^  listBox5;
			 this->listBox5 = (gcnew System::Windows::Forms::ListBox());
			// this->Controls->Add(this->listBox5);
			 m_srv->sendMsg("multiuserctr_0","Info");

        /*
            for (size_t z = 0; z < container.size(); z++){
                listBox5->Items->Add(container[z]);
          }
         */


		 }
private: System::Void button3_Click(System::Object^  sender, System::EventArgs^  e) {

 m_srv->sendMsg("multiuserctr_0","1");

		 }
private: System::Void textBox2_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		 }
};
}

