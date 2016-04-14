#pragma once
#include "DistanceSensorView.h"

namespace DistanceSensorViewWithOpenCV3 {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// MainForm の概要
	/// </summary>
	public ref class MainForm : public System::Windows::Forms::Form
	{
	public:
		cli::array<System::String^>^ args;
		MainForm(void)
		{
			InitializeComponent();
			//
			//TODO: ここにコンストラクター コードを追加します
			//
			m_srv = nullptr;
			m_connected = false;

			// 接続を試みる
			m_srv = gcnew DistanceSensorView("DistanceSensorViewService");
			args = System::Environment::GetCommandLineArgs();

			if (m_srv->connect(args[1], System::Convert::ToInt32(args[2]))){
				m_connected = true;

				m_srv->connectToViewer();
				m_srv->setAutoExitProc(true);
			}
		}

	protected:
		/// <summary>
		/// 使用中のリソースをすべてクリーンアップします。
		/// </summary>
		~MainForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Button^  generation_button;
	protected:
	private: System::Windows::Forms::Label^  entity_label;
	private: System::Windows::Forms::Label^  camera_id_label;
	private: System::Windows::Forms::TextBox^  entity;
	private: System::Windows::Forms::NumericUpDown^  cameraID;
	private: System::Windows::Forms::Timer^  timer;
	private: System::ComponentModel::IContainer^  components;

	private:
		/// <summary>
		/// 必要なデザイナー変数です。
		/// </summary>

	public: DistanceSensorView ^m_srv;
	public: bool m_connected;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// デザイナー サポートに必要なメソッドです。このメソッドの内容を
		/// コード エディターで変更しないでください。
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			this->generation_button = (gcnew System::Windows::Forms::Button());
			this->entity_label = (gcnew System::Windows::Forms::Label());
			this->camera_id_label = (gcnew System::Windows::Forms::Label());
			this->entity = (gcnew System::Windows::Forms::TextBox());
			this->cameraID = (gcnew System::Windows::Forms::NumericUpDown());
			this->timer = (gcnew System::Windows::Forms::Timer(this->components));
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->cameraID))->BeginInit();
			this->SuspendLayout();
			// 
			// generation_button
			// 
			this->generation_button->Location = System::Drawing::Point(85, 103);
			this->generation_button->Name = L"generation_button";
			this->generation_button->Size = System::Drawing::Size(75, 23);
			this->generation_button->TabIndex = 0;
			this->generation_button->Text = L"Generate";
			this->generation_button->UseVisualStyleBackColor = true;
			this->generation_button->Click += gcnew System::EventHandler(this, &MainForm::generation_button_Click);
			// 
			// entity_label
			// 
			this->entity_label->AutoSize = true;
			this->entity_label->Location = System::Drawing::Point(13, 13);
			this->entity_label->Name = L"entity_label";
			this->entity_label->Size = System::Drawing::Size(66, 12);
			this->entity_label->TabIndex = 1;
			this->entity_label->Text = L"Entity name";
			// 
			// camera_id_label
			// 
			this->camera_id_label->AutoSize = true;
			this->camera_id_label->Location = System::Drawing::Point(12, 38);
			this->camera_id_label->Name = L"camera_id_label";
			this->camera_id_label->Size = System::Drawing::Size(59, 12);
			this->camera_id_label->TabIndex = 2;
			this->camera_id_label->Text = L"Camera ID";
			// 
			// entity
			// 
			this->entity->Location = System::Drawing::Point(85, 10);
			this->entity->Name = L"entity";
			this->entity->Size = System::Drawing::Size(134, 19);
			this->entity->TabIndex = 3;
			this->entity->Text = L"robot_000";
			// 
			// cameraID
			// 
			this->cameraID->Location = System::Drawing::Point(85, 36);
			this->cameraID->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 9, 0, 0, 0 });
			this->cameraID->Minimum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 1, 0, 0, 0 });
			this->cameraID->Name = L"cameraID";
			this->cameraID->Size = System::Drawing::Size(43, 19);
			this->cameraID->TabIndex = 4;
			this->cameraID->TextAlign = System::Windows::Forms::HorizontalAlignment::Right;
			this->cameraID->Value = System::Decimal(gcnew cli::array< System::Int32 >(4) { 1, 0, 0, 0 });
			// 
			// timer
			// 
			this->timer->Enabled = true;
			this->timer->Tick += gcnew System::EventHandler(this, &MainForm::timer_Tick);
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(231, 138);
			this->Controls->Add(this->cameraID);
			this->Controls->Add(this->entity);
			this->Controls->Add(this->camera_id_label);
			this->Controls->Add(this->entity_label);
			this->Controls->Add(this->generation_button);
			this->Name = L"MainForm";
			this->Text = L"MainForm";
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->cameraID))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void timer_Tick(System::Object^  sender, System::EventArgs^  e) {
		for (int i = 0; i < m_srv->view_class->Count; i++){
			if (m_srv->view_class[i]->main_process()){
				m_srv->view_class->RemoveAt(i);
			}
		}
	}
private: System::Void generation_button_Click(System::Object^  sender, System::EventArgs^  e) {
	m_srv->addDistanceSensor(this->entity->Text, Convert::ToInt32(this->cameraID->Text));
	this->cameraID->Text = Convert::ToString(Convert::ToInt32(this->cameraID->Text)+1);
}
};
}
