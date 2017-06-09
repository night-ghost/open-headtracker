#pragma once

#include "CompassCalibrator.h"

namespace HeadTrackerGUI
{
	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Collections::Generic;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

    enum CALMODE { AXISALIGNMENT, MINMAXTRACKING };

	value struct Vector3d {
//	ref class Vector3d {
		Vector3f *v;
//		Vector3d(double x,double y,double z) {v=new Vector3f(x,y,z); }
//		Vector3d(Vector3f n) { v= new Vector3f(n); }
//		~Vector3d(void) { delete v; }
	};

	/// <summary>
	/// Summary for CalWizard
	/// </summary>
	public ref class CalWizard : public System::Windows::Forms::Form
	{
	public:
		CalWizard(void)
		{
			InitializeComponent();
			CurrentStep = 0;
            //SerialData = new float[10];
			SerialLine="";
			DigitLine="";

            serialbyte = 0;
            Serialindex = 0;
            
            MagXOffset = 0;
            MagYOffset = 0;
            MagZOffset = 0;

            MagXmax = 0; 
            MagXmin = 0;
            MagYmax = 0; 
            MagYmin = 0; 
            MagZmax = 0; 
            MagZmin = 0; 

            MagXReading = 0;
            MagYReading = 0;
            MagZReading = 0; 

            AccXOffset = 0;
            AccYOffset = 0;
            AccZOffset = 0;

            AccXMin = 0;
            AccXMax = 0;
            AccYMin = 0;
            AccYMax = 0;
            AccZMin = 0;
            AccZMax = 0;

            AccXReading = 0;
            AccYReading = 0;
            AccZReading = 0;

            CalMode = MINMAXTRACKING;
            TrackMinMax = false;

			AccData = gcnew List<Vector3d>();
			MagData = gcnew List<Vector3d>();

		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~CalWizard()
		{
			if (components)
			    delete components;

            if ( SerialData )
                delete SerialData;

			// free unmanaged memory 
			for(int i=0; i< MagData->Count; i++){
				Vector3d v = MagData[i];
				delete v.v;
			}
			for(int i=0; i< AccData->Count; i++){
				Vector3d v = AccData[i];
				delete v.v;
			}

		}

    private: System::Windows::Forms::Timer^  mainUpdateTimer;
    private: System::Windows::Forms::Button^  WizardNextBtn;
    private: System::Windows::Forms::Button^  WizardPrevBtn;
    private: System::Windows::Forms::Button^  WizardCancelBtn;
    private: System::Windows::Forms::Label^  WizardInstructionsLbl;
    private: System::Windows::Forms::PictureBox^  WizardPictureBox;
    private: System::Windows::Forms::GroupBox^  WizardDataGrp;
    private: System::Windows::Forms::Label^  label2;
    private: System::Windows::Forms::Label^  label1;
    private: System::Windows::Forms::Label^  AccCurZLbl;
    private: System::Windows::Forms::Label^  AccCurYLbl;
    private: System::Windows::Forms::Label^  AccCurXLbl;
    private: System::Windows::Forms::Label^  MagCurZLbl;
    private: System::Windows::Forms::Label^  MagCurYLbl;
    private: System::Windows::Forms::Label^  MagCurXLbl;


    private: System::Windows::Forms::Label^  label8;


    private: System::Windows::Forms::Label^  label3;
    private: System::Windows::Forms::Label^  AccZOffsetLbl;
    private: System::Windows::Forms::Label^  AccYOffsetLbl;
    private: System::Windows::Forms::Label^  AccXOffsetLbl;
    private: System::Windows::Forms::Label^  MagZOffsetLbl;
    private: System::Windows::Forms::Label^  MagYOffsetLbl;
    private: System::Windows::Forms::Label^  MagXOffsetLbl;
	private: System::Windows::Forms::Label^  AccMaxZ;
	private: System::Windows::Forms::Label^  AccMaxY;
	private: System::Windows::Forms::Label^  AccMaxX;
	private: System::Windows::Forms::Label^  AccMinZ;
	private: System::Windows::Forms::Label^  AccMinY;
	private: System::Windows::Forms::Label^  AccMinX;

	private: System::Windows::Forms::Label^  MagMaxZ;
	private: System::Windows::Forms::Label^  MagMaxY;
	private: System::Windows::Forms::Label^  MagMaxX;
	private: System::Windows::Forms::Label^  MagMinZ;
	private: System::Windows::Forms::Label^  MagMinY;
	private: System::Windows::Forms::Label^  MagMinX;

	private: System::Windows::Forms::Label^  label11;
	private: System::Windows::Forms::Label^  label12;
	private: System::Windows::Forms::Label^  label10;
	private: System::Windows::Forms::Label^  label9;
private: System::Windows::Forms::Label^  label16;
private: System::Windows::Forms::Label^  label17;
private: System::Windows::Forms::Label^  label18;
private: System::Windows::Forms::Label^  label15;
private: System::Windows::Forms::Label^  label14;
private: System::Windows::Forms::Label^  label13;
private: System::Windows::Forms::Label^  lAccZGain;
private: System::Windows::Forms::Label^  lAccYGain;
private: System::Windows::Forms::Label^  lAccXGain;
private: System::Windows::Forms::Label^  lMagZGain;
private: System::Windows::Forms::Label^  lMagYGain;
private: System::Windows::Forms::Label^  lMagXGain;
private: System::Windows::Forms::Label^  label21;
private: System::Windows::Forms::Label^  label22;
private: System::Windows::Forms::PictureBox^  pictureBox1;
private: System::Windows::Forms::Label^  label4;
private: System::Windows::Forms::ProgressBar^  prgAcc;
private: System::Windows::Forms::ProgressBar^  prgMag;
private: System::Windows::Forms::Label^  lblMagDone;
private: System::Windows::Forms::Label^  lblAccelDone;
private: System::Windows::Forms::TextBox^  txtMag;
private: System::Windows::Forms::TextBox^  txtAcc;
	private: System::ComponentModel::IContainer^  components;

System::Void CalWizard::pictureBox1_Paint(System::Object^ sender, System::Windows::Forms::PaintEventArgs^ e);

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			this->mainUpdateTimer = (gcnew System::Windows::Forms::Timer(this->components));
			this->WizardNextBtn = (gcnew System::Windows::Forms::Button());
			this->WizardPrevBtn = (gcnew System::Windows::Forms::Button());
			this->WizardCancelBtn = (gcnew System::Windows::Forms::Button());
			this->WizardInstructionsLbl = (gcnew System::Windows::Forms::Label());
			this->WizardPictureBox = (gcnew System::Windows::Forms::PictureBox());
			this->WizardDataGrp = (gcnew System::Windows::Forms::GroupBox());
			this->lAccZGain = (gcnew System::Windows::Forms::Label());
			this->lAccYGain = (gcnew System::Windows::Forms::Label());
			this->lAccXGain = (gcnew System::Windows::Forms::Label());
			this->lMagZGain = (gcnew System::Windows::Forms::Label());
			this->lMagYGain = (gcnew System::Windows::Forms::Label());
			this->lMagXGain = (gcnew System::Windows::Forms::Label());
			this->label21 = (gcnew System::Windows::Forms::Label());
			this->label22 = (gcnew System::Windows::Forms::Label());
			this->label16 = (gcnew System::Windows::Forms::Label());
			this->label17 = (gcnew System::Windows::Forms::Label());
			this->label18 = (gcnew System::Windows::Forms::Label());
			this->label15 = (gcnew System::Windows::Forms::Label());
			this->label14 = (gcnew System::Windows::Forms::Label());
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->AccMaxZ = (gcnew System::Windows::Forms::Label());
			this->AccMaxY = (gcnew System::Windows::Forms::Label());
			this->AccMaxX = (gcnew System::Windows::Forms::Label());
			this->AccMinZ = (gcnew System::Windows::Forms::Label());
			this->AccMinY = (gcnew System::Windows::Forms::Label());
			this->AccMinX = (gcnew System::Windows::Forms::Label());
			this->MagMaxZ = (gcnew System::Windows::Forms::Label());
			this->MagMaxY = (gcnew System::Windows::Forms::Label());
			this->MagMaxX = (gcnew System::Windows::Forms::Label());
			this->MagMinZ = (gcnew System::Windows::Forms::Label());
			this->MagMinY = (gcnew System::Windows::Forms::Label());
			this->MagMinX = (gcnew System::Windows::Forms::Label());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->AccZOffsetLbl = (gcnew System::Windows::Forms::Label());
			this->AccYOffsetLbl = (gcnew System::Windows::Forms::Label());
			this->AccXOffsetLbl = (gcnew System::Windows::Forms::Label());
			this->MagZOffsetLbl = (gcnew System::Windows::Forms::Label());
			this->MagYOffsetLbl = (gcnew System::Windows::Forms::Label());
			this->MagXOffsetLbl = (gcnew System::Windows::Forms::Label());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->AccCurZLbl = (gcnew System::Windows::Forms::Label());
			this->AccCurYLbl = (gcnew System::Windows::Forms::Label());
			this->AccCurXLbl = (gcnew System::Windows::Forms::Label());
			this->MagCurZLbl = (gcnew System::Windows::Forms::Label());
			this->MagCurYLbl = (gcnew System::Windows::Forms::Label());
			this->MagCurXLbl = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->prgMag = (gcnew System::Windows::Forms::ProgressBar());
			this->prgAcc = (gcnew System::Windows::Forms::ProgressBar());
			this->lblMagDone = (gcnew System::Windows::Forms::Label());
			this->lblAccelDone = (gcnew System::Windows::Forms::Label());
			this->txtMag = (gcnew System::Windows::Forms::TextBox());
			this->txtAcc = (gcnew System::Windows::Forms::TextBox());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->WizardPictureBox))->BeginInit();
			this->WizardDataGrp->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBox1))->BeginInit();
			this->SuspendLayout();
			// 
			// mainUpdateTimer
			// 
			this->mainUpdateTimer->Interval = 50;
			this->mainUpdateTimer->Tick += gcnew System::EventHandler(this, &CalWizard::MainUpdateTimer_Tick);
			// 
			// WizardNextBtn
			// 
			this->WizardNextBtn->Location = System::Drawing::Point(89, 508);
			this->WizardNextBtn->Name = L"WizardNextBtn";
			this->WizardNextBtn->Size = System::Drawing::Size(75, 23);
			this->WizardNextBtn->TabIndex = 0;
			this->WizardNextBtn->Text = L"Next";
			this->WizardNextBtn->UseVisualStyleBackColor = true;
			this->WizardNextBtn->Click += gcnew System::EventHandler(this, &CalWizard::NextBtn_Click);
			// 
			// WizardPrevBtn
			// 
			this->WizardPrevBtn->Location = System::Drawing::Point(0, 508);
			this->WizardPrevBtn->Name = L"WizardPrevBtn";
			this->WizardPrevBtn->Size = System::Drawing::Size(75, 23);
			this->WizardPrevBtn->TabIndex = 1;
			this->WizardPrevBtn->Text = L"Previous";
			this->WizardPrevBtn->UseVisualStyleBackColor = true;
			this->WizardPrevBtn->Click += gcnew System::EventHandler(this, &CalWizard::PreviousBtn_Click);
			// 
			// WizardCancelBtn
			// 
			this->WizardCancelBtn->Location = System::Drawing::Point(273, 508);
			this->WizardCancelBtn->Name = L"WizardCancelBtn";
			this->WizardCancelBtn->Size = System::Drawing::Size(75, 23);
			this->WizardCancelBtn->TabIndex = 2;
			this->WizardCancelBtn->Text = L"Cancel";
			this->WizardCancelBtn->UseVisualStyleBackColor = true;
			this->WizardCancelBtn->Click += gcnew System::EventHandler(this, &CalWizard::CancelBtn_Click);
			// 
			// WizardInstructionsLbl
			// 
			this->WizardInstructionsLbl->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9, System::Drawing::FontStyle::Regular, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->WizardInstructionsLbl->Location = System::Drawing::Point(12, 9);
			this->WizardInstructionsLbl->Name = L"WizardInstructionsLbl";
			this->WizardInstructionsLbl->Size = System::Drawing::Size(336, 93);
			this->WizardInstructionsLbl->TabIndex = 3;
			this->WizardInstructionsLbl->Text = L"Instructions";
			// 
			// WizardPictureBox
			// 
			this->WizardPictureBox->Location = System::Drawing::Point(4, 105);
			this->WizardPictureBox->Name = L"WizardPictureBox";
			this->WizardPictureBox->Size = System::Drawing::Size(348, 211);
			this->WizardPictureBox->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->WizardPictureBox->TabIndex = 4;
			this->WizardPictureBox->TabStop = false;
			// 
			// WizardDataGrp
			// 
			this->WizardDataGrp->Controls->Add(this->prgAcc);
			this->WizardDataGrp->Controls->Add(this->prgMag);
			this->WizardDataGrp->Controls->Add(this->lAccZGain);
			this->WizardDataGrp->Controls->Add(this->lAccYGain);
			this->WizardDataGrp->Controls->Add(this->lAccXGain);
			this->WizardDataGrp->Controls->Add(this->lMagZGain);
			this->WizardDataGrp->Controls->Add(this->lMagYGain);
			this->WizardDataGrp->Controls->Add(this->lMagXGain);
			this->WizardDataGrp->Controls->Add(this->label21);
			this->WizardDataGrp->Controls->Add(this->label22);
			this->WizardDataGrp->Controls->Add(this->label16);
			this->WizardDataGrp->Controls->Add(this->label17);
			this->WizardDataGrp->Controls->Add(this->label18);
			this->WizardDataGrp->Controls->Add(this->label15);
			this->WizardDataGrp->Controls->Add(this->label14);
			this->WizardDataGrp->Controls->Add(this->label13);
			this->WizardDataGrp->Controls->Add(this->AccMaxZ);
			this->WizardDataGrp->Controls->Add(this->AccMaxY);
			this->WizardDataGrp->Controls->Add(this->AccMaxX);
			this->WizardDataGrp->Controls->Add(this->AccMinZ);
			this->WizardDataGrp->Controls->Add(this->AccMinY);
			this->WizardDataGrp->Controls->Add(this->AccMinX);
			this->WizardDataGrp->Controls->Add(this->MagMaxZ);
			this->WizardDataGrp->Controls->Add(this->MagMaxY);
			this->WizardDataGrp->Controls->Add(this->MagMaxX);
			this->WizardDataGrp->Controls->Add(this->MagMinZ);
			this->WizardDataGrp->Controls->Add(this->MagMinY);
			this->WizardDataGrp->Controls->Add(this->MagMinX);
			this->WizardDataGrp->Controls->Add(this->label11);
			this->WizardDataGrp->Controls->Add(this->label12);
			this->WizardDataGrp->Controls->Add(this->label10);
			this->WizardDataGrp->Controls->Add(this->label9);
			this->WizardDataGrp->Controls->Add(this->AccZOffsetLbl);
			this->WizardDataGrp->Controls->Add(this->AccYOffsetLbl);
			this->WizardDataGrp->Controls->Add(this->AccXOffsetLbl);
			this->WizardDataGrp->Controls->Add(this->MagZOffsetLbl);
			this->WizardDataGrp->Controls->Add(this->MagYOffsetLbl);
			this->WizardDataGrp->Controls->Add(this->MagXOffsetLbl);
			this->WizardDataGrp->Controls->Add(this->label8);
			this->WizardDataGrp->Controls->Add(this->label3);
			this->WizardDataGrp->Controls->Add(this->AccCurZLbl);
			this->WizardDataGrp->Controls->Add(this->AccCurYLbl);
			this->WizardDataGrp->Controls->Add(this->AccCurXLbl);
			this->WizardDataGrp->Controls->Add(this->MagCurZLbl);
			this->WizardDataGrp->Controls->Add(this->MagCurYLbl);
			this->WizardDataGrp->Controls->Add(this->MagCurXLbl);
			this->WizardDataGrp->Controls->Add(this->label2);
			this->WizardDataGrp->Controls->Add(this->label1);
			this->WizardDataGrp->Location = System::Drawing::Point(4, 322);
			this->WizardDataGrp->Name = L"WizardDataGrp";
			this->WizardDataGrp->Size = System::Drawing::Size(348, 180);
			this->WizardDataGrp->TabIndex = 5;
			this->WizardDataGrp->TabStop = false;
			this->WizardDataGrp->Text = L"Calibration Data";
			// 
			// lAccZGain
			// 
			this->lAccZGain->AutoSize = true;
			this->lAccZGain->Location = System::Drawing::Point(302, 127);
			this->lAccZGain->Name = L"lAccZGain";
			this->lAccZGain->Size = System::Drawing::Size(13, 13);
			this->lAccZGain->TabIndex = 49;
			this->lAccZGain->Text = L"0";
			this->lAccZGain->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// lAccYGain
			// 
			this->lAccYGain->AutoSize = true;
			this->lAccYGain->Location = System::Drawing::Point(272, 127);
			this->lAccYGain->Name = L"lAccYGain";
			this->lAccYGain->Size = System::Drawing::Size(13, 13);
			this->lAccYGain->TabIndex = 48;
			this->lAccYGain->Text = L"0";
			this->lAccYGain->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// lAccXGain
			// 
			this->lAccXGain->AutoSize = true;
			this->lAccXGain->Location = System::Drawing::Point(240, 127);
			this->lAccXGain->Name = L"lAccXGain";
			this->lAccXGain->Size = System::Drawing::Size(13, 13);
			this->lAccXGain->TabIndex = 47;
			this->lAccXGain->Text = L"0";
			this->lAccXGain->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// lMagZGain
			// 
			this->lMagZGain->AutoSize = true;
			this->lMagZGain->Location = System::Drawing::Point(133, 127);
			this->lMagZGain->Name = L"lMagZGain";
			this->lMagZGain->Size = System::Drawing::Size(13, 13);
			this->lMagZGain->TabIndex = 46;
			this->lMagZGain->Text = L"0";
			this->lMagZGain->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// lMagYGain
			// 
			this->lMagYGain->AutoSize = true;
			this->lMagYGain->Location = System::Drawing::Point(102, 127);
			this->lMagYGain->Name = L"lMagYGain";
			this->lMagYGain->Size = System::Drawing::Size(13, 13);
			this->lMagYGain->TabIndex = 45;
			this->lMagYGain->Text = L"0";
			this->lMagYGain->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// lMagXGain
			// 
			this->lMagXGain->AutoSize = true;
			this->lMagXGain->Location = System::Drawing::Point(71, 127);
			this->lMagXGain->Name = L"lMagXGain";
			this->lMagXGain->Size = System::Drawing::Size(13, 13);
			this->lMagXGain->TabIndex = 44;
			this->lMagXGain->Text = L"0";
			this->lMagXGain->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// label21
			// 
			this->label21->AutoSize = true;
			this->label21->Location = System::Drawing::Point(187, 127);
			this->label21->Name = L"label21";
			this->label21->Size = System::Drawing::Size(32, 13);
			this->label21->TabIndex = 43;
			this->label21->Text = L"Gain:";
			// 
			// label22
			// 
			this->label22->AutoSize = true;
			this->label22->Location = System::Drawing::Point(16, 127);
			this->label22->Name = L"label22";
			this->label22->Size = System::Drawing::Size(32, 13);
			this->label22->TabIndex = 42;
			this->label22->Text = L"Gain:";
			// 
			// label16
			// 
			this->label16->AutoSize = true;
			this->label16->Location = System::Drawing::Point(302, 16);
			this->label16->Name = L"label16";
			this->label16->Size = System::Drawing::Size(14, 13);
			this->label16->TabIndex = 41;
			this->label16->Text = L"Z";
			// 
			// label17
			// 
			this->label17->AutoSize = true;
			this->label17->Location = System::Drawing::Point(271, 16);
			this->label17->Name = L"label17";
			this->label17->Size = System::Drawing::Size(14, 13);
			this->label17->TabIndex = 40;
			this->label17->Text = L"Y";
			// 
			// label18
			// 
			this->label18->AutoSize = true;
			this->label18->Location = System::Drawing::Point(241, 16);
			this->label18->Name = L"label18";
			this->label18->Size = System::Drawing::Size(14, 13);
			this->label18->TabIndex = 39;
			this->label18->Text = L"X";
			// 
			// label15
			// 
			this->label15->AutoSize = true;
			this->label15->Location = System::Drawing::Point(133, 16);
			this->label15->Name = L"label15";
			this->label15->Size = System::Drawing::Size(14, 13);
			this->label15->TabIndex = 38;
			this->label15->Text = L"Z";
			// 
			// label14
			// 
			this->label14->AutoSize = true;
			this->label14->Location = System::Drawing::Point(102, 16);
			this->label14->Name = L"label14";
			this->label14->Size = System::Drawing::Size(14, 13);
			this->label14->TabIndex = 37;
			this->label14->Text = L"Y";
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->Location = System::Drawing::Point(72, 16);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(14, 13);
			this->label13->TabIndex = 36;
			this->label13->Text = L"X";
			// 
			// AccMaxZ
			// 
			this->AccMaxZ->AutoSize = true;
			this->AccMaxZ->Location = System::Drawing::Point(302, 79);
			this->AccMaxZ->Name = L"AccMaxZ";
			this->AccMaxZ->Size = System::Drawing::Size(13, 13);
			this->AccMaxZ->TabIndex = 35;
			this->AccMaxZ->Text = L"0";
			this->AccMaxZ->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// AccMaxY
			// 
			this->AccMaxY->AutoSize = true;
			this->AccMaxY->Location = System::Drawing::Point(272, 79);
			this->AccMaxY->Name = L"AccMaxY";
			this->AccMaxY->Size = System::Drawing::Size(13, 13);
			this->AccMaxY->TabIndex = 34;
			this->AccMaxY->Text = L"0";
			this->AccMaxY->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// AccMaxX
			// 
			this->AccMaxX->AutoSize = true;
			this->AccMaxX->Location = System::Drawing::Point(240, 79);
			this->AccMaxX->Name = L"AccMaxX";
			this->AccMaxX->Size = System::Drawing::Size(13, 13);
			this->AccMaxX->TabIndex = 33;
			this->AccMaxX->Text = L"0";
			this->AccMaxX->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// AccMinZ
			// 
			this->AccMinZ->AutoSize = true;
			this->AccMinZ->Location = System::Drawing::Point(302, 56);
			this->AccMinZ->Name = L"AccMinZ";
			this->AccMinZ->Size = System::Drawing::Size(13, 13);
			this->AccMinZ->TabIndex = 32;
			this->AccMinZ->Text = L"0";
			this->AccMinZ->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// AccMinY
			// 
			this->AccMinY->AutoSize = true;
			this->AccMinY->Location = System::Drawing::Point(272, 56);
			this->AccMinY->Name = L"AccMinY";
			this->AccMinY->Size = System::Drawing::Size(13, 13);
			this->AccMinY->TabIndex = 31;
			this->AccMinY->Text = L"0";
			this->AccMinY->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// AccMinX
			// 
			this->AccMinX->AutoSize = true;
			this->AccMinX->Location = System::Drawing::Point(240, 56);
			this->AccMinX->Name = L"AccMinX";
			this->AccMinX->Size = System::Drawing::Size(13, 13);
			this->AccMinX->TabIndex = 30;
			this->AccMinX->Text = L"0";
			this->AccMinX->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// MagMaxZ
			// 
			this->MagMaxZ->AutoSize = true;
			this->MagMaxZ->Location = System::Drawing::Point(133, 79);
			this->MagMaxZ->Name = L"MagMaxZ";
			this->MagMaxZ->Size = System::Drawing::Size(13, 13);
			this->MagMaxZ->TabIndex = 29;
			this->MagMaxZ->Text = L"0";
			this->MagMaxZ->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// MagMaxY
			// 
			this->MagMaxY->AutoSize = true;
			this->MagMaxY->Location = System::Drawing::Point(102, 79);
			this->MagMaxY->Name = L"MagMaxY";
			this->MagMaxY->Size = System::Drawing::Size(13, 13);
			this->MagMaxY->TabIndex = 28;
			this->MagMaxY->Text = L"0";
			this->MagMaxY->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// MagMaxX
			// 
			this->MagMaxX->AutoSize = true;
			this->MagMaxX->Location = System::Drawing::Point(71, 79);
			this->MagMaxX->Name = L"MagMaxX";
			this->MagMaxX->Size = System::Drawing::Size(13, 13);
			this->MagMaxX->TabIndex = 27;
			this->MagMaxX->Text = L"0";
			this->MagMaxX->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// MagMinZ
			// 
			this->MagMinZ->AutoSize = true;
			this->MagMinZ->Location = System::Drawing::Point(133, 56);
			this->MagMinZ->Name = L"MagMinZ";
			this->MagMinZ->Size = System::Drawing::Size(13, 13);
			this->MagMinZ->TabIndex = 26;
			this->MagMinZ->Text = L"0";
			this->MagMinZ->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// MagMinY
			// 
			this->MagMinY->AutoSize = true;
			this->MagMinY->Location = System::Drawing::Point(102, 56);
			this->MagMinY->Name = L"MagMinY";
			this->MagMinY->Size = System::Drawing::Size(13, 13);
			this->MagMinY->TabIndex = 25;
			this->MagMinY->Text = L"0";
			// 
			// MagMinX
			// 
			this->MagMinX->AutoSize = true;
			this->MagMinX->Location = System::Drawing::Point(71, 56);
			this->MagMinX->Name = L"MagMinX";
			this->MagMinX->Size = System::Drawing::Size(13, 13);
			this->MagMinX->TabIndex = 24;
			this->MagMinX->Text = L"0";
			this->MagMinX->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->Location = System::Drawing::Point(201, 79);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(27, 13);
			this->label11->TabIndex = 23;
			this->label11->Text = L"Max";
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->Location = System::Drawing::Point(201, 56);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(24, 13);
			this->label12->TabIndex = 22;
			this->label12->Text = L"Min";
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->Location = System::Drawing::Point(30, 79);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(27, 13);
			this->label10->TabIndex = 21;
			this->label10->Text = L"Max";
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Location = System::Drawing::Point(30, 56);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(24, 13);
			this->label9->TabIndex = 20;
			this->label9->Text = L"Min";
			// 
			// AccZOffsetLbl
			// 
			this->AccZOffsetLbl->AutoSize = true;
			this->AccZOffsetLbl->Location = System::Drawing::Point(302, 105);
			this->AccZOffsetLbl->Name = L"AccZOffsetLbl";
			this->AccZOffsetLbl->Size = System::Drawing::Size(13, 13);
			this->AccZOffsetLbl->TabIndex = 19;
			this->AccZOffsetLbl->Text = L"0";
			this->AccZOffsetLbl->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// AccYOffsetLbl
			// 
			this->AccYOffsetLbl->AutoSize = true;
			this->AccYOffsetLbl->Location = System::Drawing::Point(272, 105);
			this->AccYOffsetLbl->Name = L"AccYOffsetLbl";
			this->AccYOffsetLbl->Size = System::Drawing::Size(13, 13);
			this->AccYOffsetLbl->TabIndex = 18;
			this->AccYOffsetLbl->Text = L"0";
			this->AccYOffsetLbl->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// AccXOffsetLbl
			// 
			this->AccXOffsetLbl->AutoSize = true;
			this->AccXOffsetLbl->Location = System::Drawing::Point(240, 105);
			this->AccXOffsetLbl->Name = L"AccXOffsetLbl";
			this->AccXOffsetLbl->Size = System::Drawing::Size(13, 13);
			this->AccXOffsetLbl->TabIndex = 17;
			this->AccXOffsetLbl->Text = L"0";
			this->AccXOffsetLbl->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// MagZOffsetLbl
			// 
			this->MagZOffsetLbl->AutoSize = true;
			this->MagZOffsetLbl->Location = System::Drawing::Point(133, 105);
			this->MagZOffsetLbl->Name = L"MagZOffsetLbl";
			this->MagZOffsetLbl->Size = System::Drawing::Size(13, 13);
			this->MagZOffsetLbl->TabIndex = 16;
			this->MagZOffsetLbl->Text = L"0";
			this->MagZOffsetLbl->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// MagYOffsetLbl
			// 
			this->MagYOffsetLbl->AutoSize = true;
			this->MagYOffsetLbl->Location = System::Drawing::Point(102, 105);
			this->MagYOffsetLbl->Name = L"MagYOffsetLbl";
			this->MagYOffsetLbl->Size = System::Drawing::Size(13, 13);
			this->MagYOffsetLbl->TabIndex = 15;
			this->MagYOffsetLbl->Text = L"0";
			this->MagYOffsetLbl->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// MagXOffsetLbl
			// 
			this->MagXOffsetLbl->AutoSize = true;
			this->MagXOffsetLbl->Location = System::Drawing::Point(71, 105);
			this->MagXOffsetLbl->Name = L"MagXOffsetLbl";
			this->MagXOffsetLbl->Size = System::Drawing::Size(13, 13);
			this->MagXOffsetLbl->TabIndex = 14;
			this->MagXOffsetLbl->Text = L"0";
			this->MagXOffsetLbl->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Location = System::Drawing::Point(187, 105);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(38, 13);
			this->label8->TabIndex = 11;
			this->label8->Text = L"Offset:";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(16, 105);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(38, 13);
			this->label3->TabIndex = 8;
			this->label3->Text = L"Offset:";
			// 
			// AccCurZLbl
			// 
			this->AccCurZLbl->AutoSize = true;
			this->AccCurZLbl->Location = System::Drawing::Point(302, 35);
			this->AccCurZLbl->Name = L"AccCurZLbl";
			this->AccCurZLbl->Size = System::Drawing::Size(13, 13);
			this->AccCurZLbl->TabIndex = 7;
			this->AccCurZLbl->Text = L"0";
			this->AccCurZLbl->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// AccCurYLbl
			// 
			this->AccCurYLbl->AutoSize = true;
			this->AccCurYLbl->Location = System::Drawing::Point(272, 35);
			this->AccCurYLbl->Name = L"AccCurYLbl";
			this->AccCurYLbl->Size = System::Drawing::Size(13, 13);
			this->AccCurYLbl->TabIndex = 6;
			this->AccCurYLbl->Text = L"0";
			this->AccCurYLbl->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// AccCurXLbl
			// 
			this->AccCurXLbl->AutoSize = true;
			this->AccCurXLbl->Location = System::Drawing::Point(240, 35);
			this->AccCurXLbl->Name = L"AccCurXLbl";
			this->AccCurXLbl->Size = System::Drawing::Size(13, 13);
			this->AccCurXLbl->TabIndex = 5;
			this->AccCurXLbl->Text = L"0";
			this->AccCurXLbl->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// MagCurZLbl
			// 
			this->MagCurZLbl->AutoSize = true;
			this->MagCurZLbl->Location = System::Drawing::Point(133, 35);
			this->MagCurZLbl->Name = L"MagCurZLbl";
			this->MagCurZLbl->Size = System::Drawing::Size(13, 13);
			this->MagCurZLbl->TabIndex = 4;
			this->MagCurZLbl->Text = L"0";
			this->MagCurZLbl->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// MagCurYLbl
			// 
			this->MagCurYLbl->AutoSize = true;
			this->MagCurYLbl->Location = System::Drawing::Point(102, 35);
			this->MagCurYLbl->Name = L"MagCurYLbl";
			this->MagCurYLbl->Size = System::Drawing::Size(13, 13);
			this->MagCurYLbl->TabIndex = 3;
			this->MagCurYLbl->Text = L"0";
			this->MagCurYLbl->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// MagCurXLbl
			// 
			this->MagCurXLbl->AutoSize = true;
			this->MagCurXLbl->Location = System::Drawing::Point(71, 35);
			this->MagCurXLbl->Name = L"MagCurXLbl";
			this->MagCurXLbl->Size = System::Drawing::Size(13, 13);
			this->MagCurXLbl->TabIndex = 2;
			this->MagCurXLbl->Text = L"0";
			this->MagCurXLbl->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label2->Location = System::Drawing::Point(168, 35);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(70, 13);
			this->label2->TabIndex = 1;
			this->label2->Text = L"Accel Data";
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label1->Location = System::Drawing::Point(3, 35);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(62, 13);
			this->label1->TabIndex = 0;
			this->label1->Text = L"Mag Data";
			// 
			// pictureBox1
			// 
			this->pictureBox1->BackColor = System::Drawing::Color::White;
			this->pictureBox1->Location = System::Drawing::Point(356, 106);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(360, 360);
			this->pictureBox1->TabIndex = 6;
			this->pictureBox1->TabStop = false;
			this->pictureBox1->Paint += gcnew System::Windows::Forms::PaintEventHandler(this, &CalWizard::pictureBox1_Paint);
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(366, 89);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(210, 13);
			this->label4->TabIndex = 7;
			this->label4->Text = L"Blue points - accelerometer,  red - compass";
			this->label4->Click += gcnew System::EventHandler(this, &CalWizard::label4_Click);
			// 
			// prgMag
			// 
			this->prgMag->Location = System::Drawing::Point(10, 150);
			this->prgMag->Name = L"prgMag";
			this->prgMag->Size = System::Drawing::Size(149, 19);
			this->prgMag->Step = 11;
			this->prgMag->TabIndex = 50;
			this->prgMag->Click += gcnew System::EventHandler(this, &CalWizard::prgMag_Click);
			// 
			// prgAcc
			// 
			this->prgAcc->Location = System::Drawing::Point(180, 150);
			this->prgAcc->Name = L"prgAcc";
			this->prgAcc->Size = System::Drawing::Size(149, 19);
			this->prgAcc->Step = 11;
			this->prgAcc->TabIndex = 51;
			// 
			// lblMagDone
			// 
			this->lblMagDone->AutoSize = true;
			this->lblMagDone->Location = System::Drawing::Point(374, 485);
			this->lblMagDone->Name = L"lblMagDone";
			this->lblMagDone->Size = System::Drawing::Size(27, 13);
			this->lblMagDone->TabIndex = 8;
			this->lblMagDone->Text = L"mag";
			// 
			// lblAccelDone
			// 
			this->lblAccelDone->AutoSize = true;
			this->lblAccelDone->Location = System::Drawing::Point(543, 485);
			this->lblAccelDone->Name = L"lblAccelDone";
			this->lblAccelDone->Size = System::Drawing::Size(33, 13);
			this->lblAccelDone->TabIndex = 9;
			this->lblAccelDone->Text = L"accel";
			// 
			// txtMag
			// 
			this->txtMag->Location = System::Drawing::Point(408, 484);
			this->txtMag->Name = L"txtMag";
			this->txtMag->Size = System::Drawing::Size(124, 20);
			this->txtMag->TabIndex = 10;
			// 
			// txtAcc
			// 
			this->txtAcc->Location = System::Drawing::Point(582, 482);
			this->txtAcc->Name = L"txtAcc";
			this->txtAcc->Size = System::Drawing::Size(115, 20);
			this->txtAcc->TabIndex = 11;
			// 
			// CalWizard
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(739, 534);
			this->Controls->Add(this->txtAcc);
			this->Controls->Add(this->txtMag);
			this->Controls->Add(this->lblAccelDone);
			this->Controls->Add(this->lblMagDone);
			this->Controls->Add(this->label4);
			this->Controls->Add(this->pictureBox1);
			this->Controls->Add(this->WizardDataGrp);
			this->Controls->Add(this->WizardPictureBox);
			this->Controls->Add(this->WizardInstructionsLbl);
			this->Controls->Add(this->WizardCancelBtn);
			this->Controls->Add(this->WizardPrevBtn);
			this->Controls->Add(this->WizardNextBtn);
			this->Name = L"CalWizard";
			this->Text = L"Headtracker Calibration Wizard";
			this->WindowState = System::Windows::Forms::FormWindowState::Maximized;
			this->FormClosed += gcnew System::Windows::Forms::FormClosedEventHandler(this, &CalWizard::CalWizard_OnClose);
			this->Load += gcnew System::EventHandler(this, &CalWizard::CalWizard_Load);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->WizardPictureBox))->EndInit();
			this->WizardDataGrp->ResumeLayout(false);
			this->WizardDataGrp->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBox1))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion

public: // Methods

    System::Void Begin(CALMODE mode);

private: // Vars

    System::Resources::ResourceManager^ rm;
    int CurrentStep;    // The current wizard page (0 = intro)

    int serialbyte;
    double* SerialData;
	System::String^ SerialLine;
	System::String^ DigitLine;
    int Serialindex;
    
    // Mag
    //
    double MagXOffset, MagYOffset, MagZOffset;
	double MagXgain,  MagYgain, MagZgain;

    double MagXmax; 
    double MagXmin;
    double MagYmax; 
    double MagYmin; 
    double MagZmax; 
    double MagZmin; 

    double MagXReading;
    double MagYReading;
    double MagZReading;

    // Accel
    //
    double AccXOffset, AccYOffset, AccZOffset;
    double AccXgain, AccYgain, AccZgain;

    double AccXMin;
    double AccXMax;
    double AccYMin;
    double AccYMax;
    double AccZMin;
    double AccZMax;

    double AccXReading;
    double AccYReading;
    double AccZReading; 

    CALMODE CalMode;
    bool TrackMinMax;

//	Vector3f *AccData, *MagData;

	List<Vector3d> ^AccData; //= gcnew array<Vector3d ^>();
	List<Vector3d> ^MagData;

	int dataCount;

	
private:

    System::Void CalWizard_Load(System::Object^  sender, System::EventArgs^  e);
    System::Void CalWizard_OnClose(System::Object^  sender, System::Windows::Forms::FormClosedEventArgs^  e);
    System::Void MainUpdateTimer_Tick(System::Object^  sender, System::EventArgs^  e);
    System::Void PreviousBtn_Click(System::Object^  sender, System::EventArgs^  e);
    System::Void NextBtn_Click(System::Object^  sender, System::EventArgs^  e);
    System::Void CancelBtn_Click(System::Object^  sender, System::EventArgs^  e);
    System::Void UpdateCalc();
    System::Void ZeroMinMaxTracking();
    System::Void SetupUIForStep(int Step);
    System::Void CompleteStep(int Step);
    System::Void SetupAxisCalStepUI(int Step);
    System::Void SetupTrackingCalStepUI(int Step);
    System::Void CompleteAxisCalStep(int Step);
    System::Void CompleteTrackingCalStep(int Step);
	System::Void NewPoint(double ax,double ay,double az,double mx,double my,double mz);
	System::Void updateStatus(System::Windows::Forms::ProgressBar^  progress, CompassCalibrator &calibrator, System::Windows::Forms::TextBox^ txt);



private: System::Void label4_Click(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void prgMag_Click(System::Object^  sender, System::EventArgs^  e) {
		 }
};

} // namespace
