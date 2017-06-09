//--------------------------------------------------------------------------------------
// File:    CalWizard.cpp (C++/CLI)
// Desc:    Implements the calibration wizard UI.
//
// Hist:    4/15/13 - Mark Mansur - initial creation.
//--------------------------------------------------------------------------------------
/*
    Wizard steps:
    0: Welcome/overview
    1: IMU is upright, 0 degrees. Mag X/Y values, gyro cal, accel Z max
    2: IMU is upright, panned 180 degrees. Mag X/Y/Z values
    3: IMU tilt forward 90 degrees. Accel X max
    4: IMU tilt backward -90 degrees. Accel X min
    5: IMU roll left -90 degrees. Accel Y max
    6: IMU roll right +90 degrees. Accel Y min
    7: IMU upside down, panned 0 degrees. mag Z value, accel Z min
    8: Save calibration
    9: Complete!
*/

#include "StdAfx.h"
#include "CalWizard.h"
#include "HeadTrackerHardware.h"
#include <math.h>



using namespace HeadTrackerGUI;
using namespace System::Resources;
using namespace System::Drawing;

CompassCalibrator	mag_calibrator;
CompassCalibrator	acc_calibrator;

//--------------------------------------------------------------------------------------
// Func: CalWizard_Load
// Desc: Handler called by system when the wizard is first loaded into memory.
//--------------------------------------------------------------------------------------
System::Void CalWizard::CalWizard_Load(System::Object^ sender, System::EventArgs^ e)
{
    mainUpdateTimer->Enabled = false;
    Begin(CalMode);
	
	pictureBox1->Refresh();
}

System::Void CalWizard::CalWizard_OnClose(System::Object^  sender, System::Windows::Forms::FormClosedEventArgs^  e)
{
    TrackerWrap::Tracker->StreamMagAccelData(false);
}

System::Void CalWizard::Begin(CALMODE mode)
{
    CalMode = mode;
    SetupUIForStep(0);
}

extern System::Double myConvert(System::String^ s);

//--------------------------------------------------------------------------------------
// Func: MainUpdateTimer_Tick
// Desc: Handler for the serial update timer. This handler reads any new data from the
//       serial driver's FIFO buffer, converts, and stores the results.
//--------------------------------------------------------------------------------------
System::Void CalWizard::MainUpdateTimer_Tick(System::Object^  sender, System::EventArgs^  e){
	byte lastByte;
    if (TrackerWrap::Tracker->Port->IsOpen)  {

		SerialLine="";
		DigitLine="";

		while (TrackerWrap::Tracker->Port->BytesToRead > 0)  {
			lastByte=serialbyte;

            serialbyte = TrackerWrap::Tracker->Port->ReadChar();

			if (serialbyte == 10) serialbyte = 13;
            if (serialbyte == 13) { // EOL
				if (lastByte == 13) {
					continue;			
				}

				Debug::WriteLine(SerialLine);
				if(SerialLine=="" || SerialLine[0]=='#') {
					SerialLine="";
					DigitLine="";
					Serialindex = 0;
					continue;
				}

	            if (TrackerWrap::Tracker->MagAccelStreaming) {
					array<String^>^ line = DigitLine->Split(',');

                    if ( line->Length >= 6 )  {
                        

						try {
							MagXReading = myConvert(line[0]); MagYReading = myConvert(line[1]);	MagZReading = myConvert(line[2]);
							AccXReading = myConvert(line[3]); AccYReading = myConvert(line[4]);	AccZReading = myConvert(line[5]);

							MagCurXLbl->Text = Convert::ToString(MagXReading);
							MagCurYLbl->Text = Convert::ToString(MagYReading);
							MagCurZLbl->Text = Convert::ToString(MagZReading);
							AccCurXLbl->Text = Convert::ToString(AccXReading);
							AccCurYLbl->Text = Convert::ToString(AccYReading);
							AccCurZLbl->Text = Convert::ToString(AccZReading);

							NewPoint(MagXReading,MagYReading,MagZReading, AccXReading,AccYReading,AccZReading);

							if ( CalMode == MINMAXTRACKING && TrackMinMax )
		                        UpdateCalc();

						} catch(System::Exception^ e){
							Debug::WriteLine("conversion error! "+e->Message);
						
						}

                    }
				}
				SerialLine="";
				DigitLine="";

                Serialindex = 0;
            } else{
				SerialLine += Convert::ToChar((serialbyte));
				if(serialbyte <='9')
					DigitLine += Convert::ToChar((serialbyte));
            }
            
        } // while serial data is pulled in
    } // if port is open
}

//--------------------------------------------------------------------------------------
// Func: PreviousBtn_Click
// Desc: Previous button click handler.
//--------------------------------------------------------------------------------------
System::Void CalWizard::PreviousBtn_Click(System::Object^  sender, System::EventArgs^  e)
{
    CurrentStep--;
    SetupUIForStep(CurrentStep);
    
}

//--------------------------------------------------------------------------------------
// Func: NextBtn_Click
// Desc: Next button click handler.
//--------------------------------------------------------------------------------------
System::Void CalWizard::NextBtn_Click(System::Object^  sender, System::EventArgs^  e)
{
    CompleteStep(CurrentStep++);
    SetupUIForStep(CurrentStep);
}

//--------------------------------------------------------------------------------------
// Func: CancelBtn_Click
// Desc: Cancel button click handler.
//--------------------------------------------------------------------------------------
System::Void CalWizard::CancelBtn_Click(System::Object^  sender, System::EventArgs^  e)
{
    TrackerWrap::Tracker->StreamMagAccelData(false);
    mainUpdateTimer->Enabled = false;
    Close();
}

//--------------------------------------------------------------------------------------
// Func: UpdateCalc
// Desc: Updates the calibration calculations and updates the UI to reflect new values.
//--------------------------------------------------------------------------------------
System::Void CalWizard::UpdateCalc()
{
    switch ( CalMode )
    {
    case AXISALIGNMENT:
        break;

    case MINMAXTRACKING:
        if ( MagXReading > MagXmax )
            MagXmax = MagXReading;
        if ( MagXReading < MagXmin )
            MagXmin = MagXReading;

        if ( MagYReading > MagYmax )
            MagYmax = MagYReading;
        if ( MagYReading < MagYmin )
            MagYmin = MagYReading;

        if ( MagZReading > MagZmax )
            MagZmax = MagZReading;
        if ( MagZReading < MagZmin )
            MagZmin = MagZReading;

        if ( AccXReading > AccXMax )
            AccXMax = AccXReading;
        if ( AccXReading < AccXMin )
            AccXMin = AccXReading;

        if ( AccYReading > AccYMax )
            AccYMax = AccYReading;
        if ( AccYReading < AccYMin )
            AccYMin = AccYReading;

        if ( AccZReading > AccZMax )
            AccZMax = AccZReading;
        if ( AccZReading < AccZMin )
            AccZMin = AccZReading;

		MagMaxX->Text = Convert::ToString(MagXmax);
		MagMaxY->Text = Convert::ToString(MagYmax);
		MagMaxZ->Text = Convert::ToString(MagZmax);
		MagMinX->Text = Convert::ToString(MagXmin);
		MagMinY->Text = Convert::ToString(MagYmin);
		MagMinZ->Text = Convert::ToString(MagZmin);

		AccMaxX->Text = Convert::ToString(AccXMax);
		AccMaxY->Text = Convert::ToString(AccYMax);
		AccMaxZ->Text = Convert::ToString(AccZMax);
		AccMinX->Text = Convert::ToString(AccXMin);
		AccMinY->Text = Convert::ToString(AccYMin);
		AccMinZ->Text = Convert::ToString(AccZMin);


        break;
    }

    MagXOffset = (MagXmax + MagXmin) * 0.5; 
    MagYOffset = (MagYmax + MagYmin) * 0.5;
    MagZOffset = (MagZmax + MagZmin) * 0.5;

    AccXOffset = (AccXMax + AccXMin) * -0.5;
    AccYOffset = (AccYMax + AccYMin) * -0.5;
    AccZOffset = (AccZMax + AccZMin) * -0.5;

	// диапазон изменения
	MagXgain = (MagXmax - MagXmin)/2; 
    MagYgain = (MagYmax - MagYmin)/2;
    MagZgain = (MagZmax - MagZmin)/2;
	MagXgain /= MagZgain; // normalize to Z
	MagYgain /= MagZgain;
	MagZgain  = 1;

    AccXgain = (AccXMax - AccXMin)/2;
    AccYgain = (AccYMax - AccYMin)/2;
    AccZgain = (AccZMax - AccZMin)/2;
	AccXgain /= AccZgain; // normalize to z
	AccYgain /= AccZgain;
	AccZgain  = 1;

    MagXOffsetLbl->Text = Convert::ToString(MagXOffset);
    MagYOffsetLbl->Text = Convert::ToString(MagYOffset);
    MagZOffsetLbl->Text = Convert::ToString(MagZOffset);

    AccXOffsetLbl->Text = Convert::ToString(AccXOffset);
    AccYOffsetLbl->Text = Convert::ToString(AccYOffset);
    AccZOffsetLbl->Text = Convert::ToString(AccZOffset);

    lMagXGain->Text = Convert::ToString(MagXgain);
    lMagYGain->Text = Convert::ToString(MagYgain);
    lMagZGain->Text = Convert::ToString(MagZgain);

    lAccXGain->Text = Convert::ToString(AccXgain);
    lAccYGain->Text = Convert::ToString(AccYgain);
    lAccZGain->Text = Convert::ToString(AccZgain);

}

//--------------------------------------------------------------------------------------
// Func: ZeroMinMaxTracking
// Desc: Zeros the min/max tracking variables
//--------------------------------------------------------------------------------------
System::Void CalWizard::ZeroMinMaxTracking()
{
    MagXmax = 0;
    MagXmin = 0;
    MagYmax = 0;
    MagYmin = 0;
    MagZmax = 0;
    MagZmin = 0;

    MagXOffset = 0;
    MagYOffset = 0;
    MagZOffset = 0;

	MagXgain = 1;
    MagYgain = 1;
    MagZgain = 1;



    AccXMin = 0;
    AccXMax = 0;
    AccYMin = 0;
    AccYMax = 0;
    AccZMin = 0;
    AccZMax = 0;

    AccXOffset = 0;
    AccYOffset = 0;
    AccZOffset = 0;

	AccXgain = 1;
    AccYgain = 1;
    AccZgain = 1;

}

//--------------------------------------------------------------------------------------
// Func: CompleteStep
// Desc: Gathers the necessary info and performs necessary processing from the user's
//       completion of the specified step. Used to gather sensor data when the user
//       presses the next button.
//--------------------------------------------------------------------------------------
System::Void CalWizard::CompleteStep(int Step)
{
    switch ( CalMode )
    {
    case AXISALIGNMENT:
        CompleteAxisCalStep(Step);
        break;
    case MINMAXTRACKING:
        CompleteTrackingCalStep(Step);
        break;
    }
}

//--------------------------------------------------------------------------------------
// Func: SetupUIForStep
// Desc: Initializes the UI to display to the user for the specified step.
//--------------------------------------------------------------------------------------
System::Void CalWizard::SetupUIForStep(int Step)
{
    switch ( CalMode )
    {
    case AXISALIGNMENT:
        SetupAxisCalStepUI(Step);
        break;
    case MINMAXTRACKING:
        SetupTrackingCalStepUI(Step);
        break;
    }
}

//--------------------------------------------------------------------------------------
// Func: 
// Desc: 
//--------------------------------------------------------------------------------------
System::Void CalWizard::SetupAxisCalStepUI(int Step)
{
    ResourceManager^ rm = gcnew ResourceManager("HeadTrackerGUI.Resources", GetType()->Assembly);
    switch ( Step )
    {
    case 0:
        {
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWiz0");
        }
        break;
    case 1:
        {
            WizardPictureBox->Image = (System::Drawing::Image^)rm->GetObject(L"Position1");
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWiz1");
        }
        break;
    case 2:
        {
            WizardPictureBox->Image = (System::Drawing::Image^)rm->GetObject(L"Position2");
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWiz2");
        }
        break;
    case 3:
        {
            WizardPictureBox->Image = (System::Drawing::Image^)rm->GetObject(L"Position3");
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWiz3");
        }
        break;
    case 4:
        {
            WizardPictureBox->Image = (System::Drawing::Image^)rm->GetObject(L"Position4");
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWiz4");
        }
        break;
    case 5:
        {
            WizardPictureBox->Image = (System::Drawing::Image^)rm->GetObject(L"Position5");
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWiz5");
        }
        break;
    case 6:
        {
            WizardPictureBox->Image = (System::Drawing::Image^)rm->GetObject(L"Position6");
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWiz6");
        }
        break;
    case 7:
        {
            WizardPictureBox->Image = (System::Drawing::Image^)rm->GetObject(L"Position7");
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWiz7");
        }
        break;
    case 8:
        {
            delete WizardPictureBox->Image;
            WizardPictureBox->Image = nullptr;
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWiz8");
        }
        break;
    case 9:
        {
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWiz9");
            WizardNextBtn->Text = "Finish";
            WizardPrevBtn->Enabled = false;
            WizardCancelBtn->Enabled = false;
        }
        break;
    }

    if ( Step == 0 )
    {
        WizardPrevBtn->Enabled = false;
    }
    else if ( Step != 9 )
    {
        WizardPrevBtn->Enabled = true;
    }

    if ( Step != 9 )
    {
        WizardNextBtn->Text = "Next";
    }
}

//--------------------------------------------------------------------------------------
// Func: 
// Desc: 
//--------------------------------------------------------------------------------------
System::Void CalWizard::SetupTrackingCalStepUI(int Step)
{
    ResourceManager^ rm = gcnew ResourceManager("HeadTrackerGUI.Resources", GetType()->Assembly);
    switch ( Step )
    {
    case 0:
        {
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWizTracking0");
        }
        break;
    case 1:
        {
            WizardPictureBox->Image = (System::Drawing::Image^)rm->GetObject(L"Position1");
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWizTracking1");
        }
        break;
    case 2:
        {
            WizardPictureBox->Image = (System::Drawing::Image^)rm->GetObject(L"Position2MinMax");
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWizTracking2");
        }
        break;
    case 3:
        {
            WizardPictureBox->Image = (System::Drawing::Image^)rm->GetObject(L"Position3MinMax");
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWizTracking3");
        }
        break;
    case 4:
        {
            WizardPictureBox->Image = (System::Drawing::Image^)rm->GetObject(L"Position4MinMax");
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWizTracking4");
        }
        break;
    case 5:
        {
            WizardPictureBox->Image = nullptr;
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWizTracking5");
        }
        break;
    case 6:
        {
            WizardPictureBox->Image = nullptr;
            WizardInstructionsLbl->Text = (System::String^)rm->GetObject(L"CalWizTracking6");
            WizardNextBtn->Text = "Finish";
            WizardPrevBtn->Enabled = false;
            WizardCancelBtn->Enabled = false;
        }
        break;
    }

    if ( Step == 0 )
    {
        WizardPrevBtn->Enabled = false;
    }
    else if ( Step != 6 )
    {
        WizardPrevBtn->Enabled = true;
    }

    if ( Step != 6 )
    {
        WizardNextBtn->Text = "Next";
    }
}

//--------------------------------------------------------------------------------------
// Func: CompleteAxisCalStep
// Desc: Completes (does work) the specified calibration step in axis-alignment mode.
//       Called after the user clicks 'Next' in the Wizard.
//--------------------------------------------------------------------------------------
System::Void CalWizard::CompleteAxisCalStep(int Step)
{
    switch ( Step )
    {
    case 0:
        // Start collecting data
        mainUpdateTimer->Enabled = true;
        
        // Zero the existing cal
//        TrackerWrap::Tracker->StoreAccelCal(0, 0, 0);
//        TrackerWrap::Tracker->StoreMagCal(0, 0, 0);
        TrackerWrap::Tracker->CalibrateGyro();

        // Start mag and accel output from device for calibration. This call will
        // also stop any other stream data.
        TrackerWrap::Tracker->StreamMagAccelData(true);
        break;
    case 1:
        {
            MagXmax = MagXReading;
            MagYmax = MagYReading;
            AccZMax = AccZReading;
            UpdateCalc();
        }
        break;
    case 2:
        {
            MagXmin = MagXReading;
            MagYmin = MagYReading;
            MagZmax = MagZReading;
            UpdateCalc();
        }
        break;
    case 3:
        {
            AccXMax = AccXReading;
            UpdateCalc();
        }
        break;
    case 4:
        {
            AccXMin = AccXReading;
            UpdateCalc();
        }
        break;
    case 5:
        {
            AccYMax = AccYReading;
            UpdateCalc();
        }
        break;
    case 6:
        {
            AccYMin = AccYReading;
            UpdateCalc();
        }
        break;
    case 7:
        {
            MagZmin = MagZReading;
            AccZMin = AccZReading;
            UpdateCalc();
        }
        break;
    case 8:
        {
            TrackerWrap::Tracker->StoreAccelCal(AccXOffset, AccYOffset, AccZOffset);
            TrackerWrap::Tracker->StoreMagCal(MagXOffset, MagYOffset, MagZOffset);
        }
        break;
    case 9:
        TrackerWrap::Tracker->StreamMagAccelData(false);
        mainUpdateTimer->Enabled = false;
        Close();
        break;
    }
}

//--------------------------------------------------------------------------------------
// Func: CompleteTrackingCalStep
// Desc: Takes action based on the specified step of min/max tracking calibration mode.
//--------------------------------------------------------------------------------------
System::Void CalWizard::CompleteTrackingCalStep(int Step)
{
	double delay=1;
		bool retry=false;
		const int AP_COMPASS_OFFSETS_MAX_DEFAULT = 600;
		const float calibration_threshold=16;

    switch ( Step )
    {
    case 0:
        // Start collecting data
        mainUpdateTimer->Enabled = true;
        
        // Zero the existing cal
//        TrackerWrap::Tracker->StoreAccelCal(0, 0, 0);
//        TrackerWrap::Tracker->StoreMagCal(0, 0, 0);
        TrackerWrap::Tracker->CalibrateGyro();
        ZeroMinMaxTracking();
        
        // Start mag and accel output from device for calibration. This call will
        // also stop any other stream data.
		TrackMinMax = true;
        

		mag_calibrator.set_tolerance(calibration_threshold);
		acc_calibrator.set_tolerance(calibration_threshold);
		mag_calibrator.start(retry, delay, AP_COMPASS_OFFSETS_MAX_DEFAULT);
		acc_calibrator.start(retry, delay, AP_COMPASS_OFFSETS_MAX_DEFAULT);

		txtMag->Text = "";
		txtAcc->Text = "";

		// start!
		TrackerWrap::Tracker->StreamMagAccelData(true);
		break;
    case 1:
        break;
    case 2:
        // nothing to do. Just user instruction.
        break;
    case 3:
        // nothing to do. Just user instruction.
        break;
    case 4:
        // nothing to do. Just user instruction.
        break;
    case 5:
        TrackerWrap::Tracker->StreamMagAccelData(false);

		// calculate

		if( mag_calibrator.get_status()== COMPASS_CAL_SUCCESS && 
			acc_calibrator.get_status()== COMPASS_CAL_SUCCESS ){

			Vector3f ofs, diag, offdiag;
			mag_calibrator.get_calibration(ofs, diag, offdiag);

			// Store values
			TrackerWrap::Tracker->StoreMagCal(ofs.x, ofs.y, ofs.z);
			Sleep(100);
			TrackerWrap::Tracker->StoreMagGain(diag.x, diag.y, diag.z);
			Sleep(100);
			TrackerWrap::Tracker->StoreMagDiagOff(offdiag.x, offdiag.y, offdiag.z);

			acc_calibrator.get_calibration(ofs, diag, offdiag);

			TrackerWrap::Tracker->StoreAccelCal(ofs.x, ofs.y, ofs.z);
			Sleep(100);
			TrackerWrap::Tracker->StoreAccelGain(diag.x, diag.y, diag.z);
			Sleep(100);
			TrackerWrap::Tracker->StoreAccelDiagOff(offdiag.x, offdiag.y, offdiag.z);
			Sleep(100);

		}
		Sleep(100);
        break;

    case 6:
        mainUpdateTimer->Enabled = false;
        Close();
        break;
    }
}

#define SIZE_X 360
#define SIZE_Y 360

#define sqrt_2 1.41421356237
#define sqrt_3 1.73205080757

System::Void CalWizard::updateStatus(System::Windows::Forms::ProgressBar^  progress, CompassCalibrator &calibrator, System::Windows::Forms::TextBox^ txt) {

		
        uint8_t cal_status = calibrator.get_status();
 
        if (cal_status == COMPASS_CAL_WAITING_TO_START  ||
            cal_status == COMPASS_CAL_RUNNING_STEP_ONE ||
            cal_status == COMPASS_CAL_RUNNING_STEP_TWO) {
            double completion_pct = calibrator.get_completion_percent();
            auto& completion_mask = calibrator.get_completion_mask();
            Vector3f direction(0.0f,0.0f,0.0f);
            uint8_t attempt = calibrator.get_attempt();
			progress->Value = (int)completion_pct;
			//progress->ForeColor = cal_status == COMPASS_CAL_RUNNING_STEP_TWO? RGB(0,255,0):RGB(0,0, 255);
			//System::Drawing::Color

			txt->Text = cal_status == COMPASS_CAL_RUNNING_STEP_TWO? "Step two": "Step one";
        }

		
        else if ((cal_status == COMPASS_CAL_SUCCESS ||
            cal_status == COMPASS_CAL_FAILED)) {
            double fitness = calibrator.get_fitness();
            Vector3f ofs, diag, offdiag;
            calibrator.get_calibration(ofs, diag, offdiag);
			txt->Text = (cal_status == COMPASS_CAL_SUCCESS ? "Succes" : "Failed") + " " + Convert::ToString(fitness);
		}
}

System::Void CalWizard::NewPoint(double ax,double ay,double az,double mx,double my,double mz){
	
	Vector3d tm,ta;

	const Vector3f vm= Vector3f(mx,my,mz);
	const Vector3f va= Vector3f(ax,ay,az);

	tm.v=new Vector3f(mx,my,mz);
	ta.v=new Vector3f(ax,ay,az);

	MagData->Add(tm);
	AccData->Add(ta);

	mag_calibrator.new_sample(vm);
	acc_calibrator.new_sample(va);

	updateStatus(prgMag,mag_calibrator, txtMag);
	updateStatus(prgAcc,acc_calibrator, txtAcc);

	//pictureBox1->Refresh();
	Pen^ rp = gcnew Pen( Color::Red, 1.0f );
	Pen^ bp = gcnew Pen( Color::Blue, 1.0f );

	Matrix3f izo = Matrix3f(
		sqrt_3, 0, -sqrt_3,
		1, 2, 1,
		sqrt_2,-sqrt_2,sqrt_2
	);

	Matrix3f &plan = Matrix3f(
		1,0,0,
		0,1,0,
		0,0,0
	);

	int L=180, center_x=SIZE_X/2, center_y = SIZE_Y/2 + 20;
	System::Drawing::Graphics ^ g;
	g = pictureBox1->CreateGraphics();


	Vector3f result= (izo * *tm.v);
		result = result / sqrt(6.0);
		result = plan*result;

		int x,y;
		x=(int)(center_x + result.x/3);
		y=(int)(center_y - result.y/3);

		g->DrawLine(bp, x,y,  x+1,y );

	result= (izo * *ta.v);
		result = result / sqrt(6.0);
		result = plan*result;

		x=(int)(center_x + result.x/3);
		y=(int)(center_y - result.y/3);

		g->DrawLine(rp, x,y,  x+1,y );



		// update status


}

System::Void CalWizard::pictureBox1_Paint(System::Object^ sender, System::Windows::Forms::PaintEventArgs^ e) {

/*	Graphics ^ g = pictureBox1->CreateGraphics();
    Pen^ p = gcnew Pen( Color::Black,1.0f );
    Point center = Point(180, 180);
    Point up = Point(180, 360);
    g->DrawLine(p, center, up);
*/
	Pen^ p = gcnew Pen( Color::Black, 1.0f );
	Pen^ rp = gcnew Pen( Color::Red, 1.0f );
	Pen^ bp = gcnew Pen( Color::Blue, 1.0f );

	double alp=60.0 / 180 *3.14159265;
	int L=180, center_x=SIZE_X/2, center_y = SIZE_Y/2 + 20;

	e->Graphics->DrawLine(p, center_x,center_y, center_x,center_y - L); // up
	e->Graphics->DrawLine(p, center_x,center_y, (int)(center_x - (L * sin(alp))), (int)(center_y + (L * cos(alp))) );
	e->Graphics->DrawLine(p, center_x,center_y, (int)(center_x + (L * sin(alp))), (int)(center_y + (L * cos(alp))) );

	Matrix3f izo = Matrix3f(
		sqrt_3, 0, -sqrt_3,
		1, 2, 1,
		sqrt_2,-sqrt_2,sqrt_2
	);

	Matrix3f &plan = Matrix3f(
		1,0,0,
		0,1,0,
		0,0,0
	);

	for(int i=0; i< MagData->Count; i++){
		Vector3d v = MagData[i];
		Vector3f result= (izo * *v.v);
		result = result / sqrt(6.0);
		result = plan*result;

		int x,y;
		x=(int)(center_x + result.x/3);
		y=(int)(center_y - result.y/3);

		e->Graphics->DrawLine(bp, x,y,  x+1,y );

	}

	for(int i=0; i< AccData->Count; i++){
		Vector3d v = AccData[i];
		Vector3f result= (izo * *v.v);
		result = result / sqrt(6.0);
		result = plan*result;

		int x,y;
		x=(int)(center_x + result.x/3);
		y=(int)(center_y - result.y/3);


		e->Graphics->DrawLine(rp, x,y,  x,y+1 );

	}


	//e->Graphics->DrawLine(p, 0,0, 180,180);
}

/*
https://ru.wikipedia.org/wiki/%D0%98%D0%B7%D0%BE%D0%BC%D0%B5%D1%82%D1%80%D0%B8%D1%87%D0%B5%D1%81%D0%BA%D0%B0%D1%8F_%D0%BF%D1%80%D0%BE%D0%B5%D0%BA%D1%86%D0%B8%D1%8F


izometr={ 
	{sqrt_3, 0, -sqrt_3},
	{1, 2, 1},
	{sqrt_2,-sqrt_2,sqrt_2}
};

plan={
	{1,0,0},
	{0,1,0},
	{0,0,0}
};

coords2 = plan * sqrt(6) * izometr * coords3

*/