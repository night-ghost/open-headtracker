//--------------------------------------------------------------------------------------
// File:    HeadTrackerHardware.h 
// Desc:    Implements a very basic hardware class for simple communications with Dennis
//          Frie's open source head tracker. Class client needs to read the port for
//          any streamed data; this object does not buffer replies from the hardware.
//
// Hist:    4/15/13 - Mark Mansur - initial creation
//--------------------------------------------------------------------------------------
#pragma once
#include "Windows.h"

using namespace System;
using namespace System::Diagnostics;

#define HT_TILT_REVERSE_BIT     0x01
#define HT_ROLL_REVERSE_BIT     0x02
#define HT_PAN_REVERSE_BIT      0x04

// Settings
//
value struct HTSETTINGS
{
    double LPTiltRoll;         // Firmware: tiltRollBeta
    double LPPan;              // Firmware: panBeta
    double GyroWeightTiltRoll; // Firmware: gyroWeightTiltRoll
    double GyroWeightPan;      // Firmware: GyroWeightPan
    double ServoGainPan;       // Firmware: tiltFactor
    double ServoGainTilt;      // Firmware: panFactor
    double ServoGainRoll;      // Firmware: rollFactor
    Byte ServoReverse;      // Firmware: servoReverseMask
    int PanCenter;          // Firmware: servoPanCenter
    int PanMin;             // Firmware: panMinPulse
    int PanMax;             // Firmware: panMaxPulse
    int TiltCenter;         // Firmware: servoTiltCenter
    int TiltMin;            // Firmware: tiltMinPulse
    int TiltMax;            // Firmware: tiltMaxPulse
    int RollCenter;         // Firmware: servoRollCenter
    int RollMin;            // Firmware: rollMinPulse
    int RollMax;            // Firmware: rollMaxPulse
    Byte PanCh;             // Firmware: htChannels[0]
    Byte TiltCh;            // Firmware: htChannels[1]
    Byte RollCh;            // Firmware: htChannels[2]
};

//---------------------------------------------------------------------------------------
//  Class: HeadTracker
//      Quick and dirty implementation of a head tracker helper. Application still has
//      to read the stream data when sensor streaming is enabled.
//---------------------------------------------------------------------------------------
ref class HeadTracker
{

    // The count of settings values returned from the head tracker. Update this
    // when new settings are added to the firmware. +1 for newline.
#define HT_SETTINGS_COUNT       20

private:
		System::Void (*todo)();

public: // methods

    HeadTracker()
    {
        Port = gcnew System::IO::Ports::SerialPort();
    }
    
    ~HeadTracker()
    {
    }

    System::Void Open(String^ ComPort, boolean ForceOpen ) {
        Port->PortName = ComPort;
        Port->BaudRate = 115200;
        Port->ReadTimeout = 2000;
        try    {
            Port->Open();
        }
        catch (System::Exception^ e)    {

        }
        
        if ( Port->IsOpen )
        {
			Port->DtrEnable = true;
			Sleep(30);
			Port->DtrEnable = false;
			Sleep(200);
/*			
            GetVersion();
        
            // Sometimes version retrieval early in power-up doesn't work. Try again.
            if ( _FWVersion == 0.0 )
                GetVersion();
        
            // If it failed again, the open request failed. IF we're forcing the port 
            // open, we don't care about failures, leave the COM port open.
            if ( _FWVersion == 0.0 && !ForceOpen)
                this->Close();
*/
        }
    }

    System::Void Close()
    {
        _FWVersion = 0.0;
        Port->Close();
    }

    HTSETTINGS RetrieveSettings()
    {
        HTSETTINGS ht;
        if (Port->IsOpen)
        {
            Port->DiscardInBuffer();
            //Port->WriteLine(String::Format("$GSET"));
			Port->WriteLine("$GSET\n");
            String^ Line = ReadData();
            try
            {
                if ( Line->Substring(0, 5)->Equals("$SET$") )
                {
                    String^ toParse = Line->Substring(5);
                    System::Globalization::CultureInfo^ ci = gcnew System::Globalization::CultureInfo("en-US");
                    System::Globalization::NumberFormatInfo^ fi = ci->NumberFormat;
                    fi->NumberDecimalSeparator = ".";

                    // Parse the rest of the response
                    array<wchar_t>^ delim = {',','\r','\n'};
                    array<String^>^ items = toParse->Split(delim);
                    if ( HT_SETTINGS_COUNT <= items->Length )
                    {
                        ht.LPTiltRoll = Convert::ToDouble(items[0], fi);
                        ht.LPPan = Convert::ToDouble(items[1], fi);                 
                        ht.GyroWeightTiltRoll =Convert::ToDouble(items[2], fi);        
                        ht.GyroWeightPan = Convert::ToDouble(items[3], fi);         
                        ht.ServoGainTilt = Convert::ToDouble(items[4], fi);          
                        ht.ServoGainPan = Convert::ToDouble(items[5], fi);         
                        ht.ServoGainRoll = Convert::ToDouble(items[6], fi);         
                        ht.ServoReverse = (byte)Convert::ToSingle(items[7], fi); 
                        ht.PanCenter = (int)Convert::ToSingle(items[8], fi);             
                        ht.PanMin = (int)Convert::ToSingle(items[9], fi);                
                        ht.PanMax = (int)Convert::ToSingle(items[10], fi);                
                        ht.TiltCenter = (int)Convert::ToSingle(items[11], fi);            
                        ht.TiltMin = (int)Convert::ToSingle(items[12], fi);               
                        ht.TiltMax = (int)Convert::ToSingle(items[13], fi);               
                        ht.RollCenter = (int)Convert::ToSingle(items[14], fi);            
                        ht.RollMin = (int)Convert::ToSingle(items[15], fi);               
                        ht.RollMax = (int)Convert::ToSingle(items[16], fi);               
                        ht.PanCh = (byte)Convert::ToSingle(items[17], fi); 
                        ht.TiltCh = (byte)Convert::ToSingle(items[18], fi);
                        ht.RollCh = (byte)Convert::ToSingle(items[19], fi);
                    }
                }
            }
            catch (System::Exception^ e)
            {

            }
        }
        return ht;
    }

    System::Void CommitSettings()
    {
        if ( Port->IsOpen)
        {
            Port->WriteLine("$SAVE\n");
        }
    }

    float GetVersion()
    {
        if ( Port->IsOpen)
        {
            Port->DiscardInBuffer();
            Port->WriteLine("$VERS\n");
            String^ versLine = ReadData();
            try
            {
                if ( versLine->Length > 4 && versLine->Substring(0, 4)->Equals("FW: ") )
                {
                    String^ vers = versLine->Substring(4);
                    // Parse the rest of the response
                    array<wchar_t>^ delim = {',','\r','\n'};
                    array<String^>^ items = vers->Split(delim);
                    System::Globalization::CultureInfo^ ci = gcnew System::Globalization::CultureInfo("en-US");
                    System::Globalization::NumberFormatInfo^ fi = ci->NumberFormat;
                    fi->NumberDecimalSeparator = ".";

                    _FWVersion = Convert::ToSingle(items[0], fi);
                }
                else
                {
                    _FWVersion = 0.0;
                }
            }
            catch (System::Exception^ e)
            {
                _FWVersion = 0.0;
            }
        }
        
        return _FWVersion;
    }

    System::Void StreamTrackingData(bool Start)
    {
        if ( Port->IsOpen)
        {
            if ( Start )
                Port->WriteLine("$PLST\n");
            else
            {
                Port->WriteLine("$PLEN\n");
                //_AccelStreaming = false;
                //_MagStreaming = false;
                _MagAccelStreaming = false;
            }
            _TrackStreaming = Start;
            _Streaming = Start;
        }
    }
/*    
    System::Void StreamAccelData(bool Start)
    {
        if ( Port->IsOpen)
        {
            if ( Start )
                Port->WriteLine("$GRAV\n");
            else
            {
                Port->WriteLine("$GREN\n");
                _MagStreaming = false;
                _MagAccelStreaming = false;
                _TrackStreaming = false;
            }
            _AccelStreaming = Start;
            _Streaming = Start;
        }
    }

    System::Void StreamMagData(bool Start)
    {
        if ( Port->IsOpen)
        {
            if ( Start )
                Port->WriteLine(String::Format("$CAST"));
            else
            {
                Port->WriteLine(String::Format("$CAEN"));
                _AccelStreaming = false;
                _MagAccelStreaming = false;
                _TrackStreaming = false;
            }
            _MagStreaming = Start;
            _Streaming = Start;
        }
    }
	*/
    System::Void StreamMagAccelData(bool Start)
    {
        if ( Port->IsOpen)
        {
            if ( Port->IsOpen)
            {
                if ( Start )
                    Port->WriteLine("$CMAS\n");
                else
                {
                    Port->WriteLine("$CMAE\n");
                    //_AccelStreaming = false;
                    //_MagStreaming = false;
                    _TrackStreaming = false;
                }
                _MagAccelStreaming = Start;
                _Streaming = Start;
            }
        }
    }

    System::Void StoreAccelCal(System::Double XOffset, System::Double YOffset, System::Double ZOffset)
    {
        if ( Port->IsOpen)
        {
//            StreamAccelData(false);
            Sleep(250);

            String^ xtemp = Convert::ToString((XOffset*10));
            String^ ytemp = Convert::ToString((YOffset*10));
            String^ ztemp = Convert::ToString((ZOffset*10));

            Port->WriteLine(String::Format("${0},{1},{2}ACC", xtemp, ytemp, ztemp) );
        }
    }

	System::Void StoreAccelGain(System::Double X, System::Double Y, System::Double Z)
    {
        if ( Port->IsOpen)
        {
            Sleep(250);

            String^ xtemp = Convert::ToString((X*10000));
            String^ ytemp = Convert::ToString((Y*10000));
            String^ ztemp = Convert::ToString((Z*10000));

            Port->WriteLine(String::Format("${0},{1},{2}ACG\n", xtemp, ytemp, ztemp) );
        }
    }

	System::Void StoreAccelDiagOff(System::Double X, System::Double Y, System::Double Z)
    {
        if ( Port->IsOpen)
        {
            Sleep(250);

            String^ xtemp = Convert::ToString((X*10000));
            String^ ytemp = Convert::ToString((Y*10000));
            String^ ztemp = Convert::ToString((Z*10000));

            Port->WriteLine(String::Format("${0},{1},{2}ACD\n", xtemp, ytemp, ztemp) );
        }
    }

    System::Void ResetAccelCal()
    {
        StoreAccelCal(0, 0, 0);
    }

    System::Void StoreMagCal(System::Double XOffset, System::Double YOffset, System::Double ZOffset)
    {
        if ( Port->IsOpen)
        {
//            StreamMagData(false);
            Sleep(250);

            String^ xtemp = Convert::ToString((XOffset*10));
            String^ ytemp = Convert::ToString((YOffset*10));
            String^ ztemp = Convert::ToString((ZOffset*10));

            Port->WriteLine(String::Format("${0},{1},{2}MAG\n",xtemp,ytemp,ztemp) );
        }
    }

	System::Void StoreMagGain(System::Double X, System::Double Y, System::Double Z)
    {
        if ( Port->IsOpen)
        {
            String^ xtemp = Convert::ToString((X*10000));
            String^ ytemp = Convert::ToString((Y*10000));
            String^ ztemp = Convert::ToString((Z*10000));

            Port->WriteLine(String::Format("${0},{1},{2}MGA\n",xtemp,ytemp,ztemp) );
        }
    }

	    System::Void StoreMagDiagOff(System::Double X, System::Double Y, System::Double Z)
    {
        if ( Port->IsOpen)
        {
            String^ xtemp = Convert::ToString((X*10000));
            String^ ytemp = Convert::ToString((Y*10000));
            String^ ztemp = Convert::ToString((Z*10000));

            Port->WriteLine(String::Format("${0},{1},{2}MDA\n",xtemp,ytemp,ztemp) );
        }
    }

    System::Void ResetMagCal()
    {
        StoreMagCal(0, 0, 0);
    }

    System::Void CalibrateGyro()    {
        if ( Port->IsOpen)        {
            Port->WriteLine("$CALG\n");
        }
    }

    System::String^ ReadData()
    {
        System::String^ line;
     again:
        try {
            line = Port->ReadLine();
			if(line[0]=='#') goto again;
        }		
		catch (System::Exception^ e){
			
			Debug::WriteLine( e->Message );
			
			return "#Error#";
        }
		if(line == "$OK!$")
			(*todo)();
        return line;
    }

public: // objects

    // Be careful accessing this directly. Use built-in methods whenever possible.
    System::IO::Ports::SerialPort^ Port;

public: // properties
/*
    property bool AccelStreaming
    {
        bool get()
        {
            return _AccelStreaming;
        }
    }
    property bool MagStreaming
    {
        bool get()
        {
            return _MagStreaming;
        }
    }
*/
    property bool MagAccelStreaming
    {
        bool get()
        {
            return _MagAccelStreaming;
        }
    }
    property bool TrackStreaming
    {
        bool get()
        {
            return _TrackStreaming;
        }
    }

    property bool Streaming
    {
        bool get()
        {
            return _Streaming;
        }
    }

    property float Version
    {
        float get()
        {
            return _FWVersion;
        }
    }

private:

    bool _TrackStreaming;
 //   bool _MagStreaming;
//    bool _AccelStreaming;
    bool _MagAccelStreaming;
    bool _Streaming;
    float _FWVersion;
};

ref class TrackerWrap
{
public:
    static HeadTracker^ Tracker;
};