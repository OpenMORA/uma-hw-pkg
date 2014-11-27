/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2012  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |    Robotics (MAPIR) Lab, University of Malaga (Spain).                    |
   |    Contact: Carlos Sánchez  <carlossanchez@uma.es>                        |
   |                                                                           |
   |   This file is part of the MORA project.                                  |
   |                                                                           |
   |     MORA is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MORA is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MORA.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

/**  @moos_module Interface to a Data AQuisition Board (DAQ) from Advantech (USB4711A, etc).
  *  Used to read the voltage and input/output currents from the main battery of the robot
  *  (Useful when this info is not provided by the robot base, or when aditional batteries are equipped)
  */

#include "CDataAcquisition.h"

using namespace std;

// Constructor
CDataAcquisition::CDataAcquisition() 
{
}

// Destructor
CDataAcquisition::~CDataAcquisition()
{		
	//-------------------------------------
	// Close Device
	//------------------------------------

	m_ErrCode = DRV_DeviceClose(&m_DriverHandle); // Closes the DriverHandle specified device

	if (m_ErrCode != SUCCESS) // Error Checking
	{
		DRV_GetErrorMessage(m_ErrCode,(LPSTR)m_szErrorMsg); //Retrieves the corresponding error message by the specified error code
		MOOSTrace("Driver Message 4: %s\n",m_szErrorMsg);
	}

}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool CDataAcquisition::OnStartUp()
{

	//! @moos_param Mode  Acquisition Mode, 0: Single Input, 1: Multiple Inputs
	Mode = m_ini.read_float("","Mode",0,true); 

	//! @moos_param m_dwDevideNum  The number that identifies the device
	m_dwDeviceNum = m_ini.read_float("","m_dwDeviceNum",0,true); 

	//! @moos_param Chan  The sampled channel
	Chan = m_ini.read_float("","Chan",0,true); 

	//! @moos_param Gain  Gain Code: Refer to hardware manual for Code Range
	Gain = m_ini.read_float("","Gain",0,true); 
	
	//! @moos_param TrigMode  Trigger Mode, 0: internal trigger, 1:external trigger
	TrigMode = m_ini.read_float("","TrigMode",0,true); 	

	//-------------------------------------
	// Open Device
	//-------------------------------------

	m_ErrCode = DRV_DeviceOpen(m_dwDeviceNum, &m_DriverHandle); // Opens the corresponding installed device by its device number(DeviceNum) and returns the DriverHandle for later operations

	if (m_ErrCode != SUCCESS) // Error Checking
	{
		DRV_GetErrorMessage(m_ErrCode,(LPSTR)m_szErrorMsg); // Retrieves the corresponding error message by the specified error code
		MOOSTrace("Driver Message 1: %s\n",m_szErrorMsg);
		DRV_DeviceClose(&m_DriverHandle); // Closes the DriverHandle specified device
		return TRUE;
	}

	return DoRegistrations();
}
//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool CDataAcquisition::OnCommandMsg( CMOOSMsg Msg ) /////////////////DUDA
{
	if(Msg.IsSkewed( MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}

//-------------------------------------
// Iterate()
//-------------------------------------
bool CDataAcquisition::Iterate()
{
	//-------------------------------------
	// Acquisition Mode, 0: Single Input, 1: Multiple Inputs
	//-------------------------------------
	switch (Mode)
	{
	case 0: // Single Input Mode

		//-------------------------------------
		// Configure Input Range
		//------------------------------------
		ptAIConfig.DasChan = Chan;// The sampled channel
		ptAIConfig.DasGain = Gain;// Gain Code: Refer to hardware manual for Code Range

		dwErrCde = DRV_AIConfig (m_DriverHandle, &ptAIConfig); // Configure an AI (analog input) channel's input voltage range by setting the corresponding GainCode on the DriverHandle specified device
	
		if (dwErrCde != SUCCESS) // Error Checking
		{ 
			DRV_GetErrorMessage(dwErrCde,(LPSTR)m_szErrorMsg); // Retrieves the corresponding error message by the specified error code
			MOOSTrace("Driver Message 2: %s\n",m_szErrorMsg);
			DRV_DeviceClose(&m_DriverHandle); // Closes the DriverHandle specified device
			return TRUE; 
		}

		//-------------------------------------
		// Read One Data
		//------------------------------------
		ptAIVoltageIn.chan = Chan;	// The sampled channel
		ptAIVoltageIn.gain = Gain;	// Gain Code: Refer to hardware manual for Code Range	
		ptAIVoltageIn.TrigMode = TrigMode;	// Trigger Mode, 0: internal trigger, 1:external trigger
		ptAIVoltageIn.voltage = &fVoltage;	// Voltage retrieved (Units = Volts)

		dwErrCde = DRV_AIVoltageIn (m_DriverHandle, &ptAIVoltageIn); // Reads an analog input channel and returns the result scaled to a voltage. (units = volts) 

		if (dwErrCde != SUCCESS) // Error Checking
		{ 
			DRV_GetErrorMessage(dwErrCde,(LPSTR)m_szErrorMsg); // Retrieves the corresponding error message by the specified error code
			MOOSTrace("Driver Message 3: %s\n",m_szErrorMsg);
			DRV_DeviceClose(&m_DriverHandle); // Closes the DriverHandle specified device
			return TRUE; 
		}

		//-------------------------------------
		// Display reading data
		//------------------------------------
		//!  @moos_publish   Volt1  Voltage measurement of differential channel 1 (Units = Volts)
		m_Comms.Notify("Volt1", fVoltage);	

		break;

	case 1: // Multiple Inputs Mode

		//-------------------------------------
		// Configure Input Range
		//------------------------------------
		std::fill_n(GainArray,4,Gain); 

		ptMAIConfig.NumChan = 4; // Number of logical channel(s)
		ptMAIConfig.StartChan = 0; // Start one of scanned channels
		ptMAIConfig.GainArray = (USHORT far *)&GainArray[0]; // Gain Code: Refer to hardware manual for Code Range
		ptMAIVoltageIn.NumChan = 4; // Number of logical channel(s)
		ptMAIVoltageIn.StartChan = 0; //Start channel
		ptMAIVoltageIn.GainArray = (USHORT far *)&GainArray[0]; //Gain code array
		ptMAIVoltageIn.TrigMode = TrigMode;	// Trigger mode
		ptMAIVoltageIn.VoltageArray = (FLOAT far *)&fVoltageArray[0]; //The measured voltage returned, scaled to units of volts

		dwErrCde = DRV_MAIConfig (m_DriverHandle, &ptMAIConfig); //Configure an AI (analog input) channel's input voltage range by setting the corresponding GainCode on the DriverHandle specified device
	
		if (dwErrCde != SUCCESS) // Error Checking
		{ 
			DRV_GetErrorMessage(dwErrCde,(LPSTR)m_szErrorMsg); // Retrieves the corresponding error message by the specified error code
			MOOSTrace("Driver Message 2: %s\n",m_szErrorMsg);
			DRV_DeviceClose(&m_DriverHandle); // Closes the DriverHandle specified device
			return TRUE; 
		}

		//-------------------------------------
		// Read Data
		//------------------------------------
		dwErrCde = DRV_MAIVoltageIn(m_DriverHandle, &ptMAIVoltageIn); //Reads multiple analog input channels and returns the results scaled to voltages (units = volts) 

		if (dwErrCde != SUCCESS) // Error Checking
		{	
			DRV_GetErrorMessage(dwErrCde,(LPSTR)m_szErrorMsg); // Retrieves the corresponding error message by the specified error code
			MOOSTrace("Driver Message 3: %s\n",m_szErrorMsg);
			DRV_DeviceClose(&m_DriverHandle); // Closes the DriverHandle specified device
			return 0;
		} 

		//-------------------------------------
		// Display reading data
		//------------------------------------ 
		//!  @moos_publish   DAQ_VOLT1  Voltage measurement of differential channel 1 (Units = Volts) from the Data AQuisition Board
		m_Comms.Notify("DAQ_VOLT1", fVoltageArray[0] );

		//!  @moos_publish   DAQ_VOLT2  Voltage measurement of differential channel 2 (Units = Volts) from the Data AQuisition Board
		m_Comms.Notify("DAQ_VOLT2", fVoltageArray[1] );

		//!  @moos_publish   DAQ_VOLT3  Voltage measurement of differential channel 3 (Units = Volts) from the Data AQuisition Board
		m_Comms.Notify("DAQ_VOLT3", fVoltageArray[2] );

		//!  @moos_publish   DAQ_VOLT4  Voltage measurement of differential channel 4 (Units = Volts) from the Data AQuisition Board
		m_Comms.Notify("DAQ_VOLT4", fVoltageArray[3] );
		printf(".");
		break;
	}

	return true; 
}


//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool CDataAcquisition::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool CDataAcquisition::DoRegistrations()
{
	
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	RegisterMOOSVariables();
	return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool CDataAcquisition::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{				
		if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:			
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}

	}

    UpdateMOOSVariables(NewMail);
    return true;
}

