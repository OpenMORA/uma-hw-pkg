/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MORA project.                                   |
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

/**  @moos_module Module to control the e-Neck board on our ActiveMedia Pioneer robot Sancho.
  *  This module enables communications with the e-Neck board which controls up to three servos.  
  */

#include "CNeckApp.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <mrpt/system.h>

using namespace std;

using namespace mrpt;
using namespace mrpt::utils;

const int noServos = 3;

CNeckApp::CNeckApp()  : m_initialized_ok(false)
{
	//m_eNeckBoard.setTruncateFactor( 1 );
	//m_eNeckBoard.setNumberOfPreviousAngles( 5 );
		offsetServo = new double[noServos];
	for (int i =0; i<noServos;i++){offsetServo[i]=0;}
}

CNeckApp::~CNeckApp()
{
	m_eNeckBoard.disableServo(0);
}

bool CNeckApp::OnStartUp()
{
	// Read parameters (if any) from the mission configuration file.
	unsigned int	filterDepth;
	double			truncateFactor;
	//! @moos_param filterDepth  The number of previous angles to be taken into account for computing the current angle (when filtering is active)
	if( m_MissionReader.GetConfigurationParam( "filterDepth", filterDepth ) )
		m_eNeckBoard.setNumberOfPreviousAngles( filterDepth );

	//! @moos_param truncateFactor  The maximum and minimum angle of the neck will be set to +-truncatefactor*pi/2
	if( m_MissionReader.GetConfigurationParam( "truncateFactor", truncateFactor ) )
		m_eNeckBoard.setTruncateFactor( truncateFactor );

	
	if 
	(
		//! @moos_param offsetServo0 The default offset (if any) of servo Nº0
		( m_MissionReader.GetConfigurationParam( "offsetServo0", offsetServo[0] ) ) ||

		//! @moos_param offsetServo1 The default offset of (if any) servo Nº1
		( m_MissionReader.GetConfigurationParam( "offsetServo1", offsetServo[1] ) ) ||

		//! @moos_param offsetServo2 The default offset of (if any) servo Nº2
		( m_MissionReader.GetConfigurationParam( "offsetServo2", offsetServo[2] ) )
	)
		m_eNeckBoard.setOffsets(DEG2RAD(offsetServo[0]), DEG2RAD(offsetServo[1]), DEG2RAD(offsetServo[2]));

	m_initialized_ok = false;

	return DoRegistrations();
}

bool CNeckApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}

bool CNeckApp::Iterate()
{
	int				servo = 0;
	double			angle = 0.0f;
	int				speed = 60;
	int				filter = 0;

	if (!m_initialized_ok)
	{
		// Try to connect to the eNeck:
		std::string firmVers;
		if (!m_eNeckBoard.queryFirmwareVersion( firmVers ) )
		{
			MOOSTrace("Cannot connect to USB device... Retrying in a moment.\n");
			mrpt::system::sleep(500);
		}
		else
		{
			MOOSTrace("FIRMWARE VERSION: %s\n",firmVers.c_str());
			mrpt::system::sleep(200);
			if( !m_eNeckBoard.enableServo() )
			{
				MOOSTrace("Servo could not be enabled... Retrying in a moment.\n");
				mrpt::system::sleep(500);
			}
			else
			{
				mrpt::system::sleep(200);
				if( !m_eNeckBoard.center() )
				{
					MOOSTrace("Servo could not be centered... Retrying in a moment.\n");
					mrpt::system::sleep(500);
				}
				else
				{
					m_initialized_ok=true;
					MOOSTrace("Initialization OK.\n");

					//Initialize the angles
					 m_eNeckBoard.setAngleAndSpeed( 0, 0 /*servo index*/, 30 );
					 m_eNeckBoard.setAngleAndSpeed( 0, 1 /*servo index*/, 30 );
				}
			}
		}
	}

	CMOOSVariable *neckMsg = GetMOOSVar("NECKMSG");				// Get the msg to the neck
	if (m_initialized_ok && neckMsg && neckMsg->IsFresh())
	{
		neckMsg->SetFresh(false);

		std::vector<string> tokens;
		mrpt::system::tokenize( neckMsg->GetStringVal(), "=,", tokens );
		if( tokens.size() != 8  ||			// SERVO: Servo to turn (integer)
			strcmp( tokens[0].c_str(), "SERVO" ) ||			// FILTER: Whether or not to filter the current angle (0|1)
			strcmp( tokens[2].c_str(), "ANG" ) ||			// ANG: Angle to turn (double)
			strcmp( tokens[4].c_str(), "SPEED" ) ||			// SPEED: The speed of the servo (15->250)
			strcmp( tokens[6].c_str(), "FILTER" ) )			// The msg must be as this: SERVO=0,ANG=20.3,SPEED=60,FILTER=1
		{
			this->SendSystemLogString( "pNeck: Bad string received" );
			MOOSTrace("Bad String received.\n");
			return false;
		}
		
		servo	= atoi( tokens[1].c_str() );
		angle	= atof( tokens[3].c_str() );
		speed	= atoi( tokens[5].c_str() );
		filter	= !strcmp( tokens[7].c_str(), "1" ) ? true : false;

		//MOOSTrace("%i\n",servo);
		//MOOSTrace("%f\n",angle);
		//MOOSTrace("%i\n",speed);
		//MOOSTrace("%f\n",filter);
		//MOOSTrace("%f\n",offsetServo[servo]);
		//MOOSTrace("%f\n",angle-offsetServo[servo]);
		// Main module loop code. Do the main stuff here...
		if( filter )
		{
			if( m_eNeckBoard.setAngleAndSpeed( DEG2RAD(angle), servo  /*servo index*/, speed ) )
				std::cout << "Sent angle to pNeck" << std::endl;
			else
				MOOSTrace(format("Error on SetAngle: Servo_Enabled? = %i",m_eNeckBoard.enableServo() ? 1:0 ));
			//if( m_eNeckBoard.setAngleWithFilter( DEG2RAD(angle-offsetServo[servo]), servo /*servo index*/, fast ) )
			//	std::cout << "Sent angle to pNeck (filtered)" << std::endl;
		}
		else
		{
			if( m_eNeckBoard.setAngleAndSpeed( DEG2RAD(angle), servo  /*servo index*/, speed ) )
				std::cout << "Sent angle to pNeck" << std::endl;
			else
				MOOSTrace(format("Error on SetAngle: Servo_Enabled? = %i",m_eNeckBoard.enableServo() ? 1:0 ));
		}
	}// end-if-isfresh

	return true;
}

bool CNeckApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

bool CNeckApp::DoRegistrations()
{
	//! @moos_subscribe	NeckMSG
	AddMOOSVariable("NECKMSG", "NECKMSG", "NECKMSG",0 /* Minimum time in seconds between updates */ );

	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	RegisterMOOSVariables();
	return true;
}

bool CNeckApp::OnNewMail(MOOSMSG_LIST &NewMail)
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
