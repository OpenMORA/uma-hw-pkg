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

/**  @moos_module Interface to a Wifi domotics server of the model "IP Power"  */

#include "CDomoticsIpPowerApp.h"

#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;


CDomoticsIpPowerApp::CDomoticsIpPowerApp()
	: m_IP("192.168.0.200"), m_user("admin"), m_pass("")
{
}

CDomoticsIpPowerApp::~CDomoticsIpPowerApp()
{
}

bool CDomoticsIpPowerApp::OnStartUp()
{
	// Read parameters (if any) from the mission configuration file.
	//! @moos_param IP The IP address of the IP Power server.
	m_MissionReader.GetConfigurationParam("IP",m_IP);

	//! @moos_param user The user for the HTTP get commands (default: 'admin').
	m_MissionReader.GetConfigurationParam("user",m_user);

	//! @moos_param password The password for the HTTP get commands.
	m_MissionReader.GetConfigurationParam("password",m_pass);

	return DoRegistrations();
}

bool CDomoticsIpPowerApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}

bool CDomoticsIpPowerApp::Iterate()
{

	return true;
}

bool CDomoticsIpPowerApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}


bool CDomoticsIpPowerApp::DoRegistrations()
{
	//! @moos_subscribe	DOMOTICS_OPERATION

	//! @moos_var DOMOTICS_OPERATION Orders to switch on/off each of the channels of a domotics server.
	//!       <br> The format of the strings is: <br>
	//!       ON/OFF <channel_number>   <br>
	//!       Channel numbers start at 0. For example: "ON 1", "OFF 0", "ON=2".

	m_Comms.Register("DOMOTICS_OPERATION",0);
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	RegisterMOOSVariables();
	return true;
}


bool CDomoticsIpPowerApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	for (MOOSMSG_LIST::const_iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
		const CMOOSMsg &m = *i;

		cout << m.GetName() << " " << m.GetString() << endl;

		if ( MOOSStrCmp( m.GetName(),"DOMOTICS_OPERATION"))
		{
			const std::string &s = m.GetString();
			sendCmdToIpPower(s);
		}
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


// "DOMOTICS_OPERATION=ON 0"
// cmd: "ON #channel"
// cmd: "OFF #channel"
// Return false on error.
bool CDomoticsIpPowerApp::sendCmdToIpPower(const std::string &cmd)
{
	string s = mrpt::system::trim(cmd);
	if (MOOSStrCmp(s.substr(0,3),"ON "))
	{
		const int channel = ::atoi(&s[3]);

		string ret, errmsg;
		int    retCode;
		string url = format("http://%s/SetPower.cgi?p%i=1", m_IP.c_str(), int(1+channel) );

		MOOSTrace("[pDomotics_IpPower] Switching ON channel #%u\nURL: %s\n", static_cast<unsigned int>(channel),url.c_str() );
		mrpt::utils::net::ERRORCODE_HTTP retVal = mrpt::utils::net::http_get(url,ret,errmsg,80,m_user,m_pass,&retCode);
		MOOSTrace("[pDomotics_IpPower] Response code: %i  Response: %s\n", retCode, ret.c_str());

		return retVal==net::erOk;
	}
	else
	if (MOOSStrCmp(s.substr(0,4),"OFF "))
	{
		const int channel = ::atoi(&s[4]);

		string ret, errmsg;
		int    retCode;
		string url = format("http://%s/SetPower.cgi?p%i=0", m_IP.c_str(), int(1+channel) );

		MOOSTrace("[pDomotics_IpPower] Switching OFF channel #%u\nURL: %s\n", static_cast<unsigned int>(channel),url.c_str() );
		mrpt::utils::net::ERRORCODE_HTTP retVal = mrpt::utils::net::http_get(url,ret,errmsg,80,m_user,m_pass,&retCode);
		MOOSTrace("[pDomotics_IpPower] Response code: %i  Response: %s\n", retCode, ret.c_str());

		return retVal==net::erOk;
	}
	else return false;
}
