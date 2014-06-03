/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |              http://sourceforge.net/p/openmora/home/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Javier Gonzalez Monroy  <jgmonroy@isa.uma.es>                 |
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

/**  @moos_module Module to reading the information from a Libelium waspmote based Wireless Sensor Network.
	* This module reads the variables published by other modules as:
	* The file saved is an MRPT rawlog format file that can be viewed, edited and used withing the MRPT RawlogViewer tool
	*/
#include "CLibeliumWSNApp.h"

#include <mrpt/hwdrivers/CBoardENoses.h>
#include <mrpt/utils.h>
#include <mrpt/gui.h>
#include <mrpt/hwdrivers/CSerialPort.h>


//#include <sstream>
//#include <iomanip>
#include <iostream>
#include <stdio.h>
#include <time.h>
using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::gui;



CLibeliumWSNApp::CLibeliumWSNApp()
{
}

CLibeliumWSNApp::~CLibeliumWSNApp()
{
}


//-------------------------------------
// OnStartUp()
//-------------------------------------
bool CLibeliumWSNApp::OnStartUp()
{
  EnableCommandMessageFiltering(true);

  try
  {
	//! @moos_param port_com
    m_port_com	=       m_ini.read_string("pZBModule","port_com","",true);
	//! @moos_param baud
    m_baud	=			m_ini.read_int("pZBModule","baud",38400);
	//! @moos_param data_bits
    m_data_bits	=		m_ini.read_int("pZBModule","data_bits",8);
	//! @moos_param parity
    m_parity	=		m_ini.read_int("pZBModule","parity",0);
	//! @moos_param stop_bits
    m_stop_bits	=		m_ini.read_int("pZBModule","stop_bits",1);
	//! @moos_param flow_control
    m_flow_control =	m_ini.read_bool("pZBModule","flow_control",false);

    //Port Constructor
    mPortZB.setSerialPortName(m_port_com);
    mPortZB.open();
    //cout << "Puerto inicializado corectamente" << endl;
    mPortZB.setConfig(m_baud,m_parity,m_data_bits,m_stop_bits,m_flow_control);
    cout << "Port initialized correctly" << endl;

    if(mPortZB.isOpen()){
        cout <<  m_port_com + "open" << endl;
      }

    waspmotes.clear();

    cout << "Finish OnStartUp" << endl;
    return true;
  }
  catch (std::exception &e)
  {
    cerr << "**ERROR** " << e.what() << endl;
    return MOOSFail( "Closing due to an exception." );
  }
}


//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool CLibeliumWSNApp::OnCommandMsg( CMOOSMsg Msg )
{
  if(Msg.IsSkewed(MOOSTime())) return true;
  if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
  const std::string sCmd = Msg.GetString();
  //MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
  // Process the command "sCmd".

  return true;
}


//-------------------------------------
// Iterate()
//-------------------------------------
bool CLibeliumWSNApp::Iterate()
{
  try
  {
    string readline;
    size_t st;
    bool empty;
    readline=mPortZB.ReadString(-1,&empty,"\n");
    st = readline.find("ID=",0);
    if (st != string::npos)
      readline = readline.substr(st,readline.size()-st);
    else
      return 0; // Line doesn't contain any useful information, return.

    vector<string>tokens;
    tokenize(readline.c_str(),",",tokens);

    CStringList waspmote(tokens);

    //        for (unsigned int i=0;i<tokens.size();i++){
    //            cout << tokens[i] << endl;
    //          }
    unsigned int current_waspmote = waspmote.get_int("ID");

    //Search if this ID is already in our 'waspmotes' list. If it is, we include the new variables or update them. If it isn't we will add this waspmote to the waspmotes list.
    bool found = false;
    for (unsigned int i=0 ; (i < waspmotes.size())&&(!found) ; i++)
      {
        if (waspmotes.at(i).get_int(("ID")) == current_waspmote)
          {
            found=true;

            for (unsigned int j = 0; j < waspmote.size() ; j++) //If this waspmote was already in our 'waspmotes' list, update or insert new variables via the 'set' procedure.
              {
                string str;
                waspmote.get(j,str);
                tokenize(str.c_str(),"=",tokens);
                waspmotes.at(i).set(tokens[0],tokens[1]);
              }
          }
      }
    if (!found) waspmotes.push_back(waspmote); // Insert the waspmote into the waspmote list, including all the variables that we received.

    string str_all_data;
    str_all_data.clear();

    for (unsigned int i=0 ; i < waspmotes.size() ; i++)
      {
        string auxstr;
        for (unsigned int j=0 ; j < waspmotes.at(i).size() ;j++)
          {
            waspmotes.at(i).get(j,auxstr);
            str_all_data.append(auxstr);
          }

      }
	//! @moos_publish   Waspmote_Network_Data   
    m_Comms.Notify("Waspmote_Network_Data",str_all_data);
    return true;
  }

  catch(exception& e)
  {
    cerr << "**ERROR** " << e.what() << endl;
    return MOOSFail( "Closing due to an exception." );
  }
}


//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool CLibeliumWSNApp::OnConnectToServer()
{
  DoRegistrations();
  return true;
}


//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool CLibeliumWSNApp::DoRegistrations()
{
  //! @moos_subscribe SHUTDOWN
  AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

  //! @moos_subscribe Waspmote_Network_Data
  AddMOOSVariable( "Waspmote_Network_Data", "Waspmote_Network_Data", "Waspmote_Network_Data", 0 );

  RegisterMOOSVariables();
  return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool CLibeliumWSNApp::OnNewMail(MOOSMSG_LIST &NewMail)
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
