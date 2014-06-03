/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |              http://sourceforge.net/p/openmora/home/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Javier Gonzalez Monroy  <jgmonroy@isa.uma.es>                     |
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

#ifndef CLibeliumWSNApp_H
#define CLibeliumWSNApp_H

#include <MOOS/libMOOS/App/MOOSApp.h>
#include <mrpt/hwdrivers.h>
#include <mrpt/base.h>
#include <mrpt/utils.h>
#include <CMapirMOOSApp.h>
#include <mrpt/hwdrivers/CSerialPort.h>

using namespace mrpt::utils;

class CLibeliumWSNApp : public CMapirMOOSApp
{
public:
	/** Constructor */
    CLibeliumWSNApp();
	/** Destructor */
    virtual ~CLibeliumWSNApp();

protected:
	
	/** called at startup */
	virtual bool OnStartUp();
	/** called when new mail arrives */
	virtual bool OnNewMail(MOOSMSG_LIST & NewMail);
	/** called when work is to be done */
	virtual bool Iterate();
	/** called when app connects to DB */
	virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );
	// state our interest in variables
	bool DoRegistrations();


	// DATA. Your local variables here...	
	
	
	mrpt::hwdrivers::			CSerialPort mPortZB;
	//Infomation about COM
	std::string					m_port_com;
	int							m_baud;
	int							m_data_bits;
	int							m_parity;
	int							m_stop_bits;
	bool						m_flow_control;

	vector<CStringList> waspmotes;
};
#endif
