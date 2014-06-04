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

#ifndef CDataAcquisition_H
#define CDataAcquisition_H

#include <COpenMORAMOOSApp.h>
#include <sstream>
#include <iomanip>
#include <iostream>

#include "Driver.h"

class CDataAcquisition : public COpenMORAApp
{
public:
    CDataAcquisition();
    virtual ~CDataAcquisition();

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

	bool DoRegistrations();

	// DATA. Your local variables here...

	ULONG					m_dwDeviceNum;
	LONG					m_DriverHandle;
	LRESULT					m_ErrCode;  
	unsigned char			m_szBuffer[100];
	unsigned char			m_szErrorMsg[40];
	LRESULT					dwErrCde;
	float					fVoltage;
	USHORT					Chan;
    USHORT					Gain;
    USHORT					TrigMode;
	PT_AIConfig				ptAIConfig;
	PT_AIVoltageIn			ptAIVoltageIn;
	int						Mode;
	PT_MAIConfig			ptMAIConfig;
	USHORT					NumChan;
    USHORT					StartChan;
    USHORT					GainArray[4];
	DEVFEATURES				DevFeatures;
	GAINLIST				glGainList;
	PT_MAIVoltageIn			ptMAIVoltageIn;
	float					fVoltageArray[4];				

};
#endif
