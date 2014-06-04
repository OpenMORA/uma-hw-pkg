/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Francisco Meléndez Fernández  <fco.melendez@uma.es>           |
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

#ifndef CRobotGiraffApp_H
#define CRobotGiraffApp_H


#include <COpenMORAMOOSApp.h>
#include <mrpt/hwdrivers/CActivMediaRobotBase.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include "CGiraffCommunication.h"
#include "CGiraffMotorsCom.h"

class CRobotGiraffApp : public COpenMORAApp
{

 
public:
    CRobotGiraffApp();
    virtual ~CRobotGiraffApp();
		NAAS::CGiraffCommunication* pointer_communication;
		NAAS::CGiraffMotorsCom* pointer_motors;
		std::string test_command;
		bool first;
		mrpt::poses::CPose2D m_odo;
		mrpt::system::TTimeStamp m_time_odo;
		double m_left, m_right, m_absoluteX, m_absoluteY, m_orientation,m_b;
		std::string m_takesControl;
		CTicTac	m_clock;

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

    /** performs the registration for mail */
    bool DoRegistrations();

	// DATA
	double m_last_v, m_last_w;
	double m_accel_limit;
	size_t control_mode;		//The current Giraff_Control_mode (0= manual, 2=Auto)

private:
	double						Is_Charging; // Detection of Battery Recharging, 0 = Not being recharged, 1 = Is being recharged
	void setGiraffVelocities( double &lin_vel, double &ang_vel);
	void getGiraffOdometryFull(
				mrpt::poses::CPose2D	&out_odom,
				double			&out_lin_vel,
				double			&out_ang_vel,
				double			&out_left_encoder_distance,
				double			&out_right_encoder_distance
				);
	/** Retrieve battery and charging status */
	void getGiraffBatteryData(bool &isDocked, double &bat_volt);
};

#endif
