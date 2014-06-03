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


/**  @moos_module The interface to a MobileRobotics (ARIA-based) mobile robotic base.
  *  This module enables the communications (and some rudimentary control) for ActivMedia robotic bases (Giraff DX/AT, PeopleBot, etc).
  *  There is implemented access to robot odometry, ticks counts, velocities, battery charge status, and sonar readings, as well as basic velocity control.
  */

#include "CRobotGiraffApp.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <mrpt/slam.h>

using namespace std;

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;


CRobotGiraffApp::CRobotGiraffApp() :
    m_last_v(0), m_last_w(0)
{
}

CRobotGiraffApp::~CRobotGiraffApp()
{	
}

bool CRobotGiraffApp::OnStartUp()
{
    DoRegistrations();

    try
    {
		// Load common & sensor specific parameters:
		//! @moos_param CActivMediaRobotBase_Parameters See description here <a href='http://reference.mrpt.org/svn/classmrpt_1_1hwdrivers_1_1_c_activ_media_robot_base.html'>mrpt::hwdrivers::CActivMediaRobotBase</a>
		m_robot.loadConfig( m_ini, "" );
		m_robot.initialize();

		return true;
    }
	catch (std::exception &e)
	{
		return MOOSFail( (string("**ERROR**") + string(e.what())).c_str() );
	}
}

bool CRobotGiraffApp::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("pMobileRobot_Giraff only accepts string command messages\n");

    std::string sCmd = Msg.GetString();

    MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

    return true;
}

bool CRobotGiraffApp::Iterate()
{
	try
	{
		//cout << "[robot_Giraff] cmd_v=" << cmd_v << " cmd_w=" << RAD2DEG(cmd_w) << endl;
		m_robot.setVelocities(m_last_v,m_last_w);

		// Publish odometry:
		CPose2D  odo;
		double cur_v,cur_w;
		int64_t  ticks_l,ticks_r;
		m_robot.getOdometryFull(
			odo,
			cur_v,cur_w,
			ticks_l, ticks_r);

		//!  @moos_publish   ODOMETRY   The robot absolute odometry in format "[x y phi]"
		string sOdo;
		odo.asString(sOdo);
		m_Comms.Notify("ODOMETRY", sOdo );

		
		//!  @moos_publish   <ODOMETRY_OBS> The robot absolute odometry as mrpt::slam::CObservationOdometry
		mrpt::slam::CObservationOdometryPtr odom = mrpt::slam::CObservationOdometry::Create();
		odom->odometry = odo;
		odom->timestamp = now();
		odom->hasVelocities = true;
		odom->velocityLin = cur_v;
		odom->velocityAng = cur_w;
		odom->hasEncodersInfo = true;
		odom->encoderLeftTicks = ticks_l;
		odom->encoderRightTicks = ticks_r;

		string sOdom = ObjectToString( odom.pointer() );
		m_Comms.Notify("ODOMETRY_OBS", sOdom );

		//!  @moos_publish   <BATTERY_V> The actual bettery level of the Mobile Robotic Base
		mrpt::slam::CObservationBatteryStatePtr battery_obs = mrpt::slam::CObservationBatteryState::Create();
		battery_obs->timestamp = now();
		m_robot.getBatteryCharge(battery_obs->voltageMainRobotBattery);
		battery_obs->voltageMainRobotBatteryIsValid = true;
		string sBattery = ObjectToString( battery_obs.pointer() );
		m_Comms.Notify("BATTERY_V", sBattery );

		
		
		//!  @moos_publish <BUMPERS> The state of front and read bumpers (1 pressed /0 unpressed) of the Mobile Robotic Base
		mrpt::vector_bool bumper_state;
		m_robot.getBumpers(bumper_state);
		string bumper_str="";
		bool alert=false;
		static bool first_alert=true;
		////4-7 indicates the front bumpers
		for (unsigned short jb=5;jb<bumper_state.size();jb++)
		{
			if (bumper_state[jb]) 
				{
					bumper_str.append("1");
					printf("triggered bumper %d\n",jb);
					alert=true;
				}	
			else bumper_str.append("0");
		}
		if (alert && m_robot.areMotorsEnabled()) {m_robot.DisableMotors();first_alert=true;}
		if (!alert && first_alert) {m_robot.EnableMotors();first_alert=false;}

		m_Comms.Notify("FBUMPERS",bumper_str.substr(5,9));
	

	    return true;
    }
	catch (std::exception &e)
	{
		return MOOSFail( (string("**ERROR**") + string(e.what())).c_str() );
	}

}


bool CRobotGiraffApp::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CRobotGiraffApp::DoRegistrations()
{
	//! @moos_subscribe MOTION_CMD_V
    this->m_Comms.Register("MOTION_CMD_V",0);
	//! @moos_subscribe MOTION_CMD_W
    this->m_Comms.Register("MOTION_CMD_W",0);
	//! @moos_subscribe ENABLE_MOTORS
	this->m_Comms.Register("ENABLE_MOTORS",0);
	//! @moos_subscribe SHUTDOWN
	this->m_Comms.Register("SHUTDOWN",0);	

    RegisterMOOSVariables();
    return true;
}

bool CRobotGiraffApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	for (MOOSMSG_LIST::const_iterator it=NewMail.begin();it!=NewMail.end();++it)
	{
	    const CMOOSMsg &m = *it;

	    if (MOOSStrCmp(m.GetKey(),"MOTION_CMD_V"))
            m_last_v = m.GetDouble();
	    if (MOOSStrCmp(m.GetKey(),"MOTION_CMD_W"))
            m_last_w = m.GetDouble();
		if( (MOOSStrCmp(m.GetKey(),"SHUTDOWN")) && (MOOSStrCmp(m.GetString(),"true")) )
		{
			// Disconnect comms:
			/*MOOSTrace("Disconnecting and Disabling Motors \n");
			m_robot.disconnectAndDisableMotors();
			mrpt::system::sleep(1000);
			MOOSTrace("Closing Module \n");*/
			this->RequestQuit();
			
		}
		if (MOOSStrCmp(m.GetKey(),"ENABLE_MOTORS"))
		{
			if (m.GetDouble()>0) 
				{
					m_robot.EnableMotors();
				}
			else 
				{
					m_robot.DisableMotors();
				}
		}
	}

    UpdateMOOSVariables(NewMail);
    return true;
}

