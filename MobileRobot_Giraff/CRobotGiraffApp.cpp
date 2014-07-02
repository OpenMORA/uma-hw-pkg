/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |     Contact: Francisco Meléndez Fernández  <fco.melendez@uma.es>		   |
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


/**  @moos_module The interface to a Giraff robotic base
  *  This module enables the communications (and some rudimentary control) for the Giraff robotic bases (Giraff Technologies AB.)
  *  There is implemented access to robot odometry, ticks counts, velocities, battery charge status, as well as basic velocity control.
  */

#include "CRobotGiraffApp.h"
#include "CommonData.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/slam/CObservationBatteryState.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::poses;


/** Constructor*/
CRobotGiraffApp::CRobotGiraffApp() 
{
		pointer_motors= new NAAS::CGiraffMotorsCom(false);
		m_b = 0.467;
		m_absoluteX = 0;
		m_absoluteY = 0;
		m_orientation = 0;
		m_last_v = 0;
		m_last_w = 0;
		m_takesControl = "true";
}

/** Destructor*/
CRobotGiraffApp::~CRobotGiraffApp()
{	
}


bool CRobotGiraffApp::OnStartUp()
{
		//! @moos_param Accel_limit The maximum acceleration to be achieved.
		m_accel_limit = m_ini.read_double("","Accel_limit", 3.0, false);
		m_clock.Tic();
		
		//By default start with Manual Mode
		control_mode = 0;
		size_t giraffMode = 0;
		pointer_motors->execute("setMode", &giraffMode);
		MOOSDebugWrite("Starts with Control (Manual mode)\n");
		Is_Charging = 99;	//incorrect value to force publishing
		return DoRegistrations();
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
		//Debug - Iteration time
		//cout << endl << "Iteration time: " << m_clock.Tac();
		m_clock.Tic();

		//Check the control mode (Manual=Pilot= 0, or autonomous=OpenMORA= 2)
		if (control_mode == 2)
		{
			setGiraffVelocities(m_last_v,m_last_w);		
			string str=format("Velocities sent to the controller (time, v, w): (%f, %f, %f)", MOOSTime(), m_last_v, m_last_w);
			//MOOSDebugWrite(str);
		}


		// Get odometry:
		CPose2D  odo;
		double cur_v, cur_w, dist_l, dist_r;
		getGiraffOdometryFull( odo, cur_v, cur_w, dist_l, dist_r);
		string sOdo;
		odo.asString(sOdo);
		//!  @moos_publish ODOMETRY The robot absolute odometry in the format "[x y phi]"
		m_Comms.Notify("ODOMETRY", sOdo );
		//cout << "Odometry is: " << dist_l << " - " << dist_r << endl;


		// Get odometry as Obs (Serializable Object):
		mrpt::slam::CObservationOdometryPtr odom = mrpt::slam::CObservationOdometry::Create();
		odom->odometry = odo;
		odom->timestamp = now();
		odom->hasVelocities = true;
		odom->velocityLin = static_cast<float>(cur_v);
		odom->velocityAng = static_cast<float>(cur_w);
		odom->hasEncodersInfo = false;
		odom->encoderLeftTicks = 0;
		odom->encoderRightTicks = 0;		

		mrpt::vector_byte vec_odom;
		mrpt::utils::ObjectToOctetVector(odom.pointer(), vec_odom);
		//!  @moos_publish  ODOMETRY_OBS The robot absolute odometry as mrpt::slam::CObservationOdometry
		m_Comms.Notify("ODOMETRY_OBS", vec_odom);


		//Get Battery Information (charging status, bat level, etc)
		bool isDocked;
		double bat_volt;
		getGiraffBatteryData(isDocked, bat_volt);		

		return true;
    }
	catch (std::exception &e)
	{
		return MOOSFail( (string("**ERROR**") + string(e.what())).c_str() );
	}

}


bool CRobotGiraffApp::OnConnectToServer()
{
   // DoRegistrations();
    return true;
}


bool CRobotGiraffApp::DoRegistrations()
{
	//! @moos_subscribe MOTION_CMD_V
	AddMOOSVariable( "MOTION_CMD_V", "MOTION_CMD_V", "MOTION_CMD_V", 0);
    //this->m_Comms.Register("MOTION_CMD_V",0);

	//! @moos_subscribe MOTION_CMD_W    
	AddMOOSVariable( "MOTION_CMD_W", "MOTION_CMD_W", "MOTION_CMD_W", 0);
	//this->m_Comms.Register("MOTION_CMD_W",0);

	//! @moos_subscribe SHUTDOWN	
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0);
	//this->m_Comms.Register("SHUTDOWN",0);
		
	//! @moos_subscribe ROBOT_CONTROL_MODE
	AddMOOSVariable( "ROBOT_CONTROL_MODE", "ROBOT_CONTROL_MODE", "ROBOT_CONTROL_MODE", 0 );
	//this->m_Comms.Register("ROBOT_CONTROL_MODE",0);
	
	RegisterMOOSVariables();
    return true;
}

bool CRobotGiraffApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	size_t giraffModeTwo=2;
	size_t giraffMode=0;
	for (MOOSMSG_LIST::const_iterator it=NewMail.begin();it!=NewMail.end();++it)
	{
	    const CMOOSMsg &m = *it;
		// V
	    if (MOOSStrCmp(m.GetKey(),"MOTION_CMD_V"))
            m_last_v = m.GetDouble();
		
		// W
	    if (MOOSStrCmp(m.GetKey(),"MOTION_CMD_W"))
            m_last_w = m.GetDouble();
		
		// Giraff Control
		if (MOOSStrCmp(m.GetKey(),"ROBOT_CONTROL_MODE"))
		{
			control_mode = size_t(m.GetDouble());
			if (control_mode == 2)	//openMORA-Auto
			{
				pointer_motors->execute("setMode", &giraffModeTwo);
				MOOSDebugWrite("OpenMora takes Control\n");
			}
			else //Deault-Manual
			{
				control_mode = 0;
				// Stop the Giraff
				m_last_v = 0.0;
				m_last_w = 0.0;
				setGiraffVelocities(m_last_v,m_last_w);
				//Change mode
				pointer_motors->execute("setMode", &giraffMode);
				MOOSDebugWrite("Giraff takes Control\n");
			}
		}
		
		// ShutDown
		if( (MOOSStrCmp(m.GetKey(),"SHUTDOWN")) && (MOOSStrCmp(m.GetString(),"true")) )
			this->RequestQuit();		
	}
	
    UpdateMOOSVariables(NewMail);
    return true;
}

/** setGiraffVelocities Set V and W velocities to the Giraff robot when in Autonomous mode */
void CRobotGiraffApp::setGiraffVelocities( double &v, double &w)
{
	double p_pos = 1;
	double p_neg = -1;
	double p_cero = 0;
	size_t setting_mode = 2;
	double setting_a = m_accel_limit;
	size_t vel_mod = 1;
	double wrad = w;
	double speed_w_deg = (RAD2DEG(wrad));
	double vpos = -v;

	if (v==0 && speed_w_deg==0)
	{
		//pointer_motors->execute("setMode", &setting_mode );
		pointer_motors->execute("setAcceleration", &setting_a);
		pointer_motors->execute("setMaximumVirtualGearRatio", &speed_w_deg);
		pointer_motors->execute("setVel",&v,&setting_mode);
		pointer_motors->execute("setPos", &p_cero );
	}
	else if (v<0)
	{
		//pointer_motors->execute("setMode", &setting_mode );
		pointer_motors->execute("setAcceleration", &setting_a);
		pointer_motors->execute("setMaximumVirtualGearRatio", &speed_w_deg);
		pointer_motors->execute("setVel",&vpos,&setting_mode);
		pointer_motors->execute("setPos", &p_neg );
	}
	else
	{
		//pointer_motors->execute("setMode", &setting_mode );
		pointer_motors->execute("setAcceleration", &setting_a);
		pointer_motors->execute("setMaximumVirtualGearRatio", &speed_w_deg);
		pointer_motors->execute("setVel",&v,&setting_mode);
		pointer_motors->execute("setPos", &p_pos );
	}
}


void CRobotGiraffApp::getGiraffOdometryFull(poses::CPose2D	&out_odom,double &out_lin_vel,double &out_ang_vel,
											double &out_left_encoder_distance, double &out_right_encoder_distance)
{
	CPose2D cur_odo;
	double incOrientation,incCenterOfRobot;
	pointer_motors->execute("getVel",&out_lin_vel);
	pointer_motors->execute("getMaximumVirtualGearRatio",&out_ang_vel);
	pointer_motors->execute("getOdometry",&out_left_encoder_distance,&out_right_encoder_distance);
			

	incCenterOfRobot = (out_right_encoder_distance + out_left_encoder_distance)/2;
	incOrientation = (out_right_encoder_distance-out_left_encoder_distance)/m_b;
	m_orientation = m_orientation + incOrientation;

	m_absoluteX = m_absoluteX + incCenterOfRobot*cos( m_orientation - 0.5*incOrientation );
	m_absoluteY = m_absoluteY + incCenterOfRobot*sin( m_orientation - 0.5*incOrientation );
		
	cur_odo.x( m_absoluteX );
	cur_odo.y( m_absoluteY );
	cur_odo.phi( m_orientation );	
	//printf("Odometry observation Pose-> x:%f y:%f phi:%f",m_absoluteX,m_absoluteY,m_orientation);
	//printf("Odometry observation Velocities->v:%f w:%f left:%f right:%f\n",	out_lin_vel,out_ang_vel,
	//		out_left_encoder_distance,out_right_encoder_distance);
	out_odom = cur_odo;
}


/* Get battery information from Giraff robot */
void CRobotGiraffApp::getGiraffBatteryData(bool &isDocked, double &bat_volt)
{	
	// Chager data format: S:S#[Status value],[Parameter identifier]:[Value identifier]#[Parameter value] U#[CRC checksum][CR (carriage return character)]
	std::string bat_data;
	if (pointer_motors->execute("getChargerData", &bat_data))
	{
		//cout << "BATTERY string: " << bat_data << endl;

		/* Charger Status value: S:S#ABCD
		 * D=0 fast charging, D=1 Trickle charging, D=2 Error, D=3 Init, D=4 Not docked	 */
		size_t pos = bat_data.find(",");
		std::string charge_status = bat_data.substr(0, pos);
		bat_data.erase(0, pos+1);
		double ch = atof( &(charge_status.back()) );
		//cout << "Charger status is: " << ch << endl;
		ch = (ch<2);	//D=0 o 1 -> Charging
		if (ch != Is_Charging)
		{
			Is_Charging = ch;
			//! @moos_publish Is_Charging: The current charging status: 0 The battery is not charging, 1 The battery is charging
			m_Comms.Notify("Is_Charging", Is_Charging);
			if (ch)
				cout << "[CRobotGiraffApp]: Battery IS Charging" << endl;
			else
				cout << "[CRobotGiraffApp]: Battery NOT Charging" << endl;
		}
		
				
		//Get the Parameter identifier and the value
		// B#float is the current battery voltage in volts.
		pos = bat_data.find("#");
		std::string parameter = bat_data.substr(0, pos);
		bat_data.erase(0, pos+1);
		//cout << "Parameter is: " << parameter << endl;

		//Get value if parameter is battery
		if (!strcmp(parameter.c_str(),"B:F") )
		{
			pos = bat_data.find(" ");
			std::string parameter_value = bat_data.substr(0, pos);
			bat_data.erase(0, pos+1);
			
			//Pairwise-swap the string			
			for(size_t i=0, middle=parameter_value.size()/2, size=parameter_value.size(); i<middle; i+=2 )
			{
				std::swap(parameter_value[i], parameter_value[size - i- 2]);
				std::swap(parameter_value[i+1], parameter_value[size -i - 1]);
			}
			
			//IEEE 754 : HEX to Float
			unsigned int x;
			std::stringstream ss;
			ss << std::hex << parameter_value;
			ss >> x;
			float bat_voltage = reinterpret_cast<float&>(x);
			printf("Battery voltage is: %.2f V\n", bat_voltage);			
			
			//Publish
			mrpt::slam::CObservationBatteryStatePtr battery_obs = mrpt::slam::CObservationBatteryState::Create();
			battery_obs->timestamp = now();
			battery_obs->voltageMainRobotBattery = bat_voltage;
			battery_obs->voltageMainRobotBatteryIsValid = true;
			//string sBattery = ObjectToString( battery_obs.pointer() );
			
			mrpt::vector_byte vec_bat;
			mrpt::utils::ObjectToOctetVector(battery_obs.pointer(), vec_bat);
			//!  @moos_publish   BATTERY_V   The curent battery level of the Mobile Robotic Base as an mrpt::CObservationBatteryState
			m_Comms.Notify("BATTERY_V", vec_bat );
		}
	}
	else
		cout << "[CRobotGiraffApp]: ERROR reading battery data from Giraff" << endl;
}