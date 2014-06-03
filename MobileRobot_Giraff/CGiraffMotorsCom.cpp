	
	/*---------------------------------------------------------------
	|					NAAS Control Architecture					|
	|																|
	|			J.R. Ruiz-Sarmiento(jotaraul@uma.es)				|
	|		Department of Computer Engineering and Automatics.		|
	|			   MAPIR Group. University of Málaga				|
	|																|
	|							License:							|
	|	Creative Commons Attribution-NonCommercial-ShareAlike		|
	|	2.0 Generic (CC BY-NC-SA 2.0). Further information about	|
	|	this license here:											|
	|	http://creativecommons.org/licenses/by-nc-sa/2.0/			|
	|																|
	---------------------------------------------------------------*/


#include "CGiraffMotorsCom.h"
#include "CRobotGiraffApp.h"

using namespace NAAS;
using namespace std;
using namespace mrpt::utils;


//-----------------------------------------------------------
//					     CGiraffMotorsCom
//-----------------------------------------------------------

CGiraffMotorsCom::CGiraffMotorsCom( const bool &simulated )
	: m_vel(0), m_simulated(simulated)
{
	  
	logFile.open ("logFile.txt");

	init();
}


//-----------------------------------------------------------
//					     ~CGiraffMotorsCom
//-----------------------------------------------------------

CGiraffMotorsCom::~CGiraffMotorsCom()
{
	cout << "[INFO] Destroying MotorsCom object..." << endl;

	cout << "[INFO] MotorsCom object destroyed" << endl;
}


//-----------------------------------------------------------
//					        init
//-----------------------------------------------------------

bool CGiraffMotorsCom::init( )
{
	//NAV_TRY

	if ( !m_simulated )
	{

		m_comms=new NAAS::CGiraffCommunication();
		mrpt::system::sleep(1000);
		// Giraff motors send a message when you connect to them
		string retrieveStartingMessage;		
		receive(retrieveStartingMessage);

		//writeDebugLine(retrieveStartingMessage, MOTORS);

		size_t homing_state;
		getTiltHomeState( homing_state );

		while ( homing_state != 3 )
		{
			getTiltHomeState( homing_state );
			if ( homing_state == 2 )
			{
				throw std::logic_error("Error while performing the Tilt Homing task");
			}
			mrpt::system::sleep(0.1);
			
		}
		mrpt::system::sleep(5000);
		double tilt;
		getTiltAngleFromHome( tilt );

		// Set initial vel from config file
		double initial_vel = 0;//g_configFile.read_double ("MOTORS","initial_vel",0,true);
		unsigned int initial_mode=2;
		setVel( initial_vel,initial_mode );

		m_vel = initial_vel;

		//writeDebugLine(format("Initial velocity: %f, initial tilt angle from home: %f",m_vel,tilt), MOTORS);

		return true;		
	}
	else
	{	
		printf("simulateeeeeeedddd");
		mrpt::system::sleep(10000);
		//writeDebugLine("Initialized simulated motors", MOTORS);
		return true;
	}
	
	//NAV_CATCH_MODULE("In init method")

	//return false;
}


//-----------------------------------------------------------
//					      execute
//-----------------------------------------------------------

bool CGiraffMotorsCom::execute( const string &command, void *param1, void *param2, void *param3 )
{
	//NAV_TRY

	if ( !m_simulated )
	{
		if ( command == "getPos" )
			return getPos( *(static_cast<double*>(param1)) );
		else if ( command == "setPos" )
			return setPos( *(static_cast<const double*>(param1)) );
		else if ( command == "getVel" )
			return getVel( *(static_cast<double*>(param1)) );
		else if ( command == "setVel" )
		{
			if ( param2 )
				return setVel( *(static_cast<const double*>(param1))
							, *(static_cast<const unsigned int*>(param2)));
			/*else
				return setVel( *(static_cast<const double*>(param1)));*/
		}
		else if ( command == "getAcceleration" )
			return	getAcceleration( *(static_cast<double*>(param1)) );
		else if ( command == "setAcceleration" )
			return setAcceleration( *(static_cast<const double*>(param1)) );
		else if ( command == "getRadius" )
			return	getRadius( *(static_cast<double*>(param1)) );
		else if ( command == "setRadius" )
			return setRadius( *(static_cast<const double*>(param1)) );
		else if ( command == "getMode" )
			return getMode( *(static_cast<size_t*>(param1)) );
		else if ( command == "setMode" )
			return setMode( *(static_cast<const unsigned int*>(param1)) );
		else if ( command == "setUndock" )
			return setUndock();
		else if ( command == "home" )
			return home();
		else if ( command == "getTiltHomeState" )
			return getTiltHomeState( *(static_cast<size_t*>(param1)) );
		else if ( command == "getTiltAngleFromHome" )
			return getTiltAngleFromHome( *(static_cast<double*>(param1)) );
		else if ( command == "setTiltAngleFromHome" )
			return setTiltAngleFromHome( *(static_cast<const double*>(param1)) );
		else if ( command == "getMaximumVirtualGearRatio" )	
			return getMaximumVirtualGearRatio( *(static_cast<double*>(param1)) );
		else if ( command == "setMaximumVirtualGearRatio" )
			return setMaximumVirtualGearRatio( *(static_cast<const double*>(param1)) );
		else if ( command == "getVirtualGearRateOfChange" )
			return getVirtualGearRateOfChange( *(static_cast<double*>(param1)) );
		else if ( command == "setVirtualGearRateOfChange" )
			return setVirtualGearRateOfChange( *(static_cast<const double*>(param1)) );
		else if ( command == "getClothoidDecelerationPoint" )
			return getClothoidDecelerationPoint( *(static_cast<double*>(param1)) );
		else if ( command == "setClothoidDecelerationPoint" )
			return setClothoidDecelerationPoint( *(static_cast<const double*>(param1)) );
		else if ( command == "getOdometry" )
			return getOdometry( *(static_cast<double*>(param1)),
								*(static_cast<double*>(param2)) );
		else if ( command == "getChargerInfo" )
			return getChargerInfo( *(static_cast<vector<string>*>(param1)) );
		else if ( command == "getChargerData" )
			return getChargerData( *(static_cast<string*>(param1)) );
		else if ( command == "getChargerStatus" )
			return getChargerStatus( *(static_cast<string*>(param1)) );
		else if ( command == "get" )
			return get();
		else if ( command == "quit" )
			return quit();		
		else
			printf("ERROR! unknow command received...");
		//	writeDebugLine("ERROR! unknow command received...", MOTORS );
	}
	else // SIMULATED!!!
	{
		if ( command == "setPos" )
		{
			double inc;

			if ( m_simulation.movingInStraighLine )
			{
				inc = *(static_cast<double *>(param1));
				m_simulation.x += inc*cos( m_simulation.phi );
				m_simulation.y += inc*sin( m_simulation.phi );
			}
			else // turning
			{
				inc = *(static_cast<double *>(param1));
				m_simulation.phi -= DEG2RAD( inc );
			}

		//	writeDebugLine(format("Sended setPos commanded used simulated motors with value: %f", inc),MOTORS);

			return true;			
		}
		else if ( command == "setRadius" )
		{
			//writeDebugLine("Sended setRadius commanded used simulated motors",MOTORS);

			double radius = *(static_cast<double *>(param1));

			if ( radius == 0 )
				m_simulation.movingInStraighLine = false;
			else if ( radius > 50 )
				m_simulation.movingInStraighLine = true;				
			
			return true;
		}
		else if ( command == "getOdometry" )
		{
			double &p1 = *(static_cast<double *>(param1));
			double &p2 = *(static_cast<double *>(param2));
			double &p3 = *(static_cast<double *>(param3));
			p1 = m_simulation.x;
			p2 = m_simulation.y;
			p3 = m_simulation.phi;
			return true;		
		}
		else if ( command == "getVel" )
		{
			double &vel = *(static_cast<double *>(param1));
			vel = 0.5;
			return true;
		}
		
		return true;
	}

	//NAV_CATCH_MODULE("In execute method")

	//return false;
}
//-----------------------------------------------------------
//						executeGiraffCommand	
//-----------------------------------------------------------
bool CGiraffMotorsCom::executeGiraffCommand( string &command, string &response ) 
{
	//NAV_TRY
	string resp=response;
	// '|' character is used as command delimiter
	//command.append("|");

	size_t written = m_comms->write( command );
	//printf("Giraff command sent: %s",command.c_str());
	if (!written){
		return false;
	}
	size_t read;
	while (resp==response){
		read= m_comms->read(resp);
		
	}
	response=read;
	//cout << command << endl;
	//cout << "Sended: " << command << endl;

	//writeDebugLine(format("Sended: %s",command.c_str()),MOTORS);

	return true;

	//NAV_CATCH_MODULE("In transmit method")

	//return false;
}
//-----------------------------------------------------------
//					       getPos
//-----------------------------------------------------------

bool CGiraffMotorsCom::getPos( double &pos ) 
{
	//NAV_TRY

	string command("get p");
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	pos = atof(response.c_str());
	
	return true;

	//NAV_CATCH_MODULE("In method getPos")

	//return false;
}


//-----------------------------------------------------------
//					       setPos
//-----------------------------------------------------------

bool CGiraffMotorsCom::setPos( const double &pos ) 
{
	//NAV_TRY


	string command( mrpt::format( "set p %.03f",pos) );
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;
	
	return true;

	//NAV_CATCH_MODULE("In method setPos")

	//return false;
}


//-----------------------------------------------------------
//					       getVel
//-----------------------------------------------------------

bool CGiraffMotorsCom::getVel( double &vel ) 
{
	//NAV_TRY

	string command("get v");
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	vel = atof(response.c_str());
	
	return true;

	//NAV_CATCH_MODULE("In method getVel")

	//return false;
}


//-----------------------------------------------------------
//					     setVel
//-----------------------------------------------------------

bool CGiraffMotorsCom::setVel( const double &vel,const unsigned int &mode ) 
{
	//NAV_TRY

	string command;
	if ( mode == 0 ) // Relative mode
		command = mrpt::format( "set v %.03f", m_vel + vel);
	else if ( mode == 1 ) // Absolute mode
	{
		command = mrpt::format( "set v %.03f",vel);
		m_vel = vel;
	}
	else if ( mode == 2 ) // Absolute mode
	{
		command = mrpt::format( "set v %.03f",vel);
		m_vel = vel;
	}
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;
	
	//m_vel += vel;

	return true;

	//NAV_CATCH_MODULE("In method setVel")

	//return false;
}
//-----------------------------------------------------------
//					       getAcceleration
//-----------------------------------------------------------

bool CGiraffMotorsCom::getAcceleration( double &acc) 
{
	//NAV_TRY

	string command("get a");
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	acc = atof(response.c_str());
	
	return true;

	//NAV_CATCH_MODULE("In method getAcceleration")

	//return false;
}

//-----------------------------------------------------------
//					     setAcceleration
//-----------------------------------------------------------

bool CGiraffMotorsCom::setAcceleration( const double &acc ) 
{
	//NAV_TRY

	string command;
	
		command = mrpt::format( "set a %.03f",acc);
	
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;
	
	
	return true;

	//NAV_CATCH_MODULE("In method setAcceleration")

	//return false;
}

//-----------------------------------------------------------
//					    getRadius
//-----------------------------------------------------------

bool CGiraffMotorsCom::getRadius( double &radius ) 
{
	//NAV_TRY

	string command("get r");
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	radius = atof(response.c_str());
	
	return true;	

	//NAV_CATCH_MODULE("In method getRadius")

	//return false;
}


//-----------------------------------------------------------
//					     setRadius
//-----------------------------------------------------------

bool CGiraffMotorsCom::setRadius( const double &radius ) 
{
	//NAV_TRY

	string command( mrpt::format( "set r %.03f",radius) );
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;
	
	return true;

	//NAV_CATCH_MODULE("In method setRadius")

	//return false;
}


//-----------------------------------------------------------
//					      getMode
//-----------------------------------------------------------

bool CGiraffMotorsCom::getMode( unsigned int &mode ) 
{
	//NAV_TRY

	string command("get mode");
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	mode = atoi(response.c_str());
	
	return true;	

	//NAV_CATCH_MODULE("In method getMode")

	//return false;
}


//-----------------------------------------------------------
//					      setMode
//-----------------------------------------------------------

bool CGiraffMotorsCom::setMode( const unsigned int &mode ) 
{
	//NAV_TRY
	unsigned int set_mode = mode;

	string command( mrpt::format( "set mode %d",set_mode) );
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;
	
	return true;

	//NAV_CATCH_MODULE("In method setMode")

	//return false;
}


//-----------------------------------------------------------
//					     setUndock
//-----------------------------------------------------------

bool CGiraffMotorsCom::setUndock( )
{
	//NAV_TRY

	string command( "set undock" ) ;
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;
	
	return true;

	//NAV_CATCH_MODULE("In method setUndock")

	//return false;
}


//-----------------------------------------------------------
//			  		        home
//-----------------------------------------------------------

bool CGiraffMotorsCom::home()
{
	//NAV_TRY

	string command( "home" );
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;
	
	return true;

	//NAV_CATCH_MODULE("In method home")

	//return false;
}


//-----------------------------------------------------------
// 				      getTiltHomeState
//-----------------------------------------------------------

bool CGiraffMotorsCom::getTiltHomeState( size_t &state)
{
	//NAV_TRY

	string command("get tilt_homing_state");
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	state = atoi(response.c_str());
	
	return true;		

	//NAV_CATCH_MODULE("In method getTiltHomeState")

	//return false;
}


//-----------------------------------------------------------
//				    getTiltAngleFromHome
//-----------------------------------------------------------

bool CGiraffMotorsCom::getTiltAngleFromHome( double &angle )
{
	//NAV_TRY

	string command("get tilt_angle_from_home");
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	angle = atof(response.c_str());
	
	return true;	

	//NAV_CATCH_MODULE("In method getTiltAngleFromHome")

	//return false;

}


//-----------------------------------------------------------
//					setTiltAngleFromHome
//-----------------------------------------------------------

bool CGiraffMotorsCom::setTiltAngleFromHome( const double &angle )
{
	//NAV_TRY

	string command( mrpt::format( "set tilt_angle_from_home %f", angle ) );
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;
	
	return true;

	//NAV_CATCH_MODULE("In method setTiltAngleFromHome")

	//return false;
}


//-----------------------------------------------------------
//				getMaximumVirtualGearRatio
//-----------------------------------------------------------

bool CGiraffMotorsCom::getMaximumVirtualGearRatio( double &ratio )
{
	//NAV_TRY

	string command("get vg");
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	ratio = DEG2RAD(atof(response.c_str()));
	
	return true;	

	//NAV_CATCH_MODULE("In method getMaximumVirtualGearRatio")

	//return false;
}


//-----------------------------------------------------------
//				setMaximumVirtualGearRatio
//-----------------------------------------------------------

bool CGiraffMotorsCom::setMaximumVirtualGearRatio( const double &ratio )
{
	//NAV_TRY

	string command( mrpt::format( "set vg %f", ratio) );
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;
	
	return true;

	//NAV_CATCH_MODULE("In method setMaximumVirtualGearRatio")

	//return false;
}


//-----------------------------------------------------------
//				getVirtualGearRateOfChange
//-----------------------------------------------------------

bool CGiraffMotorsCom::getVirtualGearRateOfChange( double &rate )
{
	//NAV_TRY

	string command("get vgr");
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	rate = atof(response.c_str());
	
	return true;	

	//NAV_CATCH_MODULE("In method getVirtualGearRateOfChange")

	//return false;
}


//-----------------------------------------------------------
//				setVirtualGearRateOfChange
//-----------------------------------------------------------

bool CGiraffMotorsCom::setVirtualGearRateOfChange( const double &rate )
{
	//NAV_TRY

	string command( mrpt::format( "set vgr %0.3f", rate) );
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;
	
	//return true;

	//NAV_CATCH_MODULE("In method setVirtualGearRateOfChange")

	return false;
}


//-----------------------------------------------------------
//				getClothoidDecelerationPoint
//-----------------------------------------------------------

bool CGiraffMotorsCom::getClothoidDecelerationPoint( double &point )
{
	//NAV_TRY

	string command("get cdp");
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	point = atof(response.c_str());
	
	return true;	

	//NAV_CATCH_MODULE("In method getClothoidDecelerationPoint")

	//return false;
}
		

//-----------------------------------------------------------
//				setClothoidDecelerationPoint
//-----------------------------------------------------------

bool CGiraffMotorsCom::setClothoidDecelerationPoint( const double &point )
{
	//NAV_TRY

	string command( mrpt::format( "set cdp %f", point) );
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;
	
	return true;

	//NAV_CATCH_MODULE("In method setClothoidDecelerationPoint")

	//return false;
}


//-----------------------------------------------------------
//					    getOdometry
//-----------------------------------------------------------

bool CGiraffMotorsCom::getOdometry( double &left, double &right )
{
	//NAV_TRY

	string command("get imdl");
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	left = atof(response.c_str());

	command.clear();
	command = "get imdr";
	response.clear();
	
	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	right = atof(response.c_str());

	return true;	

	//NAV_CATCH_MODULE("In getOdometry")

	//return false;
}


//-----------------------------------------------------------
//					    getChargerInfo
//-----------------------------------------------------------

bool CGiraffMotorsCom::getChargerInfo( vector<string> &response )
{
	//NAV_TRY

	string command("t");

	const size_t n_lines = 8;
	
	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receiveVariousLines(response,n_lines)) ) return false;

	return true;	

	//NAV_CATCH_MODULE("In method getChargerInfo method")

	//return false;
}


//-----------------------------------------------------------
//					    getChargerStatus
//-----------------------------------------------------------

bool CGiraffMotorsCom::getChargerStatus( string &response )
{
	//NAV_TRY

	string command("get charger_data");

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	return true;	

	//NAV_CATCH_MODULE("In method getChargerStatus method")

	//return false;
}


//-----------------------------------------------------------
//							get
//-----------------------------------------------------------

bool CGiraffMotorsCom::get()
{
	//NAV_TRY

	string response;

	string command("get");

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	return true;	

	//NAV_CATCH_MODULE("In method get")

	//return false;
}




//-----------------------------------------------------------
//					    getChargerData
//-----------------------------------------------------------

bool CGiraffMotorsCom::getChargerData( string &response )
{
	//NAV_TRY
	
	string command("get charger_data");		
	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;

	return true;	

	//NAV_CATCH_MODULE("In method getChargerData method")

	//return false;
}

//-----------------------------------------------------------
//					      quit
//-----------------------------------------------------------

bool CGiraffMotorsCom::quit()
{
	//NAV_TRY
	
	string command( "q" );
	string response;
	logFile.close();
	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;
	
	return true;

	//NAV_CATCH_MODULE("In method quit")

	//return false;
}


//-----------------------------------------------------------
//					     transmit
//-----------------------------------------------------------

bool CGiraffMotorsCom::transmit( string &command ) 
{
	//NAV_TRY

	// '|' character is used as command delimiter
	command.append("|");
	string str;
	size_t written = m_comms->write( command );
	//cout << "[CGiraffMotorsCom]: Command written is: " << command << endl;
	str=mrpt::format("Tx: %s time: %s \n",command.c_str(),MOOSGetTimeStampString().c_str());
	logFile.write(str.c_str(),str.size());
 // myfile.close();
	if (!written){
		str=mrpt::format("Command could not be written \n");
			logFile.write(str.c_str(),str.size());	
		return false;
	}	
	return true;

	//NAV_CATCH_MODULE("In transmit method")

	//return false;
}

//-----------------------------------------------------------
//					     receive
//-----------------------------------------------------------

bool CGiraffMotorsCom::receive(string &response) 
{
	//NAV_TRY

	// Read response
	string str;
	if (!m_comms->read( response ))
	{
		printf("\nCommand could not be received.\n");
		str=mrpt::format("Command could not be received. time: %s \n",MOOSGetTimeStampString().c_str());
		logFile.write(str.c_str(),str.size());	
		return false;	
	}
	//writeDebugLine(format("Response: %s",response.c_str()),MOTORS);
	
	//cout << endl << "[CGiraffMotorsCom]: Response: " << response << endl;

	// Readget w OK> prompted message
	string OK;
	m_comms->read( OK );
	str=mrpt::format("Rx: %s time: %s\n",OK.c_str(),MOOSGetTimeStampString().c_str());
	logFile.write(str.c_str(),str.size());
	return true;

	//NAV_CATCH_MODULE("In receive method")

	//return false;

}


//-----------------------------------------------------------
//				  receiveVariousLines
//-----------------------------------------------------------

bool CGiraffMotorsCom::receiveVariousLines( vector<string> &response, const size_t &n_lines ) 
{
	//NAV_TRY	

	response.clear();
	response.resize( n_lines );
	
	// Read response
	for ( size_t i_line = 0; i_line < n_lines; i_line++ )
		m_comms->read( response[i_line] );

	for ( size_t i_line = 0; i_line < n_lines; i_line++ )
	//	writeDebugLine(format("Response: %s",response[i_line].c_str()),MOTORS);
	
	return true;

	//NAV_CATCH_MODULE("In receive method")

	//return false;

}
//-----------------------------------------------------------
//				  testSpeed
//-----------------------------------------------------------

bool CGiraffMotorsCom::testSpeed() 
{
	//NAV_TRY	

    string command( "test_speed" );
	string response;

	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	if ( ( !transmit(command) ) || (!receive(response)) ) return false;
	
	return true;


	//NAV_CATCH_MODULE("In receive method")

	//return false;

}

