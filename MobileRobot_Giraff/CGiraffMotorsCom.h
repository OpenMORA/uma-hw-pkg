
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


#ifndef _CGiraffMotorsCom_
#define _CGiraffMotorsCom_


#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/synch/CCriticalSection.h>
#include "CGiraffCommunication.h"
#include <fstream>

namespace NAAS
{

	/** MotorsCom class: Implements all the possible commands to send to the
	  *		wheels motors controler.
	  */
	class CGiraffMotorsCom
	{
	private:

		bool m_simulated;	//!< We are working with the real motors or simulating them?

		/** Information about the simulation state.
		  */
		struct simulation
		{
			/** Constructor. */
			simulation(): x(0),y(0),phi(0), movingInStraighLine(true)
				{};

			double x;	//!< x robot coordinate.
			double y;	//!< y robot coordinate.
			double phi;	//!< robot orientation.
			bool movingInStraighLine;	//!< Moving in a straight line?
		}m_simulation;	//!< Store information about the robot pose if we are simulating.

		double m_vel;	//!< Current velocity.
		double m_w;  //!<Current angular velocity
		std::ofstream logFile;
		bool Save_logfile;

		mrpt::synch::CCriticalSection semaphore; //!< Semaphore for ensuring data consistency

		CGiraffCommunication* m_comms;	//!< Pointer to a CGiraffCommunication object for data exchange

	public:

		/** Constructor.
		  * \param simulated if the motors communication is simulated.
		  */
		CGiraffMotorsCom( const bool &simulated = false );

		/** Destructor */
		~CGiraffMotorsCom();

		/** Excute a command.
		  * \param command command to execute.
		  * \param param1 extra paramter maybe needed for the command to execute.
		  * \param param2 extra paramter maybe needed for the command to execute.
		  * \param param3 extra paramter maybe needed for the command to execute.
		  * \return true if any error occurred.
 		  */
		bool execute( const std::string &command, void *param1=NULL,
				void *param2=NULL, void *param3=NULL );
		/** Excute a command received from Giraff SW.
		 * \param command-> a String with the whole command from Giraff SW("set p...","set v...")
		 */
		bool executeGiraffCommand( std::string &command, std::string &response );

		/** Indicates wheter or not save a log file */
		void set_SaveLogfile(bool logfile_option);
		void saveToLogfile(std::string str);

		/** Get the robot acceleration */
		//bool getAcceleration( double &acc );

		/** Set the robot acceleration */
		//bool setAcceleration( const double &acc ) ;

	private:

		/** Module initialization */
		bool init( );

		/** Get the position of the robot */
		bool getPos( double &pos );
		/** Set the relative robot position */
		bool setPos( const double &pos );

		/** Get the robot velocity */
		bool getVel( double &vel );
		/** Set the robot velocity.
		  * \param vel new velocity
		  *	\param mode mode of the change. If 0, relative (the new velocity is the
		  *		sum of the vel parameter and the current one. If 1, absolute set.
		  */
		bool setVel( const double &vel,const unsigned int &mode );
		/** Get the robot acceleration */

		bool getAcceleration( double &acc );
		/** Set the robot acceleration */

		bool setAcceleration( const double &acc ) ;
		/** Get the current radius */
		bool getRadius( double &radius );
		/** Set the radius */
		bool setRadius( const double &radius );

		/** Get the current mode */
		bool getMode( unsigned int &mode );
		/** Set the mode */
		bool setMode( const unsigned int &mode );

		/** Initiates an undock movement. Go back 10 cm and rotates 180 degrees. */
		bool setUndock();

		/** The head motor performs a home tilt sequence */
		bool home();
		/** Get the state of a home tilt sequence */
		bool getTiltHomeState( size_t &state);
		/** Get the tilt angle from home */
		bool getTiltAngleFromHome( double &angle );
		/** Set the tilt angle from home */
		bool setTiltAngleFromHome( const double &angle );

		/** Get the maximum virtual gear ratio */
		bool getMaximumVirtualGearRatio( double &ratio );
		/** Set the maximum virtual gear ratio */
		bool setMaximumVirtualGearRatio( const double &ratio );

		/** Get the virtual gear rate of change */
		bool getVirtualGearRateOfChange( double &rate );
		/** Set the virtual gear rate of change */
		bool setVirtualGearRateOfChange( const double &rate );

		/** Get the clothoid deceleration point */
		bool getClothoidDecelerationPoint( double &point );
		/** Set the clothoid deceleration point */
		bool setClothoidDecelerationPoint( const double &point );

		/** Get robot odometry.
		  * \param left movement of the left wheel in meters.
		  *	\param right movement of the right wheel in meters.
		  */
		bool getOdometry( double &left, double &right );

		/** Get charger info */
		bool getChargerInfo( std::vector<std::string> &reponse );

		/** Get charger data */
		bool getChargerData( std::string &response );

		/** Get charger status */
		bool getChargerStatus( std::string &response );

		/** Show a list with the API wheels controler commands */
		bool get();

		/** Ends the communication with the term application. To test. */
		bool quit();

		/** Transmit a command */
		bool transmit( std::string &command );

		/** Receive the response to a command */
		bool receive( std::string &response);

		/** Receive a response formed by various lines. Not used at the moment */
		bool receiveVariousLines( std::vector<std::string> &response, const size_t &n_lines );
		/** test Speed */
		bool testSpeed();

	};

}

#endif


