	
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

#ifndef _CCOMMONDATA_
#define _CCOMMONDATA_

#include <string>

#include <mrpt/system.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservationWirelessPower.h>
#include <mrpt/synch.h>
#include <mrpt/poses.h>
#include <mrpt/utils.h>


using namespace std;

namespace NAAS
{
/////////////////// CData class ///////////////////

	class CData
	{
	public:

		mrpt::system::TTimeStamp	m_time;	//!< Time when the data was updated
		//mrpt::synch::CCriticalSection mutex;
		mrpt::synch::CSemaphore		m_semaphore;	//!< Semaphore for ensuring data consistency
		std::map<string,bool> m_fresh;		//!< Is fresh the data for a certain module?

		/** Constructor. */
		CData():m_semaphore(1,1)
		{ 
			m_time = mrpt::system::now(); 
		}

		/** Constructor. */
		CData( const mrpt::system::TTimeStamp &t ) : m_time( t ), m_semaphore(1,1)
		{}

		/** Destructor. */
		~CData()
		{}

		/** Set the m_fresh map with the modules that were launched. */
		void setModules();

		/** Set the data as fresh for all the used modules */
		void setAllFresh();

		/** Get the age of the data */
		inline double getAge( const mrpt::system::TTimeStamp &now ) 
		{ 
			return mrpt::system::timeDifference( now, m_time ); 
		}

		/** Is fresh the data for a certain module? */
		inline bool isFresh(const string &module)
		{ 
			return m_fresh[module]; 
		}

		/** Set the freshness of the data for a certain module. */ 
		inline void setFresh( const string &module, const bool &newVaule )
		{ 
			m_fresh[module] = newVaule; 
		}

		/** Working with the data! */
		inline void working()
		{ 
			m_semaphore.waitForSignal(); 
		}

		/** Work finished with the data */
		inline void endWorking()
		{ 
			m_semaphore.release(); 
		}

	};





/////////////////// COdometry ///////////////////

	/** This class stores the robot localization using only odometry values. Maybe the computation of
	  * such a localization should be done here, and not in the CSharer module...
	  */
	class COdometry : public CData
	{
		mrpt::poses::CPose2D m_curPose;	//!< Current robot pose computed only by odometry.

	public:

		/** Set robot localization in accordance with the odometry. */
		void setOdometry ( const mrpt::poses::CPose2D &cur_pose , const mrpt::system::TTimeStamp &now )
		{ 
			m_curPose = cur_pose; 
			m_time = now; 
			setAllFresh(); 
		}

		/** Get robot localization in accordance with the odometry. */
		void getOdometry ( mrpt::poses::CPose2D &cur_pose, mrpt::system::TTimeStamp &time )
		{ 
			cur_pose = m_curPose; 
			time =  m_time; 
		}

		/** Get robot localization in accordance with the odometry as a CPose2D. */
		mrpt::poses::CPose2D getAsPose2D()
		{ 
			return m_curPose; 
		}

	};




/////////////////// GLOBAL VARIABLES EXTERN DECLARATION ///////////////////

	extern COdometry			g_odometry;			//!< Global object for storing the last localization computed using only odometry data.

}

#endif _CCOMMONDATA_

