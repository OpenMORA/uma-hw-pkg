	
	/*---------------------------------------------------------------
	|					NAAS Control Architecture					|
	|																|
	|			J.R. Ruiz-Sarmiento(jotaraul@uma.es)				|
	|		Department of Computer Engineering and Automatics.		|
	|			   MAPIR Group. University of M�laga				|
	|																|
	|							License:							|
	|	Creative Commons Attribution-NonCommercial-ShareAlike		|
	|	2.0 Generic (CC BY-NC-SA 2.0). Further information about	|
	|	this license here:											|
	|	http://creativecommons.org/licenses/by-nc-sa/2.0/			|
	|																|
	---------------------------------------------------------------*/


#ifndef _CGiraffCommunication_
#define _CGiraffCommunication_

#include <string>


#include <mrpt/utils/CServerTCPSocket.h>
#include <mrpt/utils/CClientTCPSocket.h>

using namespace mrpt::utils;

namespace NAAS
{
	/** Communication class. This clas open a TCP Server attached to a socket for
	  *		sending a retrieving data.
	  */
	class CGiraffCommunication  
	{
	private:

		std::string		m_ip;	//!< IP direction where we are going to open the port 
		unsigned short	m_port; //!< Port to open

		CServerTCPSocket *srvSocket; //!< Server
		CClientTCPSocket *socket;	//!< Client

	public:

		/** Constructor */
		CGiraffCommunication();

		/** Destructor */
		~CGiraffCommunication();

		/** Module initialization reading configuration parameters from file. 
		  * \return true if the initialization was successful, false in otherwise.	
		  */
		bool init();

		/** Wait for a connection in the specified ip and port.
		  * \return true if the client connection was successful, false in otherwise.	
		  */
		bool run();

		/** Write a string in the socket.
		  * \param str string to write.
		  * \return true if any error occurs.
		  */
		bool write( const std::string &str );

		/** Read a string from the socket.
		  * \param read strin readed.
		  * \return true if any error occurs.
		  */
		bool read(std::string &read);

		/** Read various lines from the socket.
		  * \param read vector with the lines read as strings.
		  * \param n_lines number of lines to read.
		  * \return true if any error occurs.
		  */
		bool readVariousLines(std::vector<std::string> &read, const size_t &n_lines);

	};
}


#endif