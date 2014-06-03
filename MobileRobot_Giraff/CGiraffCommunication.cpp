	
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


#include "CGiraffCommunication.h"
#include <iostream>
#include <mrpt/system/os.h>

using namespace NAAS;
using namespace mrpt::utils;
using namespace std;


//-----------------------------------------------------------
//					 CGiraffCommunication
//-----------------------------------------------------------

CGiraffCommunication::CGiraffCommunication()
{	
	
	init();

	//writeDebugLine("Object well initialized",COMMUNICATION);
}


//-----------------------------------------------------------
//					  ~CGiraffCommunication
//-----------------------------------------------------------

CGiraffCommunication::~CGiraffCommunication()
{
	cout << "[INFO] Destroying Communication object..." << endl;

	if ( srvSocket )
		delete srvSocket;

	if ( socket )
		delete socket;

	cout << "[INFO] Communication object destroyed" << endl;
}


//-----------------------------------------------------------
//						    init
//-----------------------------------------------------------

bool CGiraffCommunication::init()
{
	//NAV_TRY

	m_ip	="127.0.0.1";// g_configFile.read_string("COMMUNICATION","ip","",true);
	m_port	=25557;// g_configFile.read_uint64_t("COMMUNICATION","port",0,true);

	srvSocket = new mrpt::utils::CServerTCPSocket(m_port,m_ip);
	run();

	return true;

	//NAV_CATCH_MODULE("In init method")

	//return false;
}


//-----------------------------------------------------------
//						    run
//-----------------------------------------------------------

bool CGiraffCommunication::run()
{
	//NAV_TRY

	if (srvSocket->isListening())
	{
		//writeDebugLine(format("Waiting external connection to ip:%s port:%u ...", m_ip.c_str(), m_port),COMMUNICATION);
		printf("Waiting external connection to ip:%s port:%u ...\n", m_ip.c_str(), m_port);
		socket = srvSocket->accept();		
		if( socket==NULL)
			printf("Connection not OK");
		socket->setTCPNoDelay(1);
		//socket.connect( ip, port );

		if ( socket->isConnected() )
		{
			//writeDebugLine("Connection established...",COMMUNICATION);
			printf("Connection established...");
			return true;
		}
	}

	//NAV_CATCH_MODULE("In start method")

	//return false;
}


//-----------------------------------------------------------
//						   write
//-----------------------------------------------------------

bool CGiraffCommunication::write(const std::string &str)
{
	//NAV_TRY

	char buf[128];
	mrpt::system::os::sprintf( buf, 128, str.c_str() );
	unsigned short toSend = str.length();

	socket->WriteBuffer( buf, toSend ); 

	//writeDebugLine( format("Writed: %s",string(buf).c_str()), COMMUNICATION );

	//socket.ReadBuffer( buf2, 5 );
	
	return true;

	//NAV_CATCH_MODULE("In write method")

	//return false;
}


//-----------------------------------------------------------
//						    read
//-----------------------------------------------------------

bool CGiraffCommunication::read(std::string &read)
{
	//NAV_TRY

	if (socket->getline( read ))
	{
		//writeDebugLine(format("Readed: %s", read.c_str()),COMMUNICATION); 
		//cout << "[CGiraffCommunication-read]: Readed line: " << read << endl;
		return true;
	}
	else
	{
		//writeDebugLine("EOF or any other read error while reading",COMMUNICATION);
		printf("\nEOF or any other read error while reading\n");
		delete socket;
		delete srvSocket;
		init();			//restart communication
		printf("\nSocket restarted OK\n");
		return false;
	}

	//NAV_CATCH_MODULE("In read method")

	//return false;
}


//-----------------------------------------------------------
//					 readVariousLines
//-----------------------------------------------------------

bool CGiraffCommunication::readVariousLines(vector<std::string> &read, const size_t &n_lines)
{
	//NAV_TRY

	read.clear();
	read.resize(n_lines);

	for ( size_t i_line = 0; i_line < n_lines; i_line++ )
	{
		if (socket->getline( read[i_line] ))
		{
			//writeDebugLine(format("Readed: %s", read[i_line].c_str()),COMMUNICATION); 
			return true;
		}
		else
		{
			//writeDebugLine("EOF or any other read error while reading",COMMUNICATION);
			return false;
		}
	}

	//NAV_CATCH_MODULE("In read method")

	//return false;
}