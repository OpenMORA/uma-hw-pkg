#include "CommonData.h"

using namespace mrpt::synch;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace NAAS;
using namespace mrpt::utils;


COdometry					NAAS::g_odometry;
//-----------------------------------------------------------
//						setModules
//-----------------------------------------------------------

void CData::setModules()
{
	/*const TUsedModules &modules = commonData.modulesUsed;

	if (modules.reactNavigator)	m_fresh[REAC_NAV] = false;
	if (modules.navigator)		m_fresh[SIMPLE_NAV] = false;
	if (modules.localizator)	m_fresh[LOCALIZATOR] = false;
	if (modules.grabber)		m_fresh[GRABBER] = false;
	if (modules.sharer)			m_fresh[SHARER] = false;
	if (modules.motorsCom)		m_fresh[MOTORS] = false;
	if (modules.communication)	m_fresh[COMMUNICATION] = false;
	if (modules.localizationFusion)	m_fresh[LOCALIZATION_FUSION] = false;
	if (modules.autoDocking)	m_fresh[AUTODOCKING] = false;
	if (modules.wifiControler)	m_fresh[WIFICONTROLER] = false;
	if (modules.viewer)			m_fresh[VIEWER] = false;
	if (modules.server)			m_fresh[SERVER] = false;
	if (modules.server_mosq)			m_fresh[SERVER_MOSQ] = false;*/
}


//-----------------------------------------------------------
//						setAllFresh
//-----------------------------------------------------------

void CData::setAllFresh()
{
	/*const TUsedModules &modules = commonData.modulesUsed;

	if (modules.reactNavigator)	m_fresh[REAC_NAV] = true;
	if (modules.navigator)		m_fresh[SIMPLE_NAV] = true;
	if (modules.localizator)	m_fresh[LOCALIZATOR] = true;
	if (modules.grabber)		m_fresh[GRABBER] = true;
	if (modules.sharer)			m_fresh[SHARER] = true;
	if (modules.motorsCom)		m_fresh[MOTORS] = true;
	if (modules.communication)	m_fresh[COMMUNICATION] = true;
	if (modules.localizationFusion)	m_fresh[LOCALIZATION_FUSION] = true;
	if (modules.autoDocking)	m_fresh[AUTODOCKING] = true;
	if (modules.wifiControler)	m_fresh[WIFICONTROLER] = true;
	if (modules.viewer)			m_fresh[VIEWER] = true;
	if (modules.server)			m_fresh[SERVER] = true;
	if (modules.server_mosq)	    m_fresh[SERVER_MOSQ] = true;*/
}


