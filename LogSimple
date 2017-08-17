/*
 * based on ideas from
 *
 */
#include "SimpleLog.h"

/* NEW:
 * Log hierarchy: (not bit combinations any more)
 *
 *
 *
 * 		QUIET	0 	-lowest
 * 		ERROR	1
 * 		WARNING	2
 * 		INFO	3
 * 		DEBUG	4	-highest
 *
 * lower level include always higher levels:  e.g. DEBUG includes I, W, E
 *
 */
const char * prefix [ LOGLEVEL_MAX ] = {"", "E:", "W:", "I:", "D:"};

int  SimpleLog::begin(LOGLEVEL_t level, IPAddress syslogServer)
{
	_syslogServer = syslogServer;
	if ( _syslogServer )
		// _udp.begin(2390); not necessary: cause not listening
		_udp.beginPacket(syslogServer, 514);

	begin(level);
	return( SIMPLE_LOG_SUCCESS);
}


int  SimpleLog::begin(LOGLEVEL_t level)
{
	setLevel( level );	// errors must always be logged

	return( SIMPLE_LOG_SUCCESS);
}

int  SimpleLog::begin()
{
	setLevel( LOGLEVEL_INFO);	// default

	return( SIMPLE_LOG_SUCCESS);
}
int  SimpleLog::startSyslog (IPAddress syslogServer)
{
	_syslogServer = syslogServer;

	if ( _syslogServer ) {
		// _udp.begin(2390); not necessary: cause not listening
		_state = _udp.beginPacket(syslogServer, 514);
	}

	return( _state);
}

void SimpleLog::setSysLogPrefix(const char *pf) {

	strncpy( _syslogPrefix, pf, sizeof (_syslogPrefix) -2);
	strcat( _syslogPrefix, ":");
}

int  SimpleLog::startSyslog (const char * hostname)
{
	_state = _udp.beginPacket(hostname, 514);
	return( _state);
}


int  SimpleLog::stopSyslog () {

	_state = 0;
	_udp.stop();

	return( _state);
}


int SimpleLog::setLevel(LOGLEVEL_t newLevel)
{
	int retVal = SIMPLE_LOG_SUCCESS;

	if( newLevel >= LOGLEVEL_QUIET &&  newLevel < LOGLEVEL_MAX)
		_level = newLevel;
	else
		retVal = SIMPLE_LOG_LOGLEVEL_INVAL;

	return( retVal );
}

void  SimpleLog::logIntern(LOGLEVEL_t logLevel, const char* msg, va_list args)
{


	strcpy (buf, prefix [logLevel]);

	vsnprintf_P (&buf[2], sizeof(buf)-2 , msg, args );
	Serial.printf ( buf );
	sendUdpSyslog ( buf );
}

void  SimpleLog::logIntern(LOGLEVEL_t logLevel, String msg)
{
	msg = String(prefix [logLevel]) + msg ;
	Serial.println(msg);
	sendUdpSyslog (msg.c_str());

}

//  ------------------------
void SimpleLog::debug (const char * msg, ...) {

	if( _level >= LOGLEVEL_DEBUG ) {
		va_list args;
		va_start(args, msg); // store all argumenet after 'msg'
		logIntern ( LOGLEVEL_DEBUG, msg, args);
		va_end(args );
	}
}


void  SimpleLog::debug (String msg){
	logIntern( LOGLEVEL_DEBUG, msg);
}

//  --------------------------------------------------------------

void SimpleLog::info (const char * msg, ...) {

	if( _level >= LOGLEVEL_INFO) {
		va_list args;
		va_start(args, msg); 	// store all argument after 'msg'
		logIntern ( LOGLEVEL_INFO, msg, args);
		va_end(args );
	}

}

void SimpleLog::info (String msg){
	logIntern( LOGLEVEL_INFO, msg);
}

// --------------------------------------------------------------------------
void SimpleLog::error (String msg){
	logIntern( LOGLEVEL_ERROR, msg);
}

// error always
void SimpleLog::error (const char * msg, ...) {

		va_list args;
		va_start(args, msg); 	// store all argument after 'msg'
		logIntern ( LOGLEVEL_ERROR, msg, args);
		va_end(args );

}
//  --------------------------------------------------------------

void SimpleLog::sendUdpSyslog(const char * msgtosend)
{
	if  (_state) {		// != 0
	    _udp.write(_syslogPrefix);
		_udp.write(msgtosend);
		_udp.endPacket();
	}
}
