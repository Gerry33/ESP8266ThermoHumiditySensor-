#ifndef LOGGING_H
#define LOGGING_H
#include <inttypes.h>
#include <stdarg.h>

#include <WiFiUdp.h>

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

extern "C" {
}

typedef enum {
	LOGLEVEL_QUIET =  0,		// 0
	LOGLEVEL_ERROR =  1,
	LOGLEVEL_WARNING= 2,
	LOGLEVEL_INFO   = 3,
	LOGLEVEL_DEBUG  = 4,
	LOGLEVEL_MAX    = 5		// dummy
} LOGLEVEL_t;


#define LOGLEVEL_DEFAULT     LOGLEVEL_INFO

// return values for the several functions. After changing all function types from void to int this allows a better error handling for the log-functions themselves.

#define SIMPLE_LOG_SUCCESS         0
#define SIMPLE_LOG_STREAM_INVAL   -1
#define SIMPLE_LOG_LOGLEVEL_INVAL -2
#define SIMPLE_LOG_ARG_MISMATCH   -3

class SimpleLog {
private:
	LOGLEVEL_t 	_level = LOGLEVEL_INFO;
    WiFiUDP 	_udp; 				// http://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/udp-class.html
    char 		buf  	[ 200 ];
    IPAddress 	_syslogServer ;	// server
    int 		_state =0; 			// 0 = offline, 1 = online
    char 		_syslogPrefix [ 20 ]; // e.g hostname

public:
    SimpleLog() {};

    int begin	(LOGLEVEL_t level, IPAddress syslogServer );
    int begin	(LOGLEVEL_t level );
    int begin	();

    int setLevel(LOGLEVEL_t level);

    void setSysLogPrefix(const char *pf);

    void debug (String msg);
    void debug (const char *format, ...);

    void info (String msg);
    void info (const char *format, ...);

    void error ( String msg);
    void error ( const char *format, ...);

    int  startSyslog (IPAddress syslogServer);
    int  startSyslog (const char * hostname);

    int  stopSyslog ();

private:
    void  		 logIntern			(LOGLEVEL_t logLevel, const char* msg, va_list args);
    void  		 logIntern			(LOGLEVEL_t logLevel, String msg);
    void  		 sendUdpSyslog	( const char * msgtosend);

};

#endif




