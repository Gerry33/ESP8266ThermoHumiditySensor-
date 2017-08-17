/**
  A simple temperature  & humidity program to collect temp & hum in a regular intervall.
  
  	 - forward to MQTT
  	 - determine 24H/max/min
  	 - simple WEB GUI for configuration
  	 - supports SI7021, DHT, DS18X  sensors
	 

 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include <AsyncMqttClient.h>			// https://github.com/marvinroger/async-mqtt-client
#include <ESP8266HTTPUpdateServer.h> 	// https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266HTTPUpdateServer
#include <EEPROM.h>					 	// https://github.com/esp8266/Arduino/tree/master/libraries/EEPROM
#include <TimeLib.h>					// https://github.com/PaulStoffregen/Time
#include <NtpClientLib.h>	    		// https://github.com/gmag11/NtpClient
#include <TimeAlarms.h>					// https://github.com/PaulStoffregen/TimeAlarms
#include "SSD1306.h"					// https://github.com/squix78/esp8266-oled-ssd1306
#include <DHT.h>				 		// https://github.com/markruys/arduino-DHT/

#include "Common.h"

// Start defines

// #define DHTPIN       12	// D6, gpio 12
#define SI7021ADDR 0x40 // SI7021 I2C address is 0x40(64)
// #define ONE_WIRE_BUS_PIN   13	// DS18...  Sensor : D7, gpio 13

//END defines ----------------------------------------------------------------------

#ifdef SI7021ADDR
// Include the correct display library
// For a connection via I2C using Wire include
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#define SDA 12
#define SCL 13
#endif

ESP8266HTTPUpdateServer httpUpdater;
MDNSResponder 			mdns;
ESP8266WebServer 		server(80);
String 					webPage;

// Initialize the OLED display using Wire library
// SSD1306  display(0x3c, D3, D5);	// D3 = GPIO 0, D5 = GPIO 14
// SDA, SCL

SSD1306  display(0x3c, SDA,   SCL);	// D3 = GPI4 0, D5 = GPIO 14

//  SH1106 display(0x3c, D3, D5);

#ifdef DHTPIN

// #define DHTTYPE DHT11 // DHT 11		// DHT.h
// #define DHTTYPE DHT21 // DHT 21 (AM2301)
#define DHTTYPE  DHT::DHT22 				// Sensor type DHT11/21/22/AM2301/AM2302
DHT dht;

#endif

#ifdef  ONE_WIRE_BUS_PIN

// wiring:http://iot-playground.com/blog/2-uncategorised/41-esp8266-ds18b20-temperature-sensor-arduino-ide
#include <OneWire.h> // https://github.com/milesburton/Arduino-Temperature-Control-Library
#include <DallasTemperature.h>

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
#endif

TempStatus  ts [ OFS_LAST ];
TempStatusType * displaySensor = &ts[OFS_SI702];	// the sensor to display: SI 7021  sensor for this display

// mqtt ++++
const char* 	mqtt_server = "server";		// 10.0.0.102 
AsyncMqttClient mqttClient;

char 			localHostname [30];
const char* 	ssid     = "yourSSID";
const char* 	password = "yourPW";


static 			WiFiEventHandler WiFiConnectHandler, WiFiDisConnectHandler;

AlarmId  		timerId_MqttConnect = dtINVALID_ALARM_ID;
AlarmId  		timerId_TempReading = dtINVALID_ALARM_ID;
AlarmId  		timerId_ResetMaxMin = dtINVALID_ALARM_ID;		// // no date in timer as its only a daily

// when the openhab (OH) server starts, OH sends this INIT to tell all client to publish their state to OH
const 	char * 	initTopic  = "INIT";

// NTP  ---------------------------------------------------------------------
boolean 		NtpSyncEventTriggered = false; 	// True if a time even has been triggered
NTPSyncEvent_t 	ntpEvent; 						// Last triggered event
bool 			ntpInit= false; 				// currently no better idea

String text_html; 	// to EE only at runtime
String statusLine ;
// --------------------------------------------------------------
void EEPromWriteConfig (){

	EEPROM.begin(sizeof (Config));
	Serial.println("Writing EE " + String (sizeof (Config)) + " bytes");
	byte  * p = (byte *) & config;
	for (uint i  = 0; i < sizeof (Config); i++ ){
		EEPROM.write(i, *p++);
	}
	EEPROM.commit();
}

void EEPromReadConfig (){

	EEPROM.begin(sizeof (Config));

	byte  * p = (byte *) & config;
	Serial.println("Reading EE " + String (sizeof (Config)) + " bytes");

	for (uint i  = 0; i < sizeof (Config); i++ ){
		*p++ = EEPROM.read(i);
	}
}

// Start only after IP network is connected

void onWIFIConnectedGotIP(WiFiEventStationModeGotIP ipInfo) {

	Serial.println("Connected:IP:" + ipInfo.ip.toString() + ", WifiState:" + String (WiFi.status()));
	// if different, save IP and write to ee.
	if ( strcmp(config.localIP, WiFi.localIP().toString().c_str())){
		strcpy (config.localIP,  WiFi.localIP().toString().c_str());
		EEPromWriteConfig();
	}

	NTP.begin("pool.ntp.org", 1, true);
	NTP.setInterval(3600);	 // 1hr	, 1 day 86400

	strncpy (localHostname, WiFi.hostname().c_str(), min (sizeof (localHostname), strlen(WiFi.hostname().c_str() )) );

	server.begin(); 			// Start the server
	Serial.write("Server started. Hostname:<");
	Serial.write(localHostname);
	Serial.write(">, IP<");
	Serial.write (config.localIP);
	Serial.write (">\n");

	if (!mdns.begin("esp8266", WiFi.localIP()))
		Serial.println(F("MDNS responder ERROR"));

	httpUpdater.setup(&server);
	Serial.println("HTTPUpdateServer on http://" + WiFi.localIP().toString() + "/update");

	server.begin();
	MDNS.addService("http", "tcp", 80);
	mqtt_Timed_Connect();	// init the mqtt connection setup directly. donn't use timers as we're connected

}

// Manage network disconnection
void onWifiDisconnected(WiFiEventStationModeDisconnected event_info) {
	Serial.println("WLAN Disconnected from SSID " + event_info.ssid);
	Serial.printf("Reason: %d\n", event_info.reason);
	NTP.stop();
	ntpInit 	= false;
	if (Alarm.isAllocated ( timerId_MqttConnect ) ) {	// without wifi it makes no sense to connect
		Alarm.free(timerId_MqttConnect);
		timerId_MqttConnect = dtINVALID_ALARM_ID;
		Serial.println(F("onWifiDisconnected: Removed MQTT reconnect timer"));
	}
}

// <NTP  ---------------------------------------------------------------------

time_t NTP_getNTPTimer() {

	time_t t = NTP.getTime();
	Serial.println("NTP Sync called:" + NTP.getTimeDateString(t));
	return t;
}

void NTP_onNTPprocessSyncEvent(NTPSyncEvent_t ntpEvent) {
	if (ntpEvent) {
		Serial.print("Time Sync error:");
		if (ntpEvent == noResponse)
			Serial.println("NTP server not reachable");
		else if (ntpEvent == invalidAddress)
			Serial.println("Invalid NTP server address");
	}
	else {
		Serial.println("NTP time ok:"+  NTP.getTimeDateString(NTP.getLastNTPSync()));

		if ( ntpInit == false) {	// abuse at init flag
			// setTime(NTP.getTime());		// not needed cause done by next line
			setSyncProvider(NTP_getNTPTimer);	// NTP to timelib connection. time lib from NTP
			setSyncInterval(3600);			// once per hour is enough
			// this makes only sense if we have ntp time as we reset per hr, min
			timerId_ResetMaxMin = Alarm.alarmRepeat (config.resetTimeMaxMinHr, config.resetTimeMaxMinMin, 0, timedResetMaxMin );	// no need to couple it with presence of NTP time
			Serial.println("NTP init ok. Next reset:"+  NTP.getTimeStr ( Alarm.read(timerId_ResetMaxMin)));	// no date
			ntpInit = true;
		}
	}
}


void timerInit(){ 		// start the timer

	Alarm.timerRepeat(1, timedUpdateDisplay	);	// update once per sec
	timerId_TempReading = Alarm.timerRepeat(config.readingIntervallTemp, timedReadTemperatures );
}


void setup() {

	Serial.begin(115200);
	delay(100);
	Serial.println("\nStarting. Version:" + String (version));
	text_html = F("text/html");

#ifdef SI7021ADDR
	Wire.begin(SDA, SCL);
	//	Wire.beginTransmission(SI7021ADDR);	not needed
	//	Wire.endTransmission();				not needed
#endif

	// Initialising the UI will init the display too.

	if ( display.init() == true) {
		Serial.println("Display init OK");
		display.flipScreenVertically();
		display.setFont(ArialMT_Plain_16);
		// display.setLogBuffer(2, sizeof (currentTempSensor->displayHum));	// 2 zeilen, 20 zeieln
		display.drawString(0,10,"Starting ...");
		display.display();
	}else
		Serial.println("Can't establish connection to display");

	EEPromReadConfig();
	if (strcmp(config.magicKey, magicEEKey)) {
		Serial.println(F("Init EEProm"));
		memset(&config, 0, sizeof(config));
		config.readingIntervallTemp = 300;
		strcpy(config.magicKey, magicEEKey);
		EEPromWriteConfig();
	}

	Serial.println("EEPROM:LastIP"   + String (config.localIP)
			+ ";Temp-ReadIntervall:" + String (config.readingIntervallTemp)
			+ ";MqttId:<" 			 + String (config.MqttId) 		+ ">"
			+ ";Mqtt-Server:<" 		 + String (config.mqttServer) 	+ ">"
			+ ";hostname:<" 		 + String (config.hostname) 	+ ">");

	mqtt_SetTopics();

	WiFiConnectHandler 	  = WiFi.onStationModeGotIP			( onWIFIConnectedGotIP);// As soon WiFi is connected, start NTP Client
	WiFiDisConnectHandler = WiFi.onStationModeDisconnected  ( onWifiDisconnected);

	WiFi.hostname ( config.hostname); // seems to work
	WiFi.mode	  ( WIFI_STA);		   // important !! station only
	WiFi.begin	  ( ssid, password);
	delay		  ( 100 );

	NTP.onNTPSyncEvent([](NTPSyncEvent_t event) {
		ntpEvent = event;
		NtpSyncEventTriggered = true;
	});

	// callbacks inits for mqtt

	mqttClient.onConnect	( mqttOn_Connect 	);
	mqttClient.onDisconnect	( mqttOn_Disconnect );
	mqttClient.onMessage	( mqttOn_Message 	);
	// mqttClient.setKeepAlive ( 60 );	// 15 def

	// http response handler
	server.on("/", 			serverHandle_Root);
	server.on("/configRsp", serverHandle_ConfigRsp);
	server.on("/cmd", 		serverHandle_Cmd);

#ifdef DHTPIN
	dht.setup(DHTPIN,DHTTYPE);
	Serial.println("DHT Detected Typ:" + String ( dht.getModel()));
#endif

// this will likely collide with the display: TODO test
	#ifdef ONE_WIRE_BUS_PIN
	sensors.begin();
#endif

	webPage		.reserve  (2000);
	statusLine	.reserve  ( 30 );

	timedResetMaxMin();
	timerInit(); 		// start the timer regardless of time present
}

void serverHandle_ConfigRsp (){

	char tmp [ sizeof (config.hostname) ];
	bool    reconnect = false;
		int l  = server.args();
		for (int i = 0; i< l ; i++) {
			Serial.println("ConfigRsp:" + String (i) + ", argN:" + server.argName(i) + ", argV:" + server.arg(i) );
		}

	strcpy ( tmp, server.arg("devid").c_str());

		if ( strcmp(config.MqttId, tmp)  != 0 ){
			Serial.println("MQTT DevId changed from <" + String (config.MqttId) + " > to " + server.arg("devid"));
			strcpy (config.MqttId, tmp);	// changes also the topics
			reconnect = true;
		}

		strcpy ( tmp, server.arg("mqttServerName").c_str());

		if ( strcmp(config.mqttServer, tmp)  != 0  ){
			Serial.println("MQTT Server name changed from <" + String (config.mqttServer) + " > to " + server.arg("mqttServerName"));
			strcpy (config.mqttServer, tmp);
			reconnect = true;
		}

		strcpy ( tmp, server.arg("hostNameId").c_str());
		if (strcmp(config.hostname, tmp)  != 0 ){
			Serial.println("Hostname changed from:<" + String (config.hostname) + " > to " +  String ( tmp));
			strcpy (config.hostname, tmp);
			reconnect = true;
		}

	config.readingIntervallTemp = server.arg("tempIntervall").toInt();
	Alarm.free(timerId_TempReading);	// free
	timerId_TempReading = Alarm.timerRepeat(config.readingIntervallTemp, timedReadTemperatures );	// set new

	char  cr [10];
	strcpy (cr, server.arg("MaxMinResetTime").c_str());

	char* token  			 	= strtok(cr, ":");	// destroys cr, therefore copy
	if (token) config.resetTimeMaxMinHr 	= atoi(token);
	token		 	= strtok(NULL, ":");
	if (token) config.resetTimeMaxMinMin 	= atoi(token);

	if (Alarm.isAllocated ( timerId_ResetMaxMin)  ) {
		Alarm.free(timerId_ResetMaxMin);	// free
		timerId_ResetMaxMin = Alarm.alarmRepeat (config.resetTimeMaxMinHr, config.resetTimeMaxMinMin, 0, timedResetMaxMin );	// no need to couple it with presence of NTP time
	}

	// print summary
	Serial.println("Name:" + String (config.MqttId) + ", Temp read intervall:" + String (config.readingIntervallTemp)
			+ " = next schedule:" + NTP.getTimeDateString(Alarm.read (timerId_TempReading))
			+"\nMaxMinResetTime:" + String (config.resetTimeMaxMinHr) + ":" + String (config.resetTimeMaxMinMin))
																			+ " = " + NTP.getTimeDateString(Alarm.read (timerId_ResetMaxMin));
	EEPromWriteConfig();
	updateWEBPage();

	if (reconnect == true){
		mqtt_SetTopics	 	();
		mqttClient.disconnect	();	// inits an connect, calls ondisconnect()
		// no mqtt_Timer_Connect(), cause that is done in the onDisconnect
	}

	// server.sendHeader("Connection", "close");
	server.sendHeader("Access-Control-Allow-Origin", "*");
	server.send(200, text_html, webPage);

}
void serverHandle_Root(){
	// Serial.println("Root called");
	updateWEBPage();
	server.send(200, text_html, webPage);
}


void serverHandle_Cmd(){

	if(server.arg("button").equals("reboot")){
		delayMicroseconds(1000);	// give to transmit and display
		server.send(200, text_html, "Reboot ok.");
		ESP.restart();
	}
	else
		if(server.arg("button").equals("reset")){
			Serial.println("Resetting ..." );
			memset  ( &config, 0, sizeof(config));
			EEPromWriteConfig();
			server.send(200, text_html, "Reset ok. EEProm cleared.");
			ESP.restart();
		}
		else
			server.send(200, text_html, "CMD not found for 'button':" + server.arg("button") );
}

void updateWEBPage (){

	webPage   = "<html><head><h2>Temperatur & Feuchte Messer</h2></head> <body>";
	webPage.concat ("<br><h3>IP: "			+ WiFi.localIP().toString());
	webPage.concat ("<br>Hostname: " 		+ String (localHostname));
	webPage.concat ("<br>MQTTId:"   		+ String (config.MqttId)  +", State:" + String (mqttClient.connected() ? "Connected" : "DISconnected"  ) + "</h3>");
	webPage.concat ("<br>Zeit: "     		+ NTP.getTimeDateString()) ;
	webPage.concat ("<br>Compile DateTime: " + String(version) + "</h3>");
	webPage.concat ("<br>FreeHeap:" 		 + String (ESP.getFreeHeap()) +  ", ChipID:"  + String(ESP.getChipId()));

	TempStatusType * tsp = &ts [0];
	for (int i = 0 ; i < OFS_LAST; i++ , tsp++) {
		// Serial.println("SYSInfo: ofs: "+ String (i) + ",topic:" + tsp->MqttPubTopic + ";errCode:" + String(tsp->errCode));
		if(tsp->MqttPubTopic[0]){	// != 0 : if empty, not used
			webPage  .concat(  "<br> topic:" + String (tsp->MqttPubTopic) +":");
			if (tsp->errCode == 0){
				if ( tsp->tempValid)
					webPage  .concat(  "Temperatur: " + String (tsp->temp,1) + "°C") ;
				if ( tsp->humValid)
					webPage  .concat( ", Feuchtigkeit:" + String (tsp->hum) + " %");
			}else
				webPage  .concat(  String (tsp->MqttPubTopic) + F(":Sensor Fehler"));
		}
	}
	// Serial.println("SYSInfo:final1:" + wp);
	webPage .concat( "<br>Name:" + String (config.MqttId) + ", FreeHeap:" + String (ESP.getFreeHeap()) +  ", ChipID:"  + String(ESP.getChipId()));
#ifdef DHTPIN
	webPage .concat(  "<br>DHTPIN:" + String (DHTPIN) + "; DHT Type:" + String (DHTTYPE);
#endif
#ifdef  SI7021ADDR
	webPage .concat(  "<br>Sensor SI7021: SCL:" + String (SCL) + "; SDA:" + String (SDA));
#endif

	webPage  .concat(  "</h4><br>");
// FORM
	webPage  .concat( "<h2>Configuration</h2>"\
			"<form action=configRsp method=post>"\

	// Hostname
			"<p><label for=hostNameId>Hostname (change requires reboot)</label>"\
			"<input id=hostNameId name=hostNameId type=text  maxlength=30  value=" + String(config.hostname) + ">"\
			"</p>");

	// MQTT Device ID
	webPage.concat("<label for=devid>Device ID (MQTT DevID)</label>"\
	"<input id=devName name=devid type=text  maxlength=19  value=" + String(config.MqttId) + ">"\
	"<br>");

	// MQTT Server
	webPage  .concat( "<label for=mqttServerName>MQTT Server addr</label>"\
			"<input id=mqttServer name=mqttServerName type=text  maxlength=19  value=" + String(config.mqttServer) + ">"\
			"<br>");

	// Temp intervall
	webPage .concat( "<label for=tempIntervall>Temperatur Ableseintervall secs </label>"\
			"<input id=temp name=tempIntervall type=number style=width:100px value=" + String(config.readingIntervallTemp) + ">"\
			"<br>");

	webPage .concat( "<label for=MaxMinResetTime>Daily MaxMin Temp Reset Time  </label>");
	webPage .concat( "<input type=time name=MaxMinResetTime value=" + String (config.resetTimeMaxMinHr) + ":" + String (config.resetTimeMaxMinMin) + "><br>");
	webPage .concat( F("<button name=task value=save>Speichern</button>"\
			"</form>"\
			"<form action=cmd method=post>"\
			"<button name=button value=reboot>Reboot</button><br>"\
			"<button style=width:120px name=button value=reset>Reset Configuration</button><br>"\
			"</form>"\
			"</body></html>"));

	// Serial.println("WEB page size:" + String(webPage.length()));

}

void loop() {

	if (NtpSyncEventTriggered) {
		NTP_onNTPprocessSyncEvent(ntpEvent);
		NtpSyncEventTriggered = false;
	}

	server.		handleClient();
	Alarm.delay(0);

}	// loop


void mqtt_SetTopics(){

	mqttClient.setServer	( config.mqttServer, 1883);	// might be changed in between
	mqttClient.setKeepAlive ( 60 );
	mqttClient.setClientId	( config.MqttId);

	// temperature sensors
	memset(ts, 0, sizeof(ts));

#ifdef DHTPIN
	sprintf (ts[OFS_DHT].MqttPubTopic, "%s/%s", config.MqttId,"TEMP/DHT");
	Serial.println ("DHT Topic:" + String(ts[OFS_DHT].MqttPubTopic));
#endif

#ifdef ONE_WIRE_BUS_PIN
	sprintf (ts[OFS_WIRE].MqttPubTopic, "%s/%s", config.MqttId,"TEMP/DS");
	Serial.println ("WIRE Topic:" + String(ts[OFS_WIRE].MqttPubTopic));
#endif

#ifdef SI7021ADDR
	sprintf (ts[OFS_SI702].MqttPubTopic, "%s/%s", config.MqttId,"TEMP/SI");
#endif

}

void mqtt_PublishErrorState ( char const * msg1, char const  * msg2 ){
	char errmsg [60];
	sprintf (errmsg, "%s:%s:<%s>", config.MqttId, msg1, msg2);	// ERR is an item in OH
	// snprintf (errmsg, 20, "%s-%s,<%s>",config.MqttId, msg1, msg2);	// cannot resolve
	// 	Serial.println ( errmsg);
	mqtt_Publish ( "ERR", errmsg);
}

void mqttOn_Message(const char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total){

		char pl [20] ;
		memset(pl, 0, sizeof (pl));

		Serial.print("MQTT rcv msg topic:<");
		Serial.print(topic);
		Serial.print(">,payload:<");

		for (uint i=0;i<len;i++) {
				Serial.print((char)payload[i]);
				pl[i] = payload[i];
			}
		Serial.println(">");

		if ( len > sizeof(pl)){
			mqtt_PublishErrorState ("Error: Payload longer than 20:", (const char *) payload);
			return;
		}

	// is it an init command ?
	if (strcmp(topic, initTopic) == 0 ){
		mqtt_PublishTemps ( );
		return;
	}
}

void  mqttOn_Connect (bool sessionPresent){

	Serial.println("MQTT connected. Session present:" + String (sessionPresent));
	// set the subscription topics

	Serial.print("subscribeTopic:<" + String(initTopic));
	// subscriptions are announced to the server. so if connection fails, it must be renewed
	if ( mqttClient.subscribe ( initTopic, 0) )
		Serial.println(">OK");
	else
		Serial.println(">ERROR");

	// announce our presence
	char  buf [  sizeof (config.localIP)  + sizeof (localHostname) + sizeof (config.MqttId) + 4];
	sprintf (buf, "%s;%s;%s", config.localIP, localHostname, config.MqttId);
	mqtt_Publish("ANNNOUNCE", buf); // in return Openhab could send an INIT to this device.

	Alarm.free(timerId_MqttConnect);	// works although this is the timer service routine
	timerId_MqttConnect = dtINVALID_ALARM_ID;

	Serial.println(F("Removed MQTT reconnect timer."));

	// does not work for unknown reason. just no bytes are received... ensures having all temperature present. otherwise empty payload is sent
	// timedReadTemperatures();
	// AsyncMqttClientMessageProperties properties;	// dummy
	// Pretend an init cmd from the server
	// mqttOn_Message( initTopic, (char *) "OnMQTTConnect", properties, 13, 0, 0);	// not all evaluated

}

void mqtt_Publish (const char * topic, char * payload) {

	if (mqttClient.connected ()) {
		Serial.println (NTP.getTimeStr() + ":Mqtt:<" + String(topic) + ">,msg:<" + String(payload) +">");
		mqttClient.publish	( topic , 0, true, payload);	// initial state of digit in
	}
}

void mqtt_PublishTemps () {

	char 	sensorData4MQTT [40];				// Preformatted buffer for MQTT
	struct TempStatusType  * t = & ts [ 0 ]; 	// ptr is faster

	for (int i= 0; i < OFS_LAST ; i++, t++){

		if (t->MqttPubTopic[0] != 0 ){	// only if used

			if (t->errCode == 0){

				if (t->tempValid == true)
				sprintf(sensorData4MQTT, "%s;%d", String(t->temp,1).c_str(), (int)t->hum);	// https://github.com/esp8266/Arduino/issues/73
//				if (t->humValid == true)
//					itoa ((int)t->hum, &sensorData4MQTT[strlen (sensorData4MQTT) ], 10);
				mqtt_Publish (t->MqttPubTopic, sensorData4MQTT);	// send to mqtt

			}
			else
			{
				char err [5];
				itoa(t->errCode, err, 10);
				mqtt_PublishErrorState(t->errMsg, err);
				mqtt_Publish 		(t->MqttPubTopic, t->errMsg);	// send to mqtt
			}
		}
	}
}

void ICACHE_FLASH_ATTR printFloat(float val, char *buff) {
   char smallBuff[16];
   int val1 = (int) val;
   unsigned int val2;
   if (val < 0) {
      val2 = (int) (-100.0 * val) % 100;
   } else {
      val2 = (int) (100.0 * val) % 100;
   }

   os_sprintf(smallBuff, "%i.%02u", val1, val2);

   strcat(buff, smallBuff);
}
/*
 *
 */
void   mqttOn_Disconnect(AsyncMqttClientDisconnectReason reason) {

	printf ("MQTT disconnected. Reason:%d\n", (int) reason);
	mqtt_StartConnectTimer(); 	// disconnected,start the reconnect timer.

}

void mqtt_StartConnectTimer(){

	if (config.mqttServer[0] == 0 || (config.mqttServer[0] != 0  && strlen(config.mqttServer) ==0))	// not - yet - configured
		return;

	if (!Alarm.isAllocated(timerId_MqttConnect ) ) {						// if !already scheduled
		timerId_MqttConnect = Alarm.timerRepeat(10, mqtt_Timed_Connect );	// start a reconnect every 60 secs
		Serial.println	(F("MQTT scheduled reconnect."));
	}
}

/*
   	Start the inits for mqtt if not already started.
  	  Both used as the timer service routine and as init routine onReceivalofIP. That allows
  	  for faster connect directly after receiving an IP and thereby not starting/waiting for a timer.
 */

void mqtt_Timed_Connect() {

	// Serial.println("timed_MqttConnect: Trying to connect."); //to <" +  String(config.mqttServer) +">");
	// might  be changed in between
	mqttClient.connect ();	// ASYNC call, returns nothing.

}

void timedReadTemperatures(){

	// Serial.println( NTP.getTimeStr() + ":readTemperatures Timer. Next reset:" + NTP.getTimeDateString(Alarm.read(timerId_ResetMaxMin)));

#ifdef SI7021ADDR
	sensor_SI7021();
#endif

#ifdef DHTPIN
	sensor_DHT();		// asynchron reading; start, rest asyncrhon above
#endif

	TempStatusType * sensor = &ts [0];
	for (int i =0; i < OFS_LAST ; i++, sensor++) {
		if (sensor->MqttPubTopic [0] != '\0' && sensor->errCode == 0 ){	// only if used and if correctly read
				// MAX
				if ( sensor->temp > sensor->tempMax){	// new high
					Serial.println("Sensor:" + String (sensor->MqttPubTopic) +  ", New High Temp:"+  String(sensor->temp,1) + ", was:" + String(sensor->tempMax,1) );
					sensor->tempMax = sensor->temp;
				}
				// MIN
				if ( sensor->temp < sensor->tempMin){	// new low
					Serial.println("Sensor:" + String (sensor->MqttPubTopic)  + ", New Low Temp:" + String(sensor->temp,1) + + ", was:" + String(sensor->tempMin,1) );
					sensor->tempMin = sensor->temp;
				}

				// MAX HUM, currently not displayed
				if ( sensor->hum > sensor->humMax ){	// new high
					Serial.println("New High hum:"+  String(sensor->hum) + ", was:" + String(sensor->humMax) );
					sensor->humMax  = sensor->hum;
				}
				//MIN
				if ( displaySensor->hum < sensor->humMin){	// new low
					Serial.println("New Low hum:" + String(sensor->hum) + + ", was:" + String(sensor->humMin) );
					sensor->humMin = sensor->hum;
				}
		}
	}

	mqtt_PublishTemps();
}

// DISPLAY_HEIGHT 64
// DISPLAY_WIDTH 128

void timedUpdateDisplay(){

	// Serial.println( NTP.getTimeStr() + ":updateDisplay Timer");

	display.clear();
	display.setFont(Bitstream_Charter_Italic_16);	// 0x10, // Width: 16 0x14, // Height: 20  ,,,, netter als Arial
	// display.setFont(ArialMT_Plain_16);

	// 			X-rechts(max 128), Y - von oben (max 64)
	display.drawString (0,  0, displaySensor->displayTemp);
	display.drawString (0, 15, displaySensor->displayHum);

	//		display.println(test[i]);
	//		display.drawLogBuffer()

	display.setFont(ArialMT_Plain_10);
	display.drawString (0, 35, "Hoch:" +  String (displaySensor->tempMax,1) + "  Tief:" +  String (displaySensor->tempMin,1));

	// display.setFont(ArialMT_Plain_16);	//  zu groß
	display.drawString (0, 51, NTP.getTimeStr());

	display.setFont(ArialMT_Plain_10);
	if (WiFi.status() == WL_CONNECTED) {
			if ( mqttClient.connected() )
				statusLine = "IP:" +  WiFi.localIP().toString();
			else
				statusLine = "Server Err";
	}
	else
		statusLine = "WLAN Err:" + String (WiFi.status());

	display.drawString (60, 54, statusLine);

	display.display(); // write the buffer to the display
	delay(10);
}

void timedResetMaxMin(){

	Serial.println( NTP.getTimeStr() + ":Resetting MaxMin");

	TempStatusType * sensor = &ts[0];
	for (int i =0; i < OFS_LAST ; i++, sensor++) {
		if (sensor->MqttPubTopic [0] != '\0'){	// only if used
			sensor->tempMax = -127.;			// max range of sensor
			sensor->tempMin = +127.;
			sensor->humMax  = -127;
			sensor->humMin  = +127;
		}
	}
}

#ifdef SI7021ADDR

void sensor_SI7021() {

	unsigned int 	data[2];
	float humidity	=	-1;
	float temp 		= -127.;
	// Serial.println("Started reading SI Sensor");
	TempStatusType * siSensor = & ts[ OFS_SI702 ];

	Wire.beginTransmission(SI7021ADDR); //Send humidity measurement command
	Wire.write(0xF5);
	Wire.endTransmission();
	delay(500);

	// Request 2 bytes of data
	Wire.requestFrom(SI7021ADDR, 2);
	int a;
	if( (a = Wire.available()) == 2) 	// Read 2 bytes of data to get humidity
	{
		data[0] = Wire.read();
		data[1] = Wire.read();
		// Convert the data
		humidity  = ((data[0] * 256.0) + data[1]);
		siSensor->hum = (int) ((125 * humidity) / 65536.0) - 6;
		siSensor->humValid 	= true;

		Wire.beginTransmission(SI7021ADDR); // Send temperature measurement command
		Wire.write(0xF3);
		Wire.endTransmission();
		delay(500);

		// Request 2 bytes of data
		Wire.requestFrom(SI7021ADDR, 2);

		if(Wire.available() == 2) {

			data[0] = Wire.read(); 	// Read 2 bytes of data for temperature
			data[1] = Wire.read();

			// Convert the data
			temp  		= ((data[0] * 256.0) + data[1]);
			siSensor->temp  = ((175.72 * temp) / 65536.0) - 46.85;
			siSensor->temp  =  roundf(siSensor->temp * 10) / 10; // http://stackoverflow.com/questions/1343890/rounding-number-to-2-decimal-places-in-c

			siSensor->errCode 	= 0;
			siSensor->errMsg[0]= '\0';
			siSensor->tempValid = true;

			sprintf (siSensor->displayTemp, "Temp:     %d.%d C", (int) siSensor->temp, (int)(siSensor->temp * 10.0) % 10)  ; // \t does not work
			sprintf (siSensor->displayHum,  "Feuchte: %d  %%", 		   siSensor->hum)  ;	// http://stackoverflow.com/questions/1860159/how-to-escape-the-sign-in-cs-printf

			// Serial.printf ("%s:%s; %s\n", NTP.getTimeStr().c_str(), t->displayTemp , t->displayHum);

		}
	}
	else {
		Serial.println("Error reading SI Sensor. Err:"+ String (a));
		siSensor->errCode 	 = -1;
		siSensor->tempValid = siSensor->humValid 	= false;
		strcpy (siSensor->displayTemp, "Sensor");
		strcpy (siSensor->displayHum,   "     Fehler");
		strncpy(siSensor->errMsg, "Fehler;" , sizeof(siSensor->errMsg)); //';' wichtig für das openhab String parsing
	}

}
#endif


#ifdef DHTPIN
void sensor_DHT(){

	TempStatusType * t = &ts[OFS_DHT];

		float hum  	= dht.getHumidity	();
		float temp	= dht.getTemperature();

		DHT::DHT_ERROR_t status  = dht.getStatus(); 		// get DHT status

		// Serial.println("DHT-State:" + String ( status ));

		if (status == DHT::ERROR_NONE) {
			t->hum 		 = hum;
			t->temp		 = temp;
			t->tempValid = t->humValid 	= true;
			t->errCode 	 = t->errMsg[0] = 0;
		}
		else {
			t->errCode 	 	= status;
			t->tempValid 	= t->humValid 	= false;
			strcpy(t->errMsg,"DHT Sensor Err:");
			strcat(t->errMsg, dht.getStatusString());

			strcpy (t->displayTemp, "Sensor");
			strcpy (t->displayHum,  dht.getStatusString());
			strcpy (t->errMsg,"DHT Sensor Err");
	}

}
#endif

