# ESP8266 Temperature & Humidity Sensor with OLED display.

A control program for collecting and displaying temperature &amp; humidity data from various sensors types. 

The data is transferred regularly to an MQTT server and can be displayed and processed by any Home Automation System (e.g. OpenHAB)

Features:
- WIFI
- MQTT
- WEB GUI for configuration
- maximums and minimums temperature of the last 24h.
- Supports sensors: DHT, SI7021, DS18X
- local OLED Display SSD 1306
- Analog time display 
- NTP time synchronisation 
- connection status in display (IP adr, error conditions)
- Home automation integration by automatic presence announcing to MQTT e.g. to OpenHAB
- SYSLOG messages 
- DEBUG mode to be enabled by WEB GUI.
- Over- the- Air (OTA) (WIFI) update 

See WIKI for more details.

Have fun adapting and modifying  ! 
Gerry 
