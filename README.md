This is a work in progress.
Basic features :
* ESP8266 is self powered by the smartmeter P1 port.
* Read P1 datagram every second, check checksum, parse P1 telegram
* Send data to MQTT gateway.
* Accept Telnet session on port 23 with a basic CLI
* Accept 2 TCP/IP socket on port 101 & 102 and relay P1 telegram on it.  This has been successfully tested to send P1 telegram on TCP to an Alfen EVSE for loadbalancing purpose and is used to send P1 telegram to EMS project.
 
