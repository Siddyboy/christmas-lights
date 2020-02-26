#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <RTCZero.h> \\ RTC.h or RTCZero.h?

// Don't confuse connection to WiFi with connection to a server.

int status = WL_IDLE_STATUS;

#include "arduino_secrets.h"
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

unsigned int localUdpPort = 2390;

IPAddress timeServer(xxx, y, zzz, aaa);

const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

RTCZero rtc;
WiFiUDP Udp;

void setup() {
  Serial.begin(9600); //might need the while (!Serial) {;} thing here for USB ports. Try without first.
  
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }

  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  Udp.begin(localUdpPort);
  
  rtc.begin();

  
  get time from ntp server
  set time to rtc
}

void loop
  timed on/off
  set hours plus randomness
  
  at hours (when on (except 4 pm)) 
    turn off for few seconds
    blink hour 'chimes' - see big ben timing!
    
  at quarters (when on)
    just blink off once (15 min past)
    blink off twice (30 past)
    blink off thrice (45 past).
    
  at 4pm
    turn off for a few seconds
    blink tea-for-two rhythm.
    
  at 3am get time from ntp and reset rtc
  
void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
    
