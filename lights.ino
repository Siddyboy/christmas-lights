#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <DS3231.h>
// #include <RTCZero.h>  // RTC.h or RTCZero.h?
// Don't confuse connection to WiFi with connection to a server.
#include "arduino_secrets.h"

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int status = WL_IDLE_STATUS;

unsigned int localPort = 2390;

// IPAddress timeServer(129, 6, 15, 28); //time.nist.gov
IPAddress timeServer(143, 210, 16, 201); //0.uk.pool.ntp.org

const int NTP_PACKET_SIZE = 48;

byte packetBuffer[NTP_PACKET_SIZE];

// RTCZero rtc;
WiFiUDP Udp;

void setup() {
  Serial.begin(9600);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }
  
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }

  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  Udp.begin(localPort);
  
//  rtc.begin();

  
//  get time from ntp server
//  set time to rtc
}

void loop() {
  sendNTPpacket(timeServer);
  delay(1000);
  if (Udp.parsePacket()) {
    Serial.println("packet received");
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord <<16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);
    
    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;
    Serial.print("Unix time = ");
    Serial.println(epoch);

    Serial.print("The UTC time is ");
    Serial.print((epoch % 86400L) / 3600);
    Serial.print(':');
    if (((epoch % 3600) / 60) < 10) {
      Serial.print('0');
    }
    Serial.print((epoch % 3600) / 60);
    Serial.print(':');
    if ((epoch % 60) < 10) {
      Serial.print('0');
    }
    Serial.println(epoch % 60);
  }

  delay(10000);
}
//  timed on/off
//  set hours plus randomness
  
//  at hours (when on (except 4 pm)) 
//   turn off for few seconds
//    blink hour 'chimes' - see big ben timing!
    
//  at quarters (when on)
//   just blink off once (15 min past)
//    blink off twice (30 past)
//    blink off thrice (45 past).
    
//  at 4pm
//    turn off for a few seconds
//    blink tea-for-two rhythm.
    
//  at 3am get time from ntp and reset rtc

unsigned long sendNTPpacket(IPAddress& address) {
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;
  packetBuffer[1] = 0;
  packetBuffer[2] = 6;
  packetBuffer[3] = 0xEC;
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  Udp.beginPacket(address, 123);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

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
    