#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Time.h>
#include <TimeAlarms.h> // Don't forget to change the maximum number of alarms in the header of TimeAlarms.h
#include "arduino_secrets.h"

bool lightsOn = false;                           // Keep track of lights' status.
// TODO(SCJK): CAPITALISE CONSTANTS!!!
const int onHour = 22;                           // Hour for turning lights on.
const int onMinute = 40;                         // Minute for turning lights on.
const int onTime = (onHour * 60) + onMinute;     // Minutes past midnight to turn lights on.

const int offHour = 23;                          // Hour for turning lights off.
const int offMinute = 26;                        // Minute for turning lights off.
const int offTime = (offHour * 60) + offMinute;  // Minutes past midnight to turn lights off.

const unsigned long del = 100;                   // A timing delay.

const int USB[] = {4, 7, 8, 12};                 // Array of pin numbers for the four relays.

char ssid[] = SECRET_SSID;                       // WiFi SSID from arduino_secrets.h
char pass[] = SECRET_PASS;                       // WiFi passord from arduino_secrets.h
int status = WL_IDLE_STATUS;                     // WiFi status.

unsigned int localPort = 2390;                   // Port for ??????? TODO(SCJK): Comment this properly.

//IPAddress timeServer(129, 6, 15, 28);            //time.nist.gov
IPAddress timeServer(143, 210, 16, 201);         //0.uk.pool.ntp.org

WiFiUDP Udp;

void setup() {
  for(int i = 0; i <= 3; i++) {
    pinMode(USB[i], OUTPUT);
  }
  
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
  setSyncProvider(getNtpTime);

  /*-------- Set alarms for clock functionality --------*/
  for(int i = 0; i <= 23; i++) {
    Alarm.alarmRepeat(i, 0, 0, hourChime);
    Alarm.alarmRepeat(i, 15, 0, quarterPastChime);
    Alarm.alarmRepeat(i, 30, 0, halfPastChime);
    Alarm.alarmRepeat(i, 45, 0, quarterToChime);
  }  
  
  /*-------- Set alarms for on and off times --------*/
  Alarm.alarmRepeat(onHour, onMinute, 0, allOn);
  Alarm.alarmRepeat(offHour, offMinute, 0, allOff);
  
  /*-------- Set correct status at startup --------*/
  int nowTime = (hour() * 60) + minute();
  Serial.print("nowTime = ");
  Serial.print(nowTime);
  Serial.print(" onTime = ");
  Serial.print(onTime);
  Serial.print(" offTime = ");
  Serial.println(offTime);
  if( (nowTime >= onTime) && (nowTime <= offTime) ) {
    allOn();
    Serial.println("BOOM ON!");
  }
  else {
    allOff();
    Serial.println("BOSH OFF!");
  }
}
/*
Also make lights blink an error code (that looks like nice flashing xmas tree lights :) ) when there is a problem with NTP synch or wifi link or whatever.
differnet blink codes for differnet enrrors!
*/

void loop() {
  if (timeStatus() != timeNotSet) {
    digitalClockDisplay();
    Alarm.delay(1000);
  }
}

/*-------- Digital Clock Code --------*/

void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" Status = ");
  Serial.print(lightsOn);
  Serial.print(", On = ");
  Serial.print(onHour);
  Serial.print(":");
  Serial.print(onMinute);
  Serial.print(", Off = ");
  Serial.print(offHour);
  Serial.print(":");
  Serial.println(offMinute);
}

void printDigits(int digits) {
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

/*-------- Light Control Alarms --------*/
void quarterPastChime() {
  if(lightsOn == true) {
    digitalClockDisplay();
    Serial.println("Quarter Past Chime.");
    chime(1);
  }
}

void halfPastChime() {
  if(lightsOn == true) {
    digitalClockDisplay();
    Serial.println("Half Past Chime.");
    chime(2);
  }
}

void quarterToChime() {
  if(lightsOn == true) {
    digitalClockDisplay();
    Serial.println("Quarter to Chime.");
    chime(3);
  }
}

void chime(int n) {
  for(int i = 1; i <= n; i++) {
    allOff();
    delay(500);
    allOn();
    delay(500);
  }
}

void hourChime() {
  Serial.print("Hour Dongs");
}

void allOn() {
  lightsOn = true;
  for(int i = 0; i <= 3; i++) {
    digitalWrite(USB[i], HIGH);
    delay(del);
  }
}
//  digitalWrite(USB1, HIGH);
//  delay(del);
//  digitalWrite(USB2, HIGH);
//  delay(del);
//  digitalWrite(USB3, HIGH);
//  delay(del);
// digitalWrite(USB4, HIGH);

void allOff() {
  lightsOn = false;
  for(int i = 3; i >= 0; i--) {
    digitalWrite(USB[i], LOW);
    delay(del);  
  }
}
//  digitalWrite(USB4, LOW);
//  delay(del);
//  digitalWrite(USB3, LOW);
//  delay(del);
//  digitalWrite(USB2, LOW);
//  delay(del);
//  digitalWrite(USB1, LOW);

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

time_t getNtpTime() {
  while (Udp.parsePacket() > 0);
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);
      unsigned long secsSince1900;
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0;
}

unsigned long sendNTPpacket(IPAddress & address) {
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

/*-------- WIFI code --------*/

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
    
