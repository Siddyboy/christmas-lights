#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Time.h>
#include <TimeAlarms.h> // Don't forget to change the maximum number of alarms in the header of TimeAlarms.h
#include "arduino_secrets.h"

// TODO(SCJK): CAPITALISE CONSTANTS!!!
const int ON_HOUR = 6;                             // Hour for turning lights on.
const int ON_MINUTE = 30;                           // Minute for turning lights on.
const int ON_TIME = (ON_HOUR * 60) + ON_MINUTE;     // Minutes past midnight to turn lights on.

const int OFF_HOUR = 23;                            // Hour for turning lights off.
const int OFF_MINUTE = 26;                          // Minute for turning lights off.
const int OFF_TIME = (OFF_HOUR * 60) + OFF_MINUTE;  // Minutes past midnight to turn lights off.

const unsigned long SWEEP_DELAY = 50;              // A timing delay to spread energisation of relays. Kinder?

const int RELAY_PINS[] = {4, 7, 8, 12};             // Array of pin numbers for the four relays.

char ssid[] = SECRET_SSID;                          // WiFi SSID from arduino_secrets.h
char pass[] = SECRET_PASS;                          // WiFi passord from arduino_secrets.h
int status = WL_IDLE_STATUS;                        // WiFi status.

unsigned int localPort = 2390;                      // Port for ??????? TODO(SCJK): Comment this properly.

//IPAddress timeServer(129, 6, 15, 28);             //time.nist.gov
IPAddress timeServer(143, 210, 16, 201);            //0.uk.pool.ntp.org

WiFiUDP Udp;

bool lightsOn = false;                              // Keep track of lights' status.

void setup() {
  for(int i = 0; i <= 3; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
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
  Alarm.alarmRepeat(ON_HOUR, ON_MINUTE, 0, allOn);
  Alarm.alarmRepeat(OFF_HOUR, OFF_MINUTE, 0, allOff);
  
  /*-------- Set correct status at startup --------*/
  int nowTime = (hour() * 60) + minute();
  Serial.print(" nowTime = ");
  Serial.print(nowTime);
  Serial.print(" ON_TIME = ");
  Serial.print(ON_TIME);
  Serial.print(" OFF_TIME = ");
  Serial.println(OFF_TIME);
  if( (nowTime >= ON_TIME) && (nowTime <= OFF_TIME) ) {
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
  // TODO(SCJK) Use printDigits for on and off times too.
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" Status = ");
  Serial.print(lightsOn);
  Serial.print(", On = ");
  Serial.print(ON_HOUR);
  printDigits(ON_MINUTE);
  Serial.print(", Off = ");
  Serial.print(OFF_HOUR);
  printDigits(OFF_MINUTE);
  Serial.println();
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
    sweepOff();
    delay(500);
    sweepOn();
    delay(500);
  }
}

void hourChime() {
  int dongs = hourFormat12();
  Serial.print(" Dongs = ");
  Serial.println(dongs);
  delay(1000);
  sweepOff();
  delay(2000);
  for(int i = 1; i <= dongs; i++) {
    sweepOn();
    delay(200);
    sweepOff();
    delay(800);
    Serial.println("DONG!"); 
  }
  delay(2000);
  sweepOn();
}

/*-------- Change lights' status ----------*/

void allOn() {
  lightsOn = true;
  sweepOn();
}

void allOff() {
  lightsOn = false;
  sweepOff();
}

/*-------- Sweep relays on or off ----------*/

void sweepOn() {
  for(int i = 0; i <= 3; i++) {
    digitalWrite(RELAY_PINS[i], HIGH);
    delay(SWEEP_DELAY);
  }
}

void sweepOff() {
  for(int i = 3; i >= 0; i--) {
    digitalWrite(RELAY_PINS[i], LOW);
    delay(SWEEP_DELAY);  
  }
}

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
    
