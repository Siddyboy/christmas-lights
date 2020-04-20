#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Time.h>
#include <TimeAlarms.h> // Don't forget to change the maximum number of alarms in the header of TimeAlarms.h
#include "arduino_secrets.h"

const int ON_HOUR = 6;                              // Hour for turning lights on.
const int ON_MINUTE = 10;                           // Minute for turning lights on.
const int ON_TIME = (ON_HOUR * 60) + ON_MINUTE;     // Minutes past midnight to turn lights on.

const int OFF_HOUR = 23;                            // Hour for turning lights off.
const int OFF_MINUTE = 3;                           // Minute for turning lights off.
const int OFF_TIME = (OFF_HOUR * 60) + OFF_MINUTE;  // Minutes past midnight to turn lights off.

const int RELAY_PINS[] = {4, 7, 8, 12};             // Array of pin numbers for the four relays.

const PinStatus ON = HIGH;
const PinStatus OFF = LOW;
const bool UP = true;
const bool DOWN = false;

char ssid[] = SECRET_SSID;                          // WiFi SSID from arduino_secrets.h
char pass[] = SECRET_PASS;                          // WiFi passord from arduino_secrets.h
int status = WL_IDLE_STATUS;                        // WiFi status.

unsigned int localPort = 2390;                      // Port for ??????? TODO(SCJK): Comment this properly.

//IPAddress timeServer(129, 6, 15, 28);             //time.nist.gov
IPAddress timeServer(143, 210, 16, 201);            //0.uk.pool.ntp.org

WiFiUDP Udp;

bool lightsOn = false;                              // Keep track of lights' status.
AlarmID_t alarmLightsOn;                            // Alarm ID for turn on time.
AlarmID_t alarmLightsOff;                           // Alarm ID for turn off time.

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  for (int i = 0; i <= 3; i++) {
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

  /*-------- Say hello! and turn lights off (safe state). --------*/
  
  sayHello();
  digitalClockDisplay();
  Serial.println(" Turn lights off to start");
  sweepLights(0, OFF, UP);
    
  /*-------- Set alarms for regular clock functionality --------*/
  
  for (int i = 0; i <= 23; i++) {
    Alarm.alarmRepeat(i, 59, 52, hourChime); // A little early so chime in on the hour.
    Alarm.alarmRepeat(i, 15, 0, quarterPastChime);
    Alarm.alarmRepeat(i, 30, 0, halfPastChime);
    Alarm.alarmRepeat(i, 45, 0, quarterToChime);
  }  
  
  /*-------- Check on/off times are sensible --------*/
  
  // Check on is before off
  // Check that randomness doesn't make off before on
  // Anything else?
  
  /*-------- Set correct status at startup --------*/
  
  int nowTime = (hour() * 60) + minute();
  digitalClockDisplay();
  Serial.print(" nowTime = ");
  Serial.println(nowTime);
  digitalClockDisplay();
  Serial.print(" ON_TIME = ");
  Serial.println(ON_TIME);
  digitalClockDisplay();
  Serial.print(" OFF_TIME = ");
  Serial.println(OFF_TIME);
  
  if( (nowTime >= ON_TIME) && (nowTime <= OFF_TIME) ) {
    switchOn();
    digitalClockDisplay();
    Serial.println(" Switch lights on because the time is right");
  }
  else {
    switchOff();
    digitalClockDisplay();
    Serial.println(" Switch lights off because it's not time yet");
  }
}

/*
Also make lights blink an error code (that looks like nice flashing xmas tree lights :) ) when there is a problem with NTP synch or wifi link or whatever.
differnet blink codes for differnet enrrors!
*/

void loop() {
  if (timeStatus() != timeNotSet) {
    digitalClockDisplay();
    statusDisplay();
    digitalWrite(LED_BUILTIN, CHANGE);
    Alarm.delay(1000);
  }
}


/*-------- Hello sequence --------*/

void sayHello() {
  sweepLights(0, OFF, DOWN);
  digitalClockDisplay();
  Serial.println(" Hello there!");
  digitalWrite(RELAY_PINS[0], HIGH);
  digitalWrite(RELAY_PINS[2], HIGH);
  delay(500);
  for(int i = 0; i <= 6; i++) {
    digitalWrite(RELAY_PINS[0], CHANGE);
    digitalWrite(RELAY_PINS[1], CHANGE);
    digitalWrite(RELAY_PINS[2], CHANGE);
    digitalWrite(RELAY_PINS[3], CHANGE);
    delay(500);
  }  
  sweepLights(0, OFF, DOWN);
  delay(5000);
}

/*-------- Digital Clock Code --------*/

void digitalClockDisplay() {
  // TODO(SCJK) Use printDigits for on and off times too.
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
}

void printDigits(int digits) {
  Serial.print(":");
  if (digits < 10) {
    Serial.print('0');
  }
  Serial.print(digits);
}

/*-------- Status Display --------*/

void statusDisplay() {
  Serial.print(" Status = ");
  if (lightsOn == true) {
    Serial.print("On");
  }
  else {
    Serial.print("Off");
  }
  Serial.print(", On @ ");
  Serial.print(ON_HOUR);
  printDigits(ON_MINUTE);
  Serial.print(", Off @ ");
  Serial.print(OFF_HOUR);
  printDigits(OFF_MINUTE);
  Serial.println();
}

/*-------- Light Control Alarms --------*/

void quarterPastChime() {
  if (lightsOn == true) {
    digitalClockDisplay();
    Serial.println(" Quarter Past Chime.");
    chime(1);
  }
}

void halfPastChime() {
  if(lightsOn == true) {
    digitalClockDisplay();
    Serial.println(" Half Past Chime.");
    chime(2);
  }
}

void quarterToChime() {
  if(lightsOn == true) {
    digitalClockDisplay();
    Serial.println(" Quarter to Chime.");
    chime(3);
  }
}

void chime(int n) {
  for(int i = 1; i <= n; i++) {
    sweepLights(20, OFF, DOWN);
    delay(500);
    sweepLights(20, ON, UP);
    delay(500);
  }
}

void hourChime() {
  if(lightsOn == true) {
    int dongs = hourFormat12() + 1;  // Corrected because alarm triggered before the hour.
    if (dongs > 12) {                // Correct the correction for 13 o'clock!
      dongs = 1;
    }
    digitalClockDisplay();
    Serial.print(" Dongs = ");
    Serial.println(dongs);
    sweepLights(1000, OFF, DOWN);
    delay(3000);
    for(int i = 1; i <= dongs; i++) {
      sweepLights(20, ON, UP);
      delay(200);
      sweepLights(20, OFF, DOWN);
      delay(800);
      digitalClockDisplay();
      Serial.println(" DONG!"); 
    }
    delay(3000);
    sweepLights(1000, ON, UP);
  }
}

/*-------- Change lights' status ----------*/

void switchOn() {
  digitalClockDisplay();
  if(alarmLightsOn > 0) {
    Serial.print(" 'switchOn' called from alarm ID = ");
    Serial.println(alarmLightsOn);
  }
  else {
    Serial.println(" 'switchOn' called (not by an alarm).");
  }
  lightsOn = true;
  sweepLights(2000, ON, UP);
  digitalClockDisplay();
  alarmLightsOff = Alarm.alarmOnce(OFF_HOUR, OFF_MINUTE, 0, switchOff);
  Serial.print(" Set new 'switch off' alarm: ID = ");
  Serial.println(alarmLightsOff);
}

void switchOff() {
  digitalClockDisplay();
  if(alarmLightsOff > 0) {
    Serial.print(" 'switchOff' called from alarm ID = ");
    Serial.println(alarmLightsOff);
  }
  else {
    Serial.println(" 'switchOff' called (not by an alarm).");
  }
  lightsOn = false;
  sweepLights(2000, OFF, DOWN);
  digitalClockDisplay();
  alarmLightsOn = Alarm.alarmOnce(ON_HOUR, ON_MINUTE, 0, switchOn);
  Serial.print(" Set new 'switch on' alarm: ID = ");
  Serial.println(alarmLightsOn);
}

/*-------- Sweep relays on/off and up/down ----------*/

void sweepLights(int sweepDelay, PinStatus STATE, bool sweepUp) {
  int n;
  for(int i = 0; i <=3; i++) {
    if(sweepUp == true) {
      n = i;
    }
    else {
      n = 3 - i;
    }
    digitalWrite(RELAY_PINS[n], STATE);
    delay(sweepDelay);
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


