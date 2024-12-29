#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <TimeAlarms.h> // Don't forget to change the maximum number of alarms in the header of TimeAlarms.h
#include "arduino_secrets.h"

#define LED_BUILTIN 13                              // Beware LED_BUILTIN is default pin 25 on ATmega4809.

const int ON_HOUR = 5;                              // Hour for turning lights on.
const int ON_TIME = ON_HOUR * 60;                   // Minutes past midnight to turn lights on.

const int OFF_HOUR = 22;                            // Hour for turning lights off.
const int OFF_TIME = OFF_HOUR * 60;                 // Minutes past midnight to turn lights off.

const int RELAY_PINS[] = {4, 7, 8, 12};             // Array of pin numbers for the four relays.

const PinStatus ON = HIGH;
const PinStatus OFF = LOW;
const bool UP = true;
const bool DOWN = false;

int onMinute = 0;                                   // Variable to hold random number of minutes for on time.
int offMinute = 0;                                  // Variable to hold random number of minutes for on time.

char ssid[] = SECRET_SSID;                          // WiFi SSID from arduino_secrets.h
char pass[] = SECRET_PASS;                          // WiFi passord from arduino_secrets.h
int status = WL_IDLE_STATUS;                        // WiFi status.

unsigned int localPort = 2390;                      // Port for ??????? TODO(SCJK): Comment this properly.

const char* ntpServerName = "time.nist.gov";
//const char* ntpServerName = "pool.ntp.org";
//const char* ntpServerName = "time.google.com";

IPAddress timeServerIP(129, 6, 15, 28);
IPAddress oldIP;

int syncInt = 300;

WiFiUDP Udp;

bool lightsOn = false;                              // Keep track of lights' status.
AlarmID_t alarmLightsOn;                            // Alarm ID for turn on time.
AlarmID_t alarmLightsOff;                           // Alarm ID for turn off time.

void setup() {
  setupPins();
  Serial.begin(9600);
  sayHello();
  wifiModuleChecks();
  connectToWlan();
  printWlanStatus();
  getNtpServerIP();
  startSocket();
  
  digitalClockDisplay();
  Serial.println(" Set time sync provider.");
  setSyncProvider(getNtpTime);
  digitalClockDisplay();
  Serial.print(" Set time sync interval to ");
  Serial.print(syncInt);
  Serial.println(" seconds.");
  setSyncInterval(syncInt);

  digitalClockDisplay();
  Serial.println(" Setting up chime alarms.");
  for (int i = 0; i <= 23; i++) {
    Alarm.alarmRepeat(i, 59, 52, hourChime); // A little early so chime is on the hour.
    Alarm.alarmRepeat(i, 15, 0, quarterPastChime);
    Alarm.alarmRepeat(i, 30, 0, halfPastChime);
    Alarm.alarmRepeat(i, 45, 0, quarterToChime);
  }  
  
  /*-------- Set correct status at startup --------*/
  /*HERE TURN THIS INTO A FUNCTION!!!! TAKES nowTime as arg!*/
  int nowTime = (hour() * 60) + minute();
  digitalClockDisplay();
  Serial.print(" Time now is ");
  Serial.print(nowTime);
  Serial.println(" min past midnight.");
  digitalClockDisplay();
  Serial.print(" ON_TIME is ");
  Serial.print(ON_TIME);
  Serial.println(" min past midnight.");
  digitalClockDisplay();
  Serial.print(" OFF_TIME is ");
  Serial.print(OFF_TIME);
  Serial.println(" min past midnight.");
  
  digitalClockDisplay();
  if( (nowTime >= ON_TIME) && (nowTime <= OFF_TIME) ) {
    Serial.println(" Switch lights on; the time is right.");
    switchOn();
  }
  else {
    Serial.println(" Switch lights off; it's not time yet.");
    switchOff();
  }

}

void loop() {
  //digitalClockDisplay();
  //statusDisplay();
  status = WiFi.status();
  if (status != WL_CONNECTED) {
    digitalClockDisplay();
    Serial.println(" WiFi not connected.");
    wifiModuleChecks();
    connectToWlan();
    printWlanStatus();
  }
  
  if (timeStatus() == timeSet) {
    digitalWrite(LED_BUILTIN, CHANGE);
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
    digitalClockDisplay();
    Serial.println(" Time not set or needs synch.");
    delay(5000);
    digitalClockDisplay();
    Serial.println(" Getting new time.");
    time_t newTime = getNtpTime();
    if (newTime != 0) {
     setTime(newTime);
     digitalClockDisplay();
     Serial.println(" New time set."); //NOW NEED TO CHECK IF WE NEED TO TURN ON LIGHTS! INCASE TIME NOT SET FROM START!
    }
    else {
      digitalClockDisplay();
      Serial.println(" No time returned.");
    }
  }

/*  switch (timeStatus()) {
    case timeSet:
      digitalWrite(LED_BUILTIN, CHANGE);
      break;
    case timeNeedsSync:
      warningFlash(1);
      break;
    case timeNotSet:
      warningFlash(2);
      break;
    default:
      warningFlash(3);
      break;    
  }
  */
  Alarm.delay(1000);
}

/*-------- Set up pin types --------*/
void setupPins() {
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i <= 3; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
  }
}

/*-------- Hello sequence --------*/
void sayHello() {
  sweepLights(0, OFF, DOWN);
  digitalClockDisplay();
  Serial.println(" USB Christmas Tree Light Controller and Clock by SCJK 2020.");
  digitalClockDisplay();
  Serial.println(" https://github.com/Siddyboy/christmas-lights");
  digitalWrite(RELAY_PINS[0], HIGH);
  digitalWrite(RELAY_PINS[2], HIGH);
  delay(500);
  for(int i = 0; i <= 4; i++) {
    digitalWrite(RELAY_PINS[0], CHANGE);
    digitalWrite(RELAY_PINS[1], CHANGE);
    digitalWrite(RELAY_PINS[2], CHANGE);
    digitalWrite(RELAY_PINS[3], CHANGE);
    delay(500);
  }  
  digitalClockDisplay();
  Serial.println(" Turn lights off to start.");
  sweepLights(0, OFF, DOWN);
}

/*-------- Just Checks Radio Module --------*/
/*Only checks presence (via a status check) */
/*and checks firmware is up to date. Aborts */
/*if there is a problem.                    */
void wifiModuleChecks() {
  digitalClockDisplay();
  Serial.println(" Finding WiFi module.");
  int st = WiFi.status();
  if (st == WL_NO_MODULE) {
    digitalClockDisplay();
    Serial.println(" Communication with Arduino WiFi module failed. Abort!");
    while (true);
  }
  digitalClockDisplay();
  Serial.println(" Found WiFi module.");
  digitalClockDisplay();
  Serial.println(" Checking WiFi module firmware.");
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    digitalClockDisplay();
    Serial.print(" Wifi firmware version ");
    Serial.print(fv);
    Serial.println(" is not current.");
    digitalClockDisplay();
    Serial.println(" Please upgrade the Arduino WiFi module firmware. Abort!");
    while(true);
  }
  digitalClockDisplay();
  Serial.print(" WiFi firmware version ");
  Serial.print(fv);
  Serial.println(" is current.");
}

/*-------- WLAN connect code --------*/
void connectToWlan() {
  digitalClockDisplay();
  Serial.print(" Connecting to WLAN: ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (status != WL_CONNECTED) {
    status = WiFi.status();
    digitalWrite(LED_BUILTIN, CHANGE);
    delay(50);
  }
  digitalClockDisplay();
  Serial.println(" Connected to WLAN");
}

/*-------- WLAN status code --------*/
void printWlanStatus() {
  digitalClockDisplay();
  Serial.print("  WiFi module status: ");
  int st = WiFi.status();
  if (st == WL_IDLE_STATUS) {
    Serial.println("idle");
  }
  else if (st == WL_CONNECTED) {
    Serial.println("connected");
  }
  else {
    Serial.println("unknown");
  }
  digitalClockDisplay();
  Serial.print("  SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  digitalClockDisplay();
  Serial.print("  IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  digitalClockDisplay();
  Serial.print("  Signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");

  byte enc = WiFi.encryptionType();
  digitalClockDisplay();
  Serial.print("  Encryption type: ");
  if (enc == 4) {
    Serial.println("CCMP (WPA)");
  }
  else {
    Serial.println("unknown");
  }
}

/*-------- Obtain IP for NTP server --------*/
void getNtpServerIP() {
  digitalClockDisplay();
  Serial.println(" Getting NTP server IP address.");
  oldIP = timeServerIP;
  int error = WiFi.hostByName(ntpServerName, timeServerIP);
  digitalClockDisplay();
  if(error == 1) {
    Serial.print(" New NTP server IP ");
    Serial.print(timeServerIP);
    Serial.print(" resolved from pool ");
    Serial.println(ntpServerName);
  }
  else {
    Serial.print(" WiFi host-by-name error code: ");
    Serial.println(error);
    digitalClockDisplay();
    Serial.print(" Using previous server IP ");
    timeServerIP = oldIP;
    Serial.println(timeServerIP);
  }
}

/*-------- Listen on local port --------*/
void startSocket() {
  digitalClockDisplay();
  Serial.print(" Starting socket to listen on local port: ");
  Serial.println(localPort);
  int listening = Udp.begin(localPort);
  if (listening == 1) {
    digitalClockDisplay();
    Serial.print(" Listening on local port: ");
    Serial.println(localPort);
  }
  else {
    Serial.println(" There are no sockets available.");
    while(true);
  }
}

/*-------- Digital Clock Display --------*/
void digitalClockDisplay() {
  printDigits(hour());
  Serial.print(":");
  printDigits(minute());
  Serial.print(":");
  printDigits(second());
}

void printDigits(int digits) {
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
  printDigits(onMinute);
  Serial.print(", Off @ ");
  Serial.print(OFF_HOUR);
  printDigits(offMinute);
  
  Serial.print(", Time status: ");
  switch (timeStatus()) {
    case timeNotSet:
      Serial.print("timeNotSet");
      break;
    case timeSet:
      Serial.print("timeSet");
      break;
    case timeNeedsSync:
      Serial.print("timeNeedsSync");
      break;
    default:
      Serial.print("Unknown state");
      break;
  }
  Serial.print(" NTP IP = ");
  Serial.print(timeServerIP);
  Serial.println();
}

/*-------- Warning Flashes --------*/

void warningFlash(int flashes) {
  digitalWrite(LED_BUILTIN, LOW);
  for (int i = 1; i <= flashes*2; i++) {
    digitalWrite(LED_BUILTIN, CHANGE);
    delay(50);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

/*-------- Light Control Alarms --------*/

void quarterPastChime() {
  if (lightsOn == true) {
    digitalClockDisplay();
    Serial.println(" Quarter-past chime");
    chime(1);
  }
}

void halfPastChime() {
  if(lightsOn == true) {
    digitalClockDisplay();
    Serial.println(" Half-past chime.");
    chime(2);
  }
}

void quarterToChime() {
  if(lightsOn == true) {
    digitalClockDisplay();
    Serial.println(" Quarter-to chime.");
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
  int dongs = hourFormat12() + 1;  // Corrected because alarm triggered before the hour.
  if (dongs > 12) {                // Correct the correction for 13 o'clock!
    dongs = 1;
  }
  digitalClockDisplay();
  Serial.print(" Dongs = ");
  Serial.println(dongs);
  if(lightsOn == true) {
    sweepLights(1000, OFF, DOWN);
    delay(3000);
    digitalClockDisplay();
    Serial.println(" Donging...");
    for(int i = 1; i <= dongs; i++) {
      sweepLights(20, ON, UP);
      delay(200);
      sweepLights(20, OFF, DOWN);
      delay(800);
    }
    delay(3000);
    sweepLights(1000, ON, UP);
  }
  else {
    digitalClockDisplay();
    Serial.println(" Dongs are silent.");
  }
}

/*-------- Change lights' status ----------*/

void switchOn() {
  digitalClockDisplay();
  if(alarmLightsOn > 0) {
    Serial.print(" switchOn() called from alarm ID ");
    Serial.println(alarmLightsOn);
  }
  else {
    Serial.println(" switchOn() called directly.");
  }
  lightsOn = true;
  sweepLights(2000, ON, UP);
  digitalClockDisplay();
  offMinute = random(0, 60);
  alarmLightsOff = Alarm.alarmOnce(OFF_HOUR, offMinute, 0, switchOff);
  Serial.print(" New switch-off alarm set for ");
  Serial.print(OFF_HOUR);
  printDigits(offMinute);
  Serial.print(", Alarm ID ");
  Serial.println(alarmLightsOff);
}

void switchOff() {
  digitalClockDisplay();
  if(alarmLightsOff > 0) {
    Serial.print(" switchOff() called from alarm ID ");
    Serial.println(alarmLightsOff);
  }
  else {
    Serial.println(" switchOff() called directly.");
  }
  lightsOn = false;
  sweepLights(2000, OFF, DOWN);
  digitalClockDisplay();
  onMinute = random(0, 60);
  alarmLightsOn = Alarm.alarmOnce(ON_HOUR, onMinute, 0, switchOn);
  Serial.print(" New switch-on alarm set for ");
  Serial.print(ON_HOUR);
  printDigits(onMinute);
  Serial.print(", Alarm ID ");
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
  Serial.println("Transmit NTP packet.");
  sendNTPpacket(timeServerIP);
  delay(1500);
  if(Udp.parsePacket()) {
    Serial.println("Received NTP packet.");
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;
    Serial.print("New epoch = ");
    Serial.println(epoch);
    return epoch;
  }
  Serial.println("No NTP response :-(");
  return 0;
}

/*-------- Send an NTP request to the time server --------*/
/* Send the request to the time server at the given       */
/* address. From TimeNTP_ESP8266WiFi.ino example.         */
void sendNTPpacket(IPAddress & address) {
  memset(packetBuffer, 0, NTP_PACKET_SIZE);  // Set all bytes in the buffer to 0
  // Initialise values needed to form NTP request
  packetBuffer[0]  = 0b11100011;             // LI, version, mode 
  packetBuffer[1]  = 0;                      // Stratum, or type of clock
  packetBuffer[2]  = 6;                      // Polling interval
  packetBuffer[3]  = 0xEC;                   // Peer clock precision
  // [Eight bytes (4 to 11) of zero for root delay and root dispersion]
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // Send a packet to request timestamp
  Udp.beginPacket(address, 123);             // NTP requests to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

