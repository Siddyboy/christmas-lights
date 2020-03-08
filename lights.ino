#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Time.h>
#include <TimeAlarms.h> // Don't forget to change the maximum number of alarms in the header of TimeAlarms.h
#include "arduino_secrets.h"

bool lightsOn = false;
const unsigned long del = 100;

const int USB1 = 4;
const int USB2 = 7;
const int USB3 = 8;
const int USB4 = 12;

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int status = WL_IDLE_STATUS;

unsigned int localPort = 2390;

// IPAddress timeServer(129, 6, 15, 28); //time.nist.gov
IPAddress timeServer(143, 210, 16, 201); //0.uk.pool.ntp.org

WiFiUDP Udp;

void setup() {
  pinMode(USB1, OUTPUT);
  pinMode(USB2, OUTPUT);
  pinMode(USB3, OUTPUT);
  pinMode(USB4, OUTPUT);
  
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

  for(int i = 0; i <= 23; i++) {
    Alarm.alarmRepeat(i, 0, 0, hourChime);
    Alarm.alarmRepeat(i, 15, 0, quarterPastChime);
    Alarm.alarmRepeat(i, 30, 0, halfPastChime);
    Alarm.alarmRepeat(i, 45, 0, quarterToChime);
  }  
  
  Alarm.alarmRepeat(21, 20, 0, lightsTurnOn);
  Alarm.alarmRepeat(22, 20, 0, lightsTurnOff);

// ALso make lights blink an error code (that looks like nice flashing xmas tree lights :) ) when there is a problem with NTP synch or wifi link or whatever.
// differnet blink codes for differnet enrrors!
}

time_t prevDisplay = 0;

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
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.print(" ");
  Serial.println(lightsOn);
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
    sweepOff();
    delay(500);
    sweepOn();
    Serial.println("Quarter Past Chime.");
  }
}

void halfPastChime() {
  if(lightsOn == true) {
    sweepOff();
    delay(500);
    sweepOn();
    delay(500);
    sweepOff();
    delay(500);
    sweepOn();
    digitalClockDisplay();
    Serial.println("Half Past Chime.");
  }
}

void quarterToChime() {
  if(lightsOn == true) {
    sweepOff();
    delay(500);
    sweepOn();
    delay(500);
    sweepOff();
    delay(500);
    sweepOn();
    delay(500);
    sweepOff();
    delay(500);
    sweepOn();
    digitalClockDisplay();
    Serial.println("Quater to Chime.");
  }
}

void hourChime() {
  Serial.print("Hour Dongs");
}

void lightsTurnOn() {
  lightsOn = true;
  sweepOn();
  Serial.print("Lights turned on!");
}

void lightsTurnOff() {
  lightsOn = false;
  sweepOff();
  Serial.print("Lights turned off!");
}

void sweepOn() {
  digitalWrite(USB1, HIGH);
  delay(del);
  digitalWrite(USB2, HIGH);
  delay(del);
  digitalWrite(USB3, HIGH);
  delay(del);
  digitalWrite(USB4, HIGH);
}

void sweepOff() {
  digitalWrite(USB1, LOW);
  delay(del);
  digitalWrite(USB2, LOW);
  delay(del);
  digitalWrite(USB3, LOW);
  delay(del);
  digitalWrite(USB4, LOW);
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
    
