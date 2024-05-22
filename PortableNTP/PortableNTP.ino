/* Portable NTP */

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <HardwareSerial.h>
#include <TimeLib.h>     // Time functions  https://github.com/PaulStoffregen/Time
#include <TinyGPS.h> // GPS parsing     https://github.com/mikalhart/TinyGPS
#include <Wire.h>        // OLED and DS3231 necessary
#include <RTCDS1307.h>   // RTC functions
#include <WiFi.h>

#define GPSRX (16)
#define GPSTX (17)

const char * networkName = "Telstra1A155E";
const char * networkPswd = "fx8tvdg9qdy527vk";
const int LED_PIN = 5;

time_t prevDisplay = 0; // when the digital clock was displayed

// Offset hours from gps time (UTC)
//const int offset = 1;   // Central European Time
//const int offset = -5;  // Eastern Standard Time (USA)
//const int offset = -4;  // Eastern Daylight Time (USA)
//const int offset = -8;  // Pacific Standard Time (USA)
//const int offset = -7;  // Pacific Daylight Time (USA)
const int offset = 8;   // West Australian Standard Time


TinyGPS gps;
RTCDS1307 rtc(0x68);
HardwareSerial gpsSerial(2); // Use UART2


uint8_t ryear, rmonth, rweekday, rday, rhour, rminute, rsecond;
bool rperiod = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPSRX, GPSTX); // Initialize the serial port
  pinMode(LED_PIN, OUTPUT);
  rtc.begin();


  connectToWiFi(networkName, networkPswd);

}

void loop()
{
  while (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) { // process gps messages
      // when TinyGPS reports new data...
      unsigned long age;
      int Year;
      byte Month, Day, Hour, Minute, Second;
      gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);
      if (age < 500) {
        // set the Time to the latest GPS reading
        setTime(Hour, Minute, Second, Day, Month, Year);
        adjustTime(offset * SECS_PER_HOUR);
      }
    }
  }
  if (timeStatus()!= timeNotSet) {
    if (now() != prevDisplay) { //update the display only if the time has changed
      prevDisplay = now();
      digitalClockDisplay();  
    }
  }
}

void connectToWiFi(const char * ssid, const char * pwd)
{
  int ledState = 0;

  printLine();
  Serial.println("Connecting to WiFi network: " + String(ssid));

  WiFi.begin(ssid, pwd);

  while (WiFi.status() != WL_CONNECTED) 
  {
    // Blink LED while we're connecting:
    digitalWrite(LED_PIN, ledState);
    ledState = (ledState + 1) % 2; // Flip ledState
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void printLine()
{
  Serial.println();
  for (int i=0; i<30; i++)
    Serial.print("-");
  Serial.println();
}

void digitalClockDisplay(){
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
  Serial.println(); 
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
