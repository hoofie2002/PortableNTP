/* Portable NTP */

#include <Arduino.h>
#include <ETH.h>
#include <ESP32Time.h>
//#include <Timezone.h>
#include <LiquidCrystal_I2C.h>
#include <HardwareSerial.h>
#include <TimeLib.h>     // Time functions  https://github.com/PaulStoffregen/Time
#include <TinyGPS.h> // GPS parsing     https://github.com/mikalhart/TinyGPS
#include <Wire.h>        // LCD
#include <WiFi.h>

#define GPSRX (16)
#define GPSTX (17)

#define NTP_PORT 123
#define NTP_PACKET_SIZE 48
byte packetBuffer[NTP_PACKET_SIZE];
WiFiUDP Udp;


const char * networkName = "Telstra1A155E";
const char * networkPswd = "fx8tvdg9qdy527vk";
const int LOCK_LED = 2;

int lcdColumns = 16;
int lcdRows = 2;

time_t prevDisplay = 0; // when the digital clock was displayed

// Offset hours from gps time (UTC)
//const int offset = 1;   // Central European Time
//const int offset = -5;  // Eastern Standard Time (USA)
//const int offset = -4;  // Eastern Daylight Time (USA)
//const int offset = -8;  // Pacific Standard Time (USA)
//const int offset = -7;  // Pacific Daylight Time (USA)
const int offset = 8;   // West Australian Standard Time
ESP32Time rtc(0); // Since we are running this as an NTP server we do not apply an offset - runs in UTC

// Constants and global variables
const unsigned long oneSecond_inMilliseconds = 1000;                              // one second in milliseconds
const unsigned long oneMinute_inMilliseconds = 60 * oneSecond_inMilliseconds;     // one minute in milliseconds
const unsigned long thirtyMinutes_inMilliseconds = 30 * oneMinute_inMilliseconds; // 30 minutes in milliseconds
const long oneSecond_inMicroseconds_L = 1000000;                                  // one second in microseconds (signed long)
const double oneSecond_inMicroseconds_D = 1000000.0;                              // one second in microseconds (double)

TinyGPS gps;
HardwareSerial gpsSerial(2); // Use UART2
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);


uint8_t ryear, rmonth, rweekday, rday, rhour, rminute, rsecond;
bool rperiod = 0;
bool displayTimeZone = true;
bool debugIsOn = true;

void setup() {
  // put your setup code here, to run once:
  lcd.init();
  lcd.backlight();
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPSRX, GPSTX); // Initialize the serial port
  pinMode(LOCK_LED, OUTPUT);

  connectToWiFi(networkName, networkPswd);

  startUDPServer();

  lcd.setCursor(0,0);
  lcd.print("NTP up");

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
        rtc.setTime(Second, Minute, Hour, Day, Month, Year, 0);

        digitalWrite(LOCK_LED, HIGH);
      }
    }
  }
  if (timeStatus()!= timeNotSet) {
    if (now() != prevDisplay) { //update the display only if the time has changed
      prevDisplay = now();
      digitalClockDisplay();  
    }
  }

  processNTPRequests();
}

void startUDPServer()
{

  Udp.begin(NTP_PORT);
}

void processNTPRequests()
{

  unsigned long replyStartTime = micros();

  int packetSize = Udp.parsePacket();

  if (packetSize == NTP_PACKET_SIZE) // an NTP request has arrived
  {

    // store sender ip for later use
    IPAddress remoteIP = Udp.remoteIP();

    // read the data from the packet into the buffer for later use
    Udp.read(packetBuffer, NTP_PACKET_SIZE);

    // hold here if and while the date and time are being refreshed
    // when ok to proceed place a hold on using the mutex to prevent the date and time from being refreshed while the reply packet is being built
    //if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    //{
      // send NTP reply
      sendNTPpacket(remoteIP, Udp.remotePort());
      //xSemaphoreGive(mutex);
    //};

    // report query in serial monitor
    // note: unlike other serial monitor writes in this sketch, this particular write is on the critical path for processing NTP requests.
    // while it does not delay the response to an initial NTP request, if subsequent NTP requests are queued up to run directly afterward
    // this serial monitor write will delay responding to the queued request by approximately 1 milli second.
    if (debugIsOn)
    {

      String dateLine = "";
      String timeLine = "";
      GetAdjustedDateAndTimeStrings(rtc.getEpoch(), dateLine, timeLine);
      String updatemessage = "Query from " + remoteIP.toString() + " on " + dateLine + " at " + timeLine;
      Serial.println(updatemessage);
    };
  }
  else
  {
    if (packetSize > 0)
    {
      Udp.flush(); // not sure what this incoming packet is, but it is not an ntp request so get rid of it
      if (debugIsOn)
        Serial.println("Invalid request received on port " + String(NTP_PORT) + ", length =" + String(packetSize));
    };
  };
}

void GetAdjustedDateAndTimeStrings(time_t UTC_Time, String &dateString, String &timeString)
{

  // adjust utc time to local time
  time_t now_Local_Time = UTC_Time;

  // format dateLine

  dateString = String(year(now_Local_Time));

  dateString.concat("-");

  if (month(now_Local_Time) < 10)
    dateString.concat("0");

  dateString.concat(String(month(now_Local_Time)));

  dateString.concat("-");

  if (day(now_Local_Time) < 10)
    dateString.concat("0");

  dateString.concat(String(day(now_Local_Time)));

  // format timeLine

  timeString = String(hourFormat12(now_Local_Time));

  timeString.concat(":");

  if (minute(now_Local_Time) < 10)
    timeString.concat("0");

  timeString.concat(String(minute(now_Local_Time)));

  timeString.concat(":");

  if (second(now_Local_Time) < 10)
    timeString.concat("0");

  timeString.concat(String(second(now_Local_Time)));

  if (isAM(now_Local_Time))
    timeString.concat(" AM ");
  else
    timeString.concat(" PM ");
  
};

// send NTP reply
void sendNTPpacket(IPAddress remoteIP, int remotePort)
{

  // set the receive time to the current time
  uint64_t receiveTime_uint64_t = getCurrentTimeInNTP64BitFormat();

  // Initialize values needed to form NTP request

  // LI: 0, Version: 4, Mode: 4 (server)
  // packetBuffer[0] = 0b00100100;
  // LI: 0, Version: 3, Mode: 4 (server)
  packetBuffer[0] = 0b00011100;

  // Stratum, or type of clock
  packetBuffer[1] = 0b00000001;

  // Polling Interval
  packetBuffer[2] = 4;

  // Peer Clock Precision
  // log2(sec)
  // 0xF6 <--> -10 <--> 0.0009765625 s
  // 0xF7 <--> -9 <--> 0.001953125 s
  // 0xF8 <--> -8 <--> 0.00390625 s
  // 0xF9 <--> -7 <--> 0.0078125 s
  // 0xFA <--> -6 <--> 0.0156250 s
  // 0xFB <--> -5 <--> 0.0312500 s
  packetBuffer[3] = 0xF7;

  // 8 bytes for Root Delay & Root Dispersion
  // root delay
  packetBuffer[4] = 0;
  packetBuffer[5] = 0;
  packetBuffer[6] = 0;
  packetBuffer[7] = 0;

  // root dispersion
  packetBuffer[8] = 0;
  packetBuffer[9] = 0;
  packetBuffer[10] = 0;
  packetBuffer[11] = 0x50;

  // time source (namestring)
  packetBuffer[12] = 71; // G
  packetBuffer[13] = 80; // P
  packetBuffer[14] = 83; // S
  packetBuffer[15] = 0;

  // get the current time and write it out as the reference time to bytes 16 to 23 of the response packet
  uint64_t referenceTime_uint64_t = getCurrentTimeInNTP64BitFormat();

  packetBuffer[16] = (int)((referenceTime_uint64_t >> 56) & 0xFF);
  packetBuffer[17] = (int)((referenceTime_uint64_t >> 48) & 0xFF);
  packetBuffer[18] = (int)((referenceTime_uint64_t >> 40) & 0xFF);
  packetBuffer[19] = (int)((referenceTime_uint64_t >> 32) & 0xFF);
  packetBuffer[20] = (int)((referenceTime_uint64_t >> 24) & 0xFF);
  packetBuffer[21] = (int)((referenceTime_uint64_t >> 16) & 0xFF);
  packetBuffer[22] = (int)((referenceTime_uint64_t >> 8) & 0xFF);
  packetBuffer[23] = (int)(referenceTime_uint64_t & 0xFF);

  // copy transmit time from the NTP original request to bytes 24 to 31 of the response packet
  packetBuffer[24] = packetBuffer[40];
  packetBuffer[25] = packetBuffer[41];
  packetBuffer[26] = packetBuffer[42];
  packetBuffer[27] = packetBuffer[43];
  packetBuffer[28] = packetBuffer[44];
  packetBuffer[29] = packetBuffer[45];
  packetBuffer[30] = packetBuffer[46];
  packetBuffer[31] = packetBuffer[47];

  // write out the receive time (it was set above) to bytes 32 to 39 of the response packet
  packetBuffer[32] = (int)((receiveTime_uint64_t >> 56) & 0xFF);
  packetBuffer[33] = (int)((receiveTime_uint64_t >> 48) & 0xFF);
  packetBuffer[34] = (int)((receiveTime_uint64_t >> 40) & 0xFF);
  packetBuffer[35] = (int)((receiveTime_uint64_t >> 32) & 0xFF);
  packetBuffer[36] = (int)((receiveTime_uint64_t >> 24) & 0xFF);
  packetBuffer[37] = (int)((receiveTime_uint64_t >> 16) & 0xFF);
  packetBuffer[38] = (int)((receiveTime_uint64_t >> 8) & 0xFF);
  packetBuffer[39] = (int)(receiveTime_uint64_t & 0xFF);

  // get the current time and write it out as the transmit time to bytes 40 to 47 of the response packet
  uint64_t transmitTime_uint64_t = getCurrentTimeInNTP64BitFormat();

  packetBuffer[40] = (int)((transmitTime_uint64_t >> 56) & 0xFF);
  packetBuffer[41] = (int)((transmitTime_uint64_t >> 48) & 0xFF);
  packetBuffer[42] = (int)((transmitTime_uint64_t >> 40) & 0xFF);
  packetBuffer[43] = (int)((transmitTime_uint64_t >> 32) & 0xFF);
  packetBuffer[44] = (int)((transmitTime_uint64_t >> 24) & 0xFF);
  packetBuffer[45] = (int)((transmitTime_uint64_t >> 16) & 0xFF);
  packetBuffer[46] = (int)((transmitTime_uint64_t >> 8) & 0xFF);
  packetBuffer[47] = (int)(transmitTime_uint64_t & 0xFF);

  // send the reply
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

uint64_t getCurrentTimeInNTP64BitFormat()
{

  const uint64_t numberOfSecondsBetween1900and1970 = 2208988800;

  uint64_t clockSecondsSinceEpoch = numberOfSecondsBetween1900and1970 + (uint64_t)rtc.getEpoch();
  long clockMicroSeconds = (long)rtc.getMicros();

  // as one might infer clockMicroSeconds is in microseconds (i.e. 1 second = 1,000,000 microseconds)
  //
  // accordingly, if the clockMicroSeconds is greater than one million ...
  //   for every million that is over:
  //     add 1 (second) to clockSecondsSinceEpoch, and
  //     reduce the clockMicroSeconds by one million (microseconds)
  //
  // likewise ...
  //
  // if the clockMicroSeconds is less than zero:
  //   for every million that is under zero:
  //     subtract (second) from clockSecondsSinceEpoch, and
  //     increase the clockMicroSeconds by one million (microseconds)

  while (clockMicroSeconds > oneSecond_inMicroseconds_L)
  {
    clockSecondsSinceEpoch++;
    clockMicroSeconds -= oneSecond_inMicroseconds_L;
  };

  while (clockMicroSeconds < 0L)
  {
    clockSecondsSinceEpoch--;
    clockMicroSeconds += oneSecond_inMicroseconds_L;
  };

  // for the next two lines to be clear, please see: https://tickelton.gitlab.io/articles/ntp-timestamps/

  double clockMicroSeconds_D = (double)clockMicroSeconds * (double)(4294.967296);
  uint64_t ntpts = ((uint64_t)clockSecondsSinceEpoch << 32) | (uint64_t)(clockMicroSeconds_D);

  return ntpts;
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
    digitalWrite(LOCK_LED, ledState);
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
