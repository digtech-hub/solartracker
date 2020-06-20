#include <time.h>
#include <EEPROM.h>

#define STASSID "XXXXXXXX"  // your network SSID (name)
#define STAPSK  "YYYYYYYY"  // your network password
#define LATITUDE 35.658;    // ido (e.g. Tokyo tower)
#define LONGTITUDE 139.745; // keido

//#define MEASURE
//#define SWITCH_TEST
//#define MOTOR_TEST

/// begin of wifi
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
const char * ssid = STASSID;
const char * pass = STAPSK;
unsigned int localPort = 2390;      // local port to listen for UDP packets

/* Don't hardwire the IP address or we won't get the benefits of the pool.
    Lookup the IP address for the host name instead */
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

void setupWIFI() {
  // connecting to a WiFi network
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  int count = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    count++;
    if (count >= 50) {
      Serial.println("\nGive up connection! Reset!");
      Serial.println("Reset!");
      ESP.reset();
    }
  }
  Serial.println();
  Serial.print("WiFi connected. IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
  delay(1000);
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address) {
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}
/// end of wifi

bool getDate(int &yearDay, double &hour) {
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(2000);

  int packetSize = 0;
  if (packetSize = udp.parsePacket() != 0) {
    Serial.print("packet received, length=");
    Serial.println(packetSize);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    //Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    //Serial.println(epoch);
    // print the hour, minute and second:
    epoch += 9 * 60 * 60;                    // UTC -> JPN. UTC is the time at Greenwich Meridian (GMT)

    time_t time = epoch;
    struct tm *now = localtime(&time);
    char str[80];
    strftime(str, sizeof(str), "%a %Y-%m-%d %H:%M:%S", now);
    Serial.print("The JPN time is ");
    Serial.println(str);
    yearDay = now->tm_yday + 1;
    hour = now->tm_hour + now->tm_min / 60.0 + now->tm_sec / (60.0 * 60.0);
    Serial.print("Year day=");
    Serial.print(yearDay);
    Serial.print(" Hour=");
    Serial.println(hour);
    return true;
  }
  Serial.println("Cannot get packet.");
  return false;
}

struct MotorPosition {
  float angle;
  int rotation;
};

// 80 mm = 10 rounds
// 265 mm = 33 rounds
// 527/16 = 32.9
const MotorPosition motorPositionTable[] 
   = {{132.0,  0}, {137.0,  2}, {141.0,  4}, {146.0,  6}, {151.0,  8}, 
      {155.0, 10}, {160.0, 12}, {165.0, 14}, {170.0, 16}, {175.0, 18},
      {180.0, 20}, {185.0, 22}, {190.0, 24}, {195.0, 26}, {200.5, 28},
      {205.0, 30}, {210.0, 32}, {216.5, 34}};
      
int getSolarPositionTableSize() {
  return sizeof(motorPositionTable) / sizeof(MotorPosition);
}

int getSolarPosition(double azimuth) {
  int arraySize = getSolarPositionTableSize();
  for (int idx = 0; idx < arraySize; idx++) {
    if (motorPositionTable[idx].angle > azimuth) {
      if (idx == 0) {
        return motorPositionTable[idx].rotation;
      }
      float rotate = motorPositionTable[idx - 1].rotation + (motorPositionTable[idx].rotation - motorPositionTable[idx - 1].rotation) *
                     (azimuth - motorPositionTable[idx - 1].angle) / (motorPositionTable[idx].angle - motorPositionTable[idx - 1].angle);
      return floor(rotate);
    }
  }
  return motorPositionTable[arraySize - 1].rotation;
}

class Motor {
  public:
  Motor() {
    motorSwitchGpio = 12;
    rotationGpio = 4;
    speedGpio = 5;
    switchGpio = 13;
  }
  void setup() {
    pinMode(motorSwitchGpio, OUTPUT);
    digitalWrite(motorSwitchGpio, LOW);  // right away power down to save electricity.
    pinMode(rotationGpio, OUTPUT);
    pinMode(speedGpio, OUTPUT);
    pinMode(switchGpio, INPUT_PULLUP);
    powerOff();
  }
  void powerOn() {
    motorOn = true;
    digitalWrite(motorSwitchGpio, HIGH);
    Serial.println("* Power ON");
    delay(500);
  }
  void powerOff() {
    motorOn = false;
    digitalWrite(motorSwitchGpio, LOW);
    Serial.println("* Power OFF");
  }
  void rotate() {
    yield(); // to avoid watchdog timer
    for (int x = 0; x < 200; x++) {
      digitalWrite(rotationGpio, HIGH); 
      delayMicroseconds(speed);
      digitalWrite(rotationGpio, LOW); 
      delayMicroseconds(speed);
    }
  }
  void rotateOne() {
    setDirection();
    rotate();
  }
  void setDirection() {
    switch (direction) {
      case 0: digitalWrite(speedGpio, HIGH); break;
      case 1: digitalWrite(speedGpio, LOW);  break;
    }
  }
  bool rotate(int frequency) {
    if (!motorOn) {
      return false;
    }
    setDirection();
    // frequency:16 = 1 round because of 16th step
    Serial.print("Rotate: ");
    for (int r = 0; r < frequency; r++) {
      if (switchOn()) {
        Serial.println();
        return false;
      }
      if (r % 16 == 0) {
        Serial.print(".");
      }
      rotate();
    }
    Serial.println();
    return true;
  }
  
  bool switchOn() { return digitalRead(switchGpio) == LOW; }

  bool checkMotorSwitch() {
    if (motorOn && switchOn()) {
      // reverse the motor
      direction = !direction;
      for (int r = 0; r < 50; r++) {
        if (!switchOn()) {
          break;
        }
        rotateOne();
      }
      if (switchOn()) {
        Serial.println("checkMotorSwitch: something wrong");
        return true;
      }
      // one more to make sure.
      rotateOne();
      return true;
    }
    return false;
  }
  
  void checkSwitch() {
    if (switchOn()) {
      if (motorOn) {
        // stop the motor
        powerOff();
      }
      delay(500);  // ms
      if (!switchOn()) {
        // start
        powerOn();
        direction = !direction;
      }
      delay(1000);  // ms
    }
  }
  
  bool goEdge() {
    powerOn();
    if (switchOn()) {
      Serial.println("goEdge: switch is ON.");
      // already switch on, that means it is at any edge.
      // opposite edge?
      for (int i = 0; i < 8; i++) {
        rotateOne();
      }
      if (switchOn()) {
        Serial.println("goEdge: switch is ON again, after moving.");
        // not opposite site, already the edge.
        direction = !direction;
        for (int i = 0; i < 8; i++) {
          rotateOne();
        }
        direction = !direction;
        powerOff();
        return true;
      } else {
        // opposite size
      }
    }
    // go east during switch is off
    for (int r = 1; r < 50 * 16; r++) { // if r reaches 50, it means that the panels cannot move.
      if (checkMotorSwitch()) {
        powerOff();
        return true;
      }
      rotateOne(); // 1 : 1/16 round because of 16th step
    }
    powerOff();
    return false;
  }
  
  void setEast() { direction = 0; }
  void setWest() { direction = 1; }
  
  bool goEast() {
    Serial.println("go East");
    setEast();
    return goEdge();
  }
  
  bool goWest() {
    Serial.println("go West");
    setWest();
    return goEdge();
  }

  int motorSwitchGpio;
  int rotationGpio;
  int speedGpio;
  int switchGpio; 
  
  int direction = 0;
  int speed = 500;
  bool motorOn = false;
};

Motor motor;

void setHour(double hour) {
  double *ptr = (double *)&EEPROM[0];
  *ptr = hour;
  EEPROM.commit();
}

double getHour() {
  byte v = 0;
  for (int i = 0; i < 8; i++) {
    v |= EEPROM[i];
  }
  if (v == 0) {
    return -1.0; // not set
  }
  double *ptr = (double *)&EEPROM[0];
  return *ptr;
}

void setRunMode(byte mode) {
  EEPROM[8] = mode;
  EEPROM.commit();
}

byte getRunMode() {
  return EEPROM[8];
}

void measure() {
  motor.setup();
  Serial.println("goEast");
  int d = 10;
  motor.goEast();
  delay(d * 1000);
  // 80mm = 10 rounds
  // 265mm = 33 rounds
  motor.setWest();
  Serial.println("go west");
  for (int c = 0; c < 35; c++) {
    Serial.print("c=");
    Serial.println(c);
    Serial.print("r=");
    Serial.println(c * 2);
    motor.powerOn();
    for (int r = 0; r < 2 * 16; r++) {
      motor.rotateOne();
      if (motor.checkMotorSwitch()) {
        delay(10 * 1000);
        return;
      }
    }
    motor.powerOff();
    delay(d * 1000);
  }
}

int getFlashButtonStatus() {
  const int esp12LED = 2;
  pinMode(esp12LED, OUTPUT);
  pinMode(0, INPUT_PULLUP);
  for (int i = 0; i < 10; i++) {
    digitalWrite(esp12LED, LOW); // Turn on
    delay(100);
    digitalWrite(esp12LED, HIGH); // Turn off
    delay(100);
  }
  if (digitalRead(0) == HIGH) {
    Serial.println("Flash button is released. (0)");
    return 0;
  }
  Serial.println("Flash button is pressed.");
  for (int i = 0; i < 20; i++) {
    digitalWrite(esp12LED, LOW); // Turn on
    delay(50);
    digitalWrite(esp12LED, HIGH); // Turn off
    delay(50);
  }
  if (digitalRead(0) == HIGH) {
    Serial.println("Flash button is released. (1)");
    return 1;
  }
  digitalWrite(esp12LED, LOW); // Turn on
  delay(3000);
  digitalWrite(esp12LED, HIGH); // Turn off
  Serial.println("Flash button is pushed. (2)");
  return 2;
}

void setup() {
  delay(100);
  motor.setup();  // only to stop the motor right away.
  delay(1000);
  Serial.begin(74880);
  Serial.println("\n\nSun tracking start.");

  EEPROM.begin(512);
#ifdef MEASURE
  // Important! To measure directions of the panels.
  Serial.println("Measure directions.");
  measure();
  return;
#endif
#ifdef SWITCH_TEST
  for (;;) {
    delay(500);
    if (motor.switchOn()) {
      Serial.println("On");
    } else {
      Serial.println("Off");
    }
  }
#endif
#ifdef MOTOR_TEST
  // check motor.
  motor.setup();
  for (;;) {
    Serial.println("goWestt");
    motor.goWest();
    Serial.println("goEast");
    motor.goEast();
  }
  return;
#endif
  // Normal procedure
  setupWIFI();
  int yearDay;
  double hour;
  bool stat = false;

  for (int i = 0; i < 10; i++) {
    if (stat = getDate(yearDay, hour)) {
      break;
    }
    delay(500);
  }
  if (!stat) {
    ESP.restart();
  }
  double prevHour = getHour();
  byte runMode = getFlashButtonStatus();
  if (runMode > 0) {
    Serial.print("Initialization is set. ");
    Serial.println(runMode);
    if (runMode == 2) {
      if (getRunMode() == 0) {
        // run mode 2 init
        prevHour = 4.5;
      } else {
        runMode = 0; // clear!
        Serial.println("run mode is clear");
      }
      setRunMode(runMode);
    } 
  } else {
    runMode = getRunMode();
    if (runMode != 0) {
      Serial.print("Run mode=");
      Serial.println(runMode);
    }
  }
  if (runMode == 2) {
    // for test
    hour = prevHour + 1.0;
    if (hour > 24.0) hour -= 24.0; 
  }
  Serial.print("*** Hour=");
  Serial.print(prevHour);
  Serial.print("->");
  Serial.println(hour);
  double azimuth, altitude;
  getSunPosition(yearDay, hour, azimuth, altitude);
  setHour(hour);
  if (hour > 20.0 && hour < 21.0) {
    motor.goEast();
    deepSleep(hour, runMode);
  }
  Serial.print("sun pos=");
  Serial.print(altitude);
  Serial.print(":");
  Serial.print(azimuth);
  Serial.println();
  int pos = getSolarPosition(azimuth);
  Serial.print("panel pos=");
  Serial.println(pos);
  if (pos >= 0 && prevHour >= 0.0 || runMode == 1) {
    double prevAzimuth, prevAltitude;
    getSunPosition(yearDay, prevHour, prevAzimuth, prevAltitude);
    int prevPos = getSolarPosition(prevAzimuth);
    Serial.print("prev panel pos=");
    Serial.println(prevPos);
    if (runMode == 1) {
      prevPos = 0;
    }
    int rounds = (pos - prevPos) * 16;
    Serial.print("rounds=");
    Serial.print(rounds);
    Serial.print(",");
    Serial.println(rounds / 16);
    rounds = rounds < 0 ? 0 : rounds;
    if (rounds > 0 || runMode == 1) {
      if (motor.switchOn() || runMode == 1) {
        if (runMode == 1) {
          Serial.println("Initialize!!");
        }
        if (!motor.goEast()) {
          Serial.println("Cannot move the pannels. Something wrong!!!!!!!!");
          deepSleep(hour, runMode);
        }
        rounds = pos * 16;
        if (runMode == 1 && (hour > 20.0 || hour < 3.0)) {
          // after going East, moving is unnecessary.
          rounds = 0;
        }
      }
      if (rounds != 0) {
        motor.setWest();
        motor.powerOn();
        motor.rotate(rounds);
        motor.powerOff();
      }
    }
  }
  deepSleep(hour, runMode);
}

void getSunPosition(int yearDay, double hour, double &azimuth, double &altitude) {
  // http://butterflyandsky.fan.coocan.jp/sky2/calc/sunalt.html
  // http://www.es.ris.ac.jp/~nakagawa/met_cal/solar.html
  double latitude = LATITUDE; // ido
  double longitude = LONGTITUDE; // keido
  double latitudeRad = latitude * M_PI / 180.0;
  double theta = (yearDay - 1) * 2.0 * M_PI / 365.0;
  double delta = 0.006918 - 0.399912 * cos(theta) + 0.070257  * sin(theta) - 0.006758 * cos(2.0 * theta) + 
                 0.000907 * sin(2.0 * theta) - 0.002697 * cos(3.0 * theta) + 0.00148 * sin(3.0 * theta);
  double eq = 0.000075 + 0.001868 * cos(theta) - 0.032077 * sin(theta) - 0.014615 * cos(2.0 * theta) - 0.040849 * sin(2.0 * theta);
  double jstmLongitude = 135.0;
  double ha = (hour - 12.0) * M_PI / 12.0 + (longitude - jstmLongitude) * M_PI / 180.0 + eq;
  double alpha = asin(sin(latitudeRad) * sin(delta) + cos(latitudeRad) * cos(delta) * cos(ha));
  double phi = atan2( cos(latitudeRad) * cos(delta) * sin(ha), sin(latitudeRad) * sin(alpha) - sin(delta));
  // alpha: horizon is zero
  altitude = alpha * 180.0 / M_PI;
  // phi: North is zero degree, east is 90 degrees, west is 270 degrees.
  azimuth = phi * 180.0 / M_PI;
  azimuth += 180.0;  // North is 0.
}

void deepSleep(double hour, byte runMode) {
  unsigned long sec; // interval time
  if (hour < 6.0 || hour > 18.4) {
    sec = 60 * 60 * 1; // 1 hour
  } else {
    sec = 60 * 15; // 15 min
  }
  // sleep
  if (runMode == 2) {
    // For test
    sec = 3;
    Serial.print("wake up ");
    Serial.print(sec);
    Serial.print(" minitus lator.");
  }
  Serial.print("deep sleep sec=");
  Serial.println(sec);
  sec *= 1000 * 1000;
  Serial.println(sec);
  Serial.print("wake up at ");
  Serial.println((float)sec / 1000.0 / 1000.0 / 60.0 / 60.0 + hour);  
  ESP.deepSleep(sec);  // sleep until next interval
  Serial.println("Bye");
}

void loop() {
}
