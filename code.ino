#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <iarduino_RTC.h>
#include <EEPROM.h>

#define SERVO_VERT_PIN 8 
#define SERVO_HOR_PIN  6  
#define ONE_WIRE_BUS   9  
#define WIND_PIN       A4

// Настройка скорости сервоприводов
const int SERVO_SPEED_MS = 150; // Миллисекунд на 1 градус. чем больше значение, тем больше плавность
// RST=5, CLK=4, DAT=3
iarduino_RTC time(RTC_DS1302, 5, 4, 3);

const float latitude = 55.75;
const float longitude = 37.61;
const int timezone = 3;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Servo servoVert;
Servo servoHor;

int currentVert = 90; 
int currentHor = 90;
int targetVert = 90;
int targetHor = 90;
int tol = 10; 

unsigned long lastTempTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastServoTime = 0;
float currentTemp = -127.0;

int eeAddress = 0; 
String lastCriticalMode = "";

void setup() {
  Serial.begin(9600);
  
  // time.settime(0, 30, 12, 6, 3, 26, 6);
  
  // Начальную точка сервоприводов
  servoVert.write(currentVert);
  servoHor.write(currentHor);
  
  servoVert.attach(SERVO_VERT_PIN);
  servoHor.attach(SERVO_HOR_PIN);
  
  sensors.begin();
  

  EEPROM.get(1020, eeAddress);
  if (eeAddress < 0 || eeAddress > 1000) eeAddress = 0;
}

void loop() {
  if (Serial.available() > 0) {
    if (Serial.read() == 'R') dumpEEPROM();
  }

  time.gettime();
  int lt = analogRead(A0); int rt = analogRead(A1);
  int ld = analogRead(A2); int rd = analogRead(A3);
  int wind = analogRead(WIND_PIN);
  int avgLight = (lt + rt + ld + rd) / 4; 

  if (millis() - lastTempTime > 2000) {
    sensors.requestTemperatures();
    currentTemp = sensors.getTempCByIndex(0);
    if (currentTemp > 80.0 && currentTemp != -127.0) targetVert = constrain(targetVert - 5, 0, 90); 
    lastTempTime = millis();
  }

  String currentMode = "";
  bool isCritical = false;
  
  if (wind >= 800) {
    targetVert = 90; 
    currentMode = "STORM";
    isCritical = true;
  }
  else if (currentTemp > 80.0 && currentTemp != -127.0) {
    currentMode = "OVERHEAT";
    isCritical = true;
  }
  else if (avgLight < 30) { 
    float az, el;
    calculateSolarPosition(time.Hours, time.minutes, time.day, time.month, latitude, longitude, &az, &el);
    if (el > 0) {
      targetHor = map(constrain(az, 90, 270), 90, 270, 0, 180);
      targetVert = constrain(el, 0, 90);
    }
    currentMode = "RTC_MATH";
  }
  else { 
    int avt = (lt + rt) / 2; int avd = (ld + rd) / 2; 
    int avl = (lt + ld) / 2; int avr = (rt + rd) / 2; 
    if (abs(avt - avd) > tol) { if (avt > avd) targetVert++; else targetVert--; }
    if (abs(avl - avr) > tol) { if (avl > avr) targetHor--; else targetHor++; }
    currentMode = "TRACKING";
  }

  targetVert = constrain(targetVert, 0, 90); 
  targetHor = constrain(targetHor, 0, 180);

  // Плавность движения серво
  if (millis() - lastServoTime > SERVO_SPEED_MS) {
    if (currentVert < targetVert) currentVert++;
    else if (currentVert > targetVert) currentVert--;
    
    if (currentHor < targetHor) currentHor++;
    else if (currentHor > targetHor) currentHor--;

    servoVert.write(currentVert);
    servoHor.write(currentHor);
    
    lastServoTime = millis();
  }
  // ----------------------------------------------------

  if (millis() - lastPrintTime > 300) {
    Serial.print(F("L:")); Serial.print(avgLight);
    Serial.print(F(" | T:")); Serial.print(currentTemp, 1);
    Serial.print(F(" | M:")); Serial.println(currentMode);
    lastPrintTime = millis();
  }

  if (isCritical && currentMode != lastCriticalMode) {
    saveToEEPROM(String(time.Hours) + ":" + String(time.minutes) + " " + currentMode);
    lastCriticalMode = currentMode;
  } 
  else if (!isCritical) lastCriticalMode = "";
  
  delay(10); // Уменьшил общий delay, чтобы не мешал таймерам
}

void saveToEEPROM(String msg) {
  for (unsigned int i = 0; i < msg.length(); i++) {
    EEPROM.update(eeAddress, msg[i]);
    eeAddress++;
    if (eeAddress >= 1000) eeAddress = 0;
  }
  EEPROM.update(eeAddress, '\n');
  eeAddress++;
  if (eeAddress >= 1000) eeAddress = 0;
  EEPROM.put(1020, eeAddress); 
}

void dumpEEPROM() {
  for (int i = 0; i < 1000; i++) {
    char c = EEPROM.read(i);
    if (c != 255) Serial.print(c);
  }
}

void calculateSolarPosition(int hr, int mn, int dy, int mo, float lat, float lon, float* az, float* el) {
  float utcHour = hr - timezone + (mn / 60.0);
  int N = dy + (31 * (mo - 1));
  float declination = 23.45 * sin((360.0 / 365.0) * (N - 81) * DEG_TO_RAD);
  float b = (360.0 / 364.0) * (N - 81);
  float eq = 9.87 * sin(2 * b * DEG_TO_RAD) - 7.53 * cos(b * DEG_TO_RAD) - 1.5 * sin(b * DEG_TO_RAD);
  float solarTime = utcHour + (4.0 * lon / 60.0) + (eq / 60.0);
  float hourAngle = 15.0 * (solarTime - 12.0);
  float latRad = lat * DEG_TO_RAD;
  float decRad = declination * DEG_TO_RAD;
  float hrRad = hourAngle * DEG_TO_RAD;
  float sinEl = sin(latRad) * sin(decRad) + cos(latRad) * cos(decRad) * cos(hrRad);
  *el = asin(constrain(sinEl, -1.0, 1.0)) * RAD_TO_DEG;
  float cosAz = (sin(decRad) - sin(latRad) * sin(sinEl)) / (cos(latRad) * cos(asin(constrain(sinEl, -1.0, 1.0))));
  float azRaw = acos(constrain(cosAz, -1.0, 1.0)) * RAD_TO_DEG;
  if (hourAngle > 0) *az = 360.0 - azRaw; else *az = azRaw;
}