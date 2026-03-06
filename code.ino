#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <virtuabotixRTC.h>
#include <EEPROM.h>

#define SERVO_VERT_PIN 5  
#define SERVO_HOR_PIN  6  
#define ONE_WIRE_BUS   9  
#define WIND_PIN       A6

// Пины для DS1302: CLK, DAT, RST
virtuabotixRTC myRTC(5, 4, 3); 

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
float currentTemp = -127.0;

int eeAddress = 0; 
String lastCriticalMode = "";

void setup() {
  Serial.begin(9600);
  
  servoVert.attach(SERVO_VERT_PIN);
  servoHor.attach(SERVO_HOR_PIN);
  
  servoVert.write(currentVert);
  servoHor.write(currentHor);
  
  pinMode(WIND_PIN, INPUT);
  sensors.begin();
  
  // Установка времени (раскомментировать следующую строку 1 раз, чтобы установить время)
  // myRTC.setDS1302Time(00, 30, 12, 6, 6, 3, 2026); // сек, мин, час, день_нед, день_мес, мес, год
  
  EEPROM.get(1020, eeAddress);
  if (eeAddress < 0 || eeAddress > 1000) eeAddress = 0;

  Serial.println(F("SYSTEM STARTUP (DS1302)..."));
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') dumpEEPROM();
  }

  // Обновляем время из модуля
  myRTC.updateTime();

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
    calculateSolarPosition(myRTC.hours, myRTC.minutes, myRTC.dayofmonth, myRTC.month, latitude, longitude, &az, &el);
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

  if (currentVert < targetVert) currentVert++;
  else if (currentVert > targetVert) currentVert--;
  
  if (currentHor < targetHor) currentHor++;
  else if (currentHor > targetHor) currentHor--;

  servoVert.write(currentVert);
  servoHor.write(currentHor);

  if (millis() - lastPrintTime > 300) {
    Serial.print(F("[LIVE] L:")); Serial.print(avgLight);
    Serial.print(F(" | T:")); Serial.print(currentTemp, 1);
    Serial.print(F(" | MODE:")); Serial.println(currentMode);
    lastPrintTime = millis();
  }

  if (isCritical && currentMode != lastCriticalMode) {
    String logMsg = String(myRTC.hours) + ":" + String(myRTC.minutes) + " " + currentMode;
    saveToEEPROM(logMsg);
    lastCriticalMode = currentMode;
  } 
  else if (!isCritical) lastCriticalMode = "";

  delay(40); 
}