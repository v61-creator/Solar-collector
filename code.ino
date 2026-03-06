#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <RTClib.h>
#include <EEPROM.h>

#define SERVO_VERT_PIN 5  
#define SERVO_HOR_PIN  6  
#define ONE_WIRE_BUS   9  
#define WIND_PIN       A6

const float latitude = 55.75;
const float longitude = 37.61;
const int timezone = 3;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Servo servoVert;
Servo servoHor;
RTC_DS3231 rtc; 

int currentVert = 90; 
int currentHor = 90;
int targetVert = 90;
int targetHor = 90;
int tol = 10; 

unsigned long lastTempTime = 0;
unsigned long lastPrintTime = 0;
float currentTemp = -127.0;


int eeAddress = 0; 
String lastCriticalMode = ""; // Чтобы не спамить одно и то же событие

void setup() {
  Serial.begin(9600);
  
  servoVert.attach(SERVO_VERT_PIN);
  servoHor.attach(SERVO_HOR_PIN);
  
  servoVert.write(currentVert);
  servoHor.write(currentHor);
  
  pinMode(WIND_PIN, INPUT);
  sensors.begin();
  
  if (!rtc.begin()) {
    Serial.println(F("RTC ERROR! Check I2C wiring (A4, A5)."));
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }

  
  EEPROM.get(1020, eeAddress);
  if (eeAddress < 0 || eeAddress > 1000) eeAddress = 0;

  Serial.println(F("SYSTEM STARTUP..."));
  Serial.println(F("Send 'R' in Serial Monitor to read EEPROM logs."));
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      dumpEEPROM();
    }
  }

  // Чтение датчиков
  int lt = analogRead(A0); int rt = analogRead(A1);
  int ld = analogRead(A2); int rd = analogRead(A3);
  int wind = analogRead(WIND_PIN);
  int avgLight = (lt + rt + ld + rd) / 4; 

  // Температура
  if (millis() - lastTempTime > 2000) {
    sensors.requestTemperatures();
    currentTemp = sensors.getTempCByIndex(0);
    
    if (currentTemp > 80.0 && currentTemp != -127.0) {
       targetVert = constrain(targetVert - 5, 0, 90); 
    }
    lastTempTime = millis();
  }

  String currentMode = "";
  bool isCritical = false; // Флаг важного события
  
  // 1. Защита от ветра 
  if (wind >= 800) {
    targetVert = 90; 
    currentMode = "STORM";
    isCritical = true;
  }
  else if (currentTemp > 80.0 && currentTemp != -127.0) {
    currentMode = "OVERHEAT";
    isCritical = true;
  }
  // 2. Работа по часам
  else if (avgLight < 30) { 
    DateTime now = rtc.now(); 
    float az, el;
    calculateSolarPosition(now.hour(), now.minute(), now.day(), now.month(), latitude, longitude, &az, &el);
    
    if (el > 0) {
      targetHor = map(constrain(az, 90, 270), 90, 270, 0, 180);
      targetVert = constrain(el, 0, 90);
    }
    currentMode = "RTC_MATH";
  }
  // 3. Авто-трекинг по датчикам
  else { 
    int avt = (lt + rt) / 2; int avd = (ld + rd) / 2; 
    int avl = (lt + ld) / 2; int avr = (rt + rd) / 2; 

    if (abs(avt - avd) > tol) {
      if (avt > avd) targetVert++; else targetVert--;
    }
    if (abs(avl - avr) > tol) {
      if (avl > avr) targetHor--; else targetHor++;
    }
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

  // Вывод в терминал (каждые 300 мс)
  if (millis() - lastPrintTime > 300) {
    Serial.print(F("[LIVE] L:")); Serial.print(avgLight);
    Serial.print(F(" | T:")); Serial.print(currentTemp, 1);
    Serial.print(F(" | W:")); Serial.print(wind);
    Serial.print(F(" | MODE:")); Serial.println(currentMode);
    
    lastPrintTime = millis();
  }

  
  if (isCritical && currentMode != lastCriticalMode) {
    DateTime now = rtc.now();
    String logMsg = String(now.hour()) + ":" + String(now.minute()) + " " + currentMode;
    saveToEEPROM(logMsg);
    lastCriticalMode = currentMode;
  } 
  else if (!isCritical) {
    lastCriticalMode = ""; // Сбрасываем, когда всё хорошо
  }

  delay(40); 
}


void saveToEEPROM(String msg) {
  Serial.println(">>> SAVING: " + msg);
  for (unsigned int i = 0; i < msg.length(); i++) {
    EEPROM.update(eeAddress, msg[i]); // update бережет память, не пишет если символ тот же
    eeAddress++;
    if (eeAddress >= 1000) eeAddress = 0; // Кольцевой буфер
  }
  EEPROM.update(eeAddress, '\n'); // Перенос строки
  eeAddress++;
  if (eeAddress >= 1000) eeAddress = 0;
  
  // Запоминаем текущий адрес, чтобы не потерять после перезагрузки
  EEPROM.put(1020, eeAddress); 
}


void dumpEEPROM() {
  Serial.println(F("=== EEPROM LOG DUMP ==="));
  for (int i = 0; i < 1000; i++) {
    char c = EEPROM.read(i);
    if (c == 255) continue; // Пропускаем пустые ячейки
    Serial.print(c);
  }
  Serial.println(F("=== END OF DUMP ==="));
}


void calculateSolarPosition(int hr, int mn, int dy, int mo, float lat, float lon, float* az, float* el) {
  float utcHour = hr - timezone + (mn / 60.0);
  int N = dy + (31 * (mo - 1));
  float declination = 23.45 * sin((360.0 / 365.0) * (N - 81) * DEG_TO_RAD);
  float b = (360.0 / 364.0) * (N - 81);
  float equationOfTime = 9.87 * sin(2 * b * DEG_TO_RAD) - 7.53 * cos(b * DEG_TO_RAD) - 1.5 * sin(b * DEG_TO_RAD);
  float solarTime = utcHour + (4.0 * lon / 60.0) + (equationOfTime / 60.0);
  float hourAngle = 15.0 * (solarTime - 12.0);
  float latRad = lat * DEG_TO_RAD;
  float decRad = declination * DEG_TO_RAD;
  float hrAngleRad = hourAngle * DEG_TO_RAD;
  float sinElevation = sin(latRad) * sin(decRad) + cos(latRad) * cos(decRad) * cos(hrAngleRad);
  *el = asin(constrain(sinElevation