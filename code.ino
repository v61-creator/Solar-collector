#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <iarduino_RTC.h>
#include <EEPROM.h>

#define SERVO_VERT_PIN 8 
#define SERVO_HOR_PIN  6  
#define ONE_WIRE_BUS   9  
#define WIND_PIN       A4

// Настройка плавности
const int SERVO_SPEED_MS = 50; 
// RST=5, CLK=4, DAT=3
iarduino_RTC time(RTC_DS1302, 5, 4, 3);

const float latitude = 55.75;
const float longitude = 37.61;
const int timezone = 3;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Servo servoVert;
Servo servoHor;

// Стартовые позиции 
int currentVert = 90; 
int currentHor = 90;
int targetVert = 90;
int targetHor = 90;

unsigned long lastTempTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastServoTime = 0;
float currentTemp = -127.0;

int eeAddress = 0; 
String lastCriticalMode = "";

// Переменные для таймера перехода на RTC (20 секунд)
unsigned long rtcDelayTimer = 0;
bool lowLightActive = false;

void setup() {
  Serial.begin(9600);
  
  // Принудительно ставим моторы в 90 при старте
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

  time.gettime(); // Обновляем переменные времени
  
  int lt = analogRead(A0); int rt = analogRead(A1);
  int ld = analogRead(A2); int rd = analogRead(A3);
  int wind = analogRead(WIND_PIN);
  int avgLight = (lt + rt + ld + rd) / 4; 

  // Датчик температуры
  if (millis() - lastTempTime > 2000) {
    sensors.requestTemperatures();
    currentTemp = sensors.getTempCByIndex(0);
    lastTempTime = millis();
  }

  String currentMode = "";
  bool isCritical = false;
  
  // 1. ШТОРМ
  if (wind >= 800) {
    targetVert = 90; 
    currentMode = "STORM";
    isCritical = true;
  }
  // 2. ПЕРЕГРЕВ
  else if (currentTemp > 80.0 && currentTemp != -127.0) {
    targetVert = 20; // Отворачиваемся от солнца
    currentMode = "OVERHEAT";
    isCritical = true;
  }
  // 3. ПРОВЕРКА ОСВЕЩЕННОСТИ (Переход на RTC через 20 сек)
  else if (avgLight < 40) { // Порог темноты
    if (!lowLightActive) {
      rtcDelayTimer = millis();
      lowLightActive = true;
    }
    
    if (millis() - rtcDelayTimer > 20000) { // Если прошло 20 секунд
      float az, el;
      calculateSolarPosition(time.hours, time.minutes, time.day, time.month, latitude, longitude, &az, &el);
      if (el > 0) {
        targetHor = map(constrain(az, 90, 270), 90, 270, 0, 180);
        targetVert = constrain(el, 0, 90);
        currentMode = "RTC_MODE";
      } else {
        targetVert = 90; // Ночь - смотрим в зенит или на восток
        currentMode = "NIGHT";
      }
    } else {
      currentMode = "WAIT_RTC"; // Статус ожидания 20 сек
    }
  }
  // 4. ТРЕКИНГ ПО ФОТОРЕЗИСТОРАМ
  else { 
    lowLightActive = false; // Сброс таймера RTC
    
    // Динамическая чувствительность: чем светлее, тем меньше шаг
    // На ярком свету (avgLight > 800) ставим tol = 2, в тени tol = 15
    int dynamicTol = map(constrain(avgLight, 40, 900), 40, 900, 15, 2); 

    int avt = (lt + rt) / 2; int avd = (ld + rd) / 2; 
    int avl = (lt + ld) / 2; int avr = (rt + rd) / 2; 
    
    if (abs(avt - avd) > dynamicTol) { 
      if (avt > avd) targetVert++; else targetVert--; 
    }
    if (abs(avl - avr) > dynamicTol) { 
      if (avl > avr) targetHor--; else targetHor++; 
    }
    currentMode = "LDR_TRACK";
  }

  // Ограничения для серво
  targetVert = constrain(targetVert, 0, 90); 
  targetHor = constrain(targetHor, 0, 180);

  // ПЛАВНОЕ ДВИЖЕНИЕ 
  if (millis() - lastServoTime > SERVO_SPEED_MS) {
    if (currentVert < targetVert) currentVert++;
    else if (currentVert > targetVert) currentVert--;
    
    if (currentHor < targetHor) currentHor++;
    else if (currentHor > targetHor) currentHor--;

    servoVert.write(currentVert);
    servoHor.write(currentHor);
    lastServoTime = millis();
  }

  // Мониторинг
  if (millis() - lastPrintTime > 500) {
    Serial.print(F("Mode:")); Serial.print(currentMode);
    Serial.print(F(" | L:")); Serial.print(avgLight);
    Serial.print(F(" | V:")); Serial.print(currentVert);
    Serial.print(F(" | H:")); Serial.println(currentHor);
    lastPrintTime = millis();
  }

  // EEPROM логирование
  if (isCritical && currentMode != lastCriticalMode) {
    saveToEEPROM(String(time.hours) + ":" + String(time.minutes) + " " + currentMode);
    lastCriticalMode = currentMode;
  } 
  else if (!isCritical) lastCriticalMode = "";
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
  if (utcHour < 0) utcHour += 24.0;
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
  float cosAz = (sin(decRad) - sin(latRad) * sinEl) / (cos(latRad) * cos(asin(constrain(sinEl, -1.0, 1.0))));
  float azRaw = acos(constrain(cosAz, -1.0, 1.0)) * RAD_TO_DEG;
  if (hourAngle > 0) *az = 360.0 - azRaw; else *az = azRaw;
}