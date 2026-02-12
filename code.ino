#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>


#define SERVO_VERT_PIN 5  // Вертикальный серво (D5)
#define SERVO_HOR_PIN  6  // Горизонтальный серво (D6)
#define WIND_PIN       A4 // Потенциометр ветра (A4)
#define ONE_WIRE_BUS   9  // Датчик температуры (D9)


// Москва
const float latitude = 55.75;
const float longitude = 37.61;
const int timezone = 3;


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Servo servoVert;
Servo servoHor;


int angleVert = 90;
int angleHor = 90;
int tol = 10; // Чувствительность


unsigned long lastTempTime = 0;
unsigned long lastPrintTime = 0;
float currentTemp = -127.0;


void setup() {
  Serial.begin(9600);
 
  servoVert.attach(SERVO_VERT_PIN);
  servoHor.attach(SERVO_HOR_PIN);
 
  pinMode(WIND_PIN, INPUT);
  sensors.begin();
 
  // Тестовое начальное движение
  Serial.println(F("SYSTEM STARTUP..."));
  servoVert.write(45); delay(600);
  servoVert.write(90); delay(600);
}


void loop() {
  // Фоторезисторы
  int lt = analogRead(A0); int rt = analogRead(A1);
  int ld = analogRead(A2); int rd = analogRead(A3);
  int wind = analogRead(WIND_PIN);
  
  int avgLight = (lt + rt + ld + rd) / 4; // Средняя освещенность


  // Температура
  if (millis() - lastTempTime > 2000) {
    sensors.requestTemperatures();
    currentTemp = sensors.getTempCByIndex(0);
    
    if (currentTemp > 80.0 && currentTemp != -127.0) {
       angleVert = constrain(angleVert - 5, 0, 180); 
    }
    lastTempTime = millis();
  }



  if (millis() - lastPrintTime > 300) {
    Serial.print(F(" [SYSTEM STATUS] "));
    
    // Свет
    Serial.print(F("| LIGHT: ")); 
    Serial.print(avgLight);
    Serial.print(F(" lx "));


    // Температура
    Serial.print(F(" | TEMP: "));
    if (currentTemp == -127.0) Serial.print(F("ERR"));
    else {
      Serial.print(currentTemp, 1);
      Serial.print(F(" C"));
    }


    // Ветер и Режим
    Serial.print(F(" | WIND: "));
    if (wind > 800) {
      Serial.print(F("!! STORM !!"));
      Serial.print(F(" | MODE: PROTECT"));
    } else {
      Serial.print(F("Stable"));
      Serial.print(F(" | MODE: "));
      if (avgLight > 30) Serial.print(F("AUTO-TRACKING"));
      else Serial.print(F("MATH-CALC"));
    }
    
    Serial.println(); // Перенос строки
    lastPrintTime = millis();
  }


  // Защита от ветра
  if (wind >= 800) {
    servoVert.write(180); // Позиция "флюгера"
    servoHor.write(90);
    delay(40); 
    return; // Пропускаем остальной код в этом цикле
  }


  if (avgLight < 30) { 
    // Эмуляция времени: Июнь, 12:00 (для теста)
    int m = 6; int d = 21; int h = 12; int mn = 0;
    
    float az, el;
    calculateSolarPosition(h, mn, d, m, latitude, longitude, &az, &el);
    if (el > 0) {
      angleHor = map(constrain(az, 90, 270), 90, 270, 0, 180);
      angleVert = constrain(el, 0, 90);
    }
  }
  else { 
    // Слежение по датчикам
    int avt = (lt + rt) / 2; // Верх
    int avd = (ld + rd) / 2; // Низ
    int avl = (lt + ld) / 2; // Лево
    int avr = (rt + rd) / 2; // Право


    if (abs(avt - avd) > tol) {
      if (avt > avd) angleVert++; else angleVert--;
    }
    if (abs(avl - avr) > tol) {
      if (avl > avr) angleHor--; else angleHor++;
    }
  }


  // Ограничение углов и запись в серво
  angleVert = constrain(angleVert, 0, 180);
  angleHor = constrain(angleHor, 0, 180);


  servoVert.write(angleVert);
  servoHor.write(angleHor);


  delay(40);
}


// Математика
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
  *el = asin(constrain(sinElevation, -1.0, 1.0)) * RAD_TO_DEG;
  float cosAzimuth = (sin(decRad) - sin(latRad) * sin(sinElevation)) / (cos(latRad) * cos(asin(constrain(sinElevation, -1.0, 1.0))));
  float azRaw = acos(constrain(cosAzimuth, -1.0, 1.0)) * RAD_TO_DEG;
  if (hourAngle > 0) *az = 360.0 - azRaw; else *az = azRaw;
}

