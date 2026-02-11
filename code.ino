#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <iarduino_RTC.h> // Установи библиотеку iarduino_RTC


iarduino_RTC rtc(RTC_DS1302, 3, 5, 4); 


const float latitude = 55.7558;
const float longitude = 37.6173;
const int timezone = 3;


#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


Servo servoVert; 
Servo servoHor;  
int angleVert = 90; // угол
int angleHor = 90;
int tol = 15; 


#define WIND_PIN A6 

void setup() {
  Serial.begin(9600);
  rtc.begin();
  rtc.settime(0, 30, 22, 9, 2, 26, 1); // Сек, Мин, Час, День, Месяц, Год, день недели (настроить один раз)

  servoVert.attach(9);
  servoHor.attach(10);
  pinMode(WIND_PIN, INPUT);
  sensors.begin(); 
}

void loop() {
  
  rtc.gettime(); 
  int m = rtc.month;
  int d = rtc.day;

  
  bool isWorkingSeason = false;
  if (m > 4 && m < 9) isWorkingSeason = true; // Май, Июнь, Июль, Август
  if (m == 4) isWorkingSeason = true;         // Весь Апрель
  if (m == 9 && d <= 20) isWorkingSeason = true; // Сентябрь до 20 числа

  if (!isWorkingSeason) {
    
    servoVert.write(90); // угол
    servoHor.write(90);
    return; 
  }

 
  static unsigned long lastTempTime = 0;
  if (millis() - lastTempTime > 2000) {
    sensors.requestTemperatures();
    Serial.print("Temp: "); Serial.println(sensors.getTempCByIndex(0));
    lastTempTime = millis();
  }

  
  if (analogRead(WIND_PIN) >= 1000) {
    angleVert = 180;                      //угол
    servoVert.write(angleVert);
    return; 
  }

  
  int a0 = analogRead(A0); int a1 = analogRead(A1); 
  int a2 = analogRead(A2); int a3 = analogRead(A3); 

  
  int brightest = a0;
  if (a1 < brightest) brightest = a1;
  if (a2 < brightest) brightest = a2;
  if (a3 < brightest) brightest = a3;

  
  
  if (brightest > 1000) { 
    
    float azimuth, elevation;
    calculateSolarPosition(rtc.Hours, rtc.minutes, rtc.day, rtc.month, latitude, longitude, &azimuth, &elevation);

    if (elevation > 0) {
      angleHor = map(constrain(azimuth, 70, 290), 70, 290, 0, 180);
      angleVert = constrain(elevation, 10, 85); 
    } else {
      angleHor = 0; angleVert = 15; // Ночь
    }
  } 
  else {
    
    if (a2 < a1 - tol && a3 < a0 - tol) {
      if (abs(a1 - a2) > tol || abs(a0 - a3) > tol) angleHor++; 
      else { if (a2 < a3 - tol) angleVert++; else if (a3 < a2 - tol) angleVert--; }
    }
    else if (a0 < a3 - tol && a1 < a2 - tol) {
      if (abs(a1 - a2) > tol || abs(a0 - a3) > tol) angleHor--; 
      else { if (a1 < a0 - tol) angleVert++; else if (a0 < a1 - tol) angleVert--; }
    }
    else if (a2 < a3 - tol && a1 < a0 - tol) {
      if (abs(a1 - a0) > tol || abs(a2 - a3) > tol) angleVert++; 
      else { if (a1 < a2 - tol) angleHor--; else if (a2 < a1 - tol) angleHor++; }
    }
    else if (a0 < a1 - tol && a3 < a2 - tol) {
      if (abs(a1 - a0) > tol || abs(a2 - a3) > tol) angleVert--; 
      else { if (a0 < a3 - tol) angleHor--; else if (a3 < a0 - tol) angleHor++; }
    }
  }

  
  angleVert = constrain(angleVert, 0, 180);         //угол
  angleHor = constrain(angleHor, 0, 180);
  servoVert.write(angleVert);
  servoHor.write(angleHor);

  delay(30); 
}


void calculateSolarPosition(int hr, int mn, int dy, int mo, float lat, float lon, float* az, float* el) {
  float utcHour = hr - timezone + (mn / 60.0);
  int N = dy + (31 * (mo - 1)); 
  float declination = 23.45 * sin((360.0 / 365.0) * (N - 81) * DEG_TO_RAD);
  float b = (360.0 / 364.0) * (N - 81);
  float equationOfTime = 9.87 * sin(2 * b * DEG_TO_RAD) - 7.53 * cos(b * DEG_TO_RAD) - 1.5 * sin(b * DEG_TO_RAD);
  float solarTime = utcHour + (4.0 * lon / 60.0) + (equationOfTime / 60.0);
  float hourAngle = 15.0 * (solarTime - 12.0);
  float latRad = lat * DEG_TO_RAD, decRad = declination * DEG_TO_RAD, hrAngleRad = hourAngle * DEG_TO_RAD;
  float sinElevation = sin(latRad) * sin(decRad) + cos(latRad) * cos(decRad) * cos(hrAngleRad);
  *el = asin(sinElevation) * RAD_TO_DEG;
  float cosAzimuth = (sin(decRad) - sin(latRad) * sin(sinElevation)) / (cos(latRad) * cos(asin(sinElevation)));
  float azRaw = acos(constrain(cosAzimuth, -1.0, 1.0)) * RAD_TO_DEG;
  if (hourAngle > 0) *az = 360.0 - azRaw; else *az = azRaw;
}
