
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <RTClib.h>


RTC_DS1307 rtc; 
const float latitude = 55.7558;
const float longitude = 37.6173;
const int timezone = 3;
const int darkThreshold = 100; // Порог перехода на RTC, проверить значения фоторезисторов


#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


Servo servoVert; 
Servo servoHor;  

int angleVert = 90; 
int angleHor = 90;
int tol = 15; 


#define WIND_PIN A6 

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  if (!rtc.begin()) Serial.println("RTC не найден");

  servoVert.attach(9);
  servoHor.attach(10);
  
  servoVert.write(angleVert);
  servoHor.write(angleHor);
  
  pinMode(WIND_PIN, INPUT);
  sensors.begin(); 
 
}

void loop() {
  
  static unsigned long lastTempTime = 0;
  if (millis() - lastTempTime > 2000) {
    sensors.requestTemperatures();
    float currentTemp = sensors.getTempCByIndex(0);
    Serial.print("Температура: "); Serial.print(currentTemp); Serial.println(" C");
    lastTempTime = millis();
  }

  
  int windSpeed = analogRead(WIND_PIN);
  if (windSpeed >= 1000) {
    angleVert = 180; 
    servoVert.write(angleVert);
    return; 
  }

  
  int a0 = analogRead(A0); 
  int a1 = analogRead(A1); 
  int a2 = analogRead(A2); 
  int a3 = analogRead(A3); 

  int maxLight = max(max(a0, a1), max(a2, a3));

  
  if (maxLight < darkThreshold) {
    // РЕЖИМ ПО КООРДИНАТАМ И ЧАСАМ
    DateTime now = rtc.now();
    float azimuth, elevation;
    calculateSolarPosition(now, latitude, longitude, &azimuth, &elevation);

    if (elevation > 0) {
      angleHor = map(constrain(azimuth, 70, 290), 70, 290, 0, 180);
      angleVert = constrain(elevation, 10, 80); 
    } else {
      angleHor = 0;   
      angleVert = 15; 
    }
  } 
  else {
    
    if (a2 > a1 + tol && a3 > a0 + tol) {
      if (abs(a1 - a2) > tol || abs(a0 - a3) > tol) {
        angleHor++; 
      } else {
        if (a2 > a3 + tol) angleVert++; 
        else if (a3 > a2 + tol) angleVert--; 
      }
    }
    
    else if (a0 > a3 + tol && a1 > a2 + tol) {
      if (abs(a1 - a2) > tol || abs(a0 - a3) > tol) {
        angleHor--; 
      } else {
        if (a1 > a0 + tol) angleVert++; 
        else if (a0 > a1 + tol) angleVert--; 
      }
    }
    
    else if (a2 > a3 + tol && a1 > a0 + tol) {
      if (abs(a1 - a0) > tol || abs(a2 - a3) > tol) {
        angleVert++; 
      } else {
        if (a1 > a2 + tol) angleHor--; 
        else if (a2 > a1 + tol) angleHor++; 
      }
    }
    
    else if (a0 > a1 + tol && a3 > a2 + tol) {
      if (abs(a1 - a0) > tol || abs(a2 - a3) > tol) {
        angleVert--; 
      } else {
        if (a0 > a3 + tol) angleHor--; 
        else if (a3 > a0 + tol) angleHor++; 
      }
    }
  }

  // ПРИМЕНЕНИЕ ОГРАНИЧЕНИЙ И ЗАПИСЬ
  angleVert = constrain(angleVert, 0, 180); //угол
  angleHor = constrain(angleHor, 0, 180);

  servoVert.write(angleVert);
  servoHor.write(angleHor);

  delay(30); 
}

// расчёт солнца
void calculateSolarPosition(DateTime dt, float lat, float lon, float* az, float* el) {
  float utcHour = dt.hour() - timezone + (dt.minute() / 60.0);
  int N = dt.day() + (31 * (dt.month() - 1)); 
  float declination = 23.45 * sin((360.0 / 365.0) * (N - 81) * DEG_TO_RAD);
  float b = (360.0 / 364.0) * (N - 81);
  float equationOfTime = 9.87 * sin(2 * b * DEG_TO_RAD) - 7.53 * cos(b * DEG_TO_RAD) - 1.5 * sin(b * DEG_TO_RAD);
  float solarTime = utcHour + (4.0 * lon / 60.0) + (equationOfTime / 60.0);
  float hourAngle = 15.0 * (solarTime - 12.0);
  float latRad = lat * DEG_TO_RAD;
  float decRad = declination * DEG_TO_RAD;
  float hrAngleRad = hourAngle * DEG_TO_RAD;
  float sinElevation = sin(latRad) * sin(decRad) + cos(latRad) * cos(decRad) * cos(hrAngleRad);
  *el = asin(sinElevation) * RAD_TO_DEG;
  float cosAzimuth = (sin(decRad) - sin(latRad) * sin(sinElevation)) / (cos(latRad) * cos(asin(sinElevation)));
  float azRaw = acos(constrain(cosAzimuth, -1.0, 1.0)) * RAD_TO_DEG;
  if (hourAngle > 0) *az = 360.0 - azRaw;
  else *az = azRaw;
}
