// ============================================================
// solar_tracker_v_test.ino
// Тестовая прошивка — проверяем каждый модуль по отдельности
//
// Команды через Serial Monitor (9600 бод):
// 1  — тест фоторезисторов (живые данные)
// 2  — тест серво (интерактивное управление)
// 3  — тест RTC (вывод времени)
// 4  — тест DS18B20 (температура)
// 5  — тест ветра (аналоговый датчик)
// 6  — тест расчёта солнца (азимут/элевация по времени)
// 7  — тест режима SEARCH (имитация сканирования)
// 0  — стоп / выход в меню
// ============================================================

#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Ds1302.h>

// Пины
#define SERVO_VERT_PIN  8
#define SERVO_HOR_PIN   6
#define ONE_WIRE_BUS    9
#define WIND_PIN        A4
// TL=A1, BL=A0, TR=A3, BR=A5

// Ограничения
const int VERT_MIN = 0,  VERT_MAX = 90;
const int HOR_MIN  = 0,  HOR_MAX  = 180;
const int SAMPLES  = 8;
const int TIMEZONE = 3;
const float LATITUDE  = 55.75;
const float LONGITUDE = 37.61;

// Объекты
// Ds1302(RST=3, CLK=5, DAT=4)
Ds1302 rtc(3, 5, 4);
Ds1302::DateTime rtcDt;
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature ds(&oneWire);
Servo servoVert, servoHor;

// Состояние
int  currentVert = 0;
int  currentHor  = 90;
char activeTest  = '0';

// SEARCH-имитация
int  searchHor       = HOR_MIN;
int  searchDir       = 1;
bool searchFoundLight = false;
unsigned long searchStableStart = 0;
unsigned long searchStepTimer   = 0;
const int  SEARCH_LIGHT_THR = 700;
const long SEARCH_STABLE_MS = 5000;
const int  SEARCH_STEP_MS   = 300;


int readAvg(uint8_t pin) {
  long s = 0;
  for (int i = 0; i < SAMPLES; i++) s += analogRead(pin);
  return (int)(s / SAMPLES);
}

int dayOfYear(int dy, int mo) {
  const int dim[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
  int n = dy;
  for (int i = 1; i < mo; i++) n += dim[i];
  return n;
}

void calculateSolarPosition(int hr, int mn, int dy, int mo,
                             float lat, float lon,
                             float* az, float* el) {
  float utcHour = hr - TIMEZONE + (mn / 60.0);
  if (utcHour < 0) utcHour += 24.0;
  int   N    = dayOfYear(dy, mo);
  float decl = 23.45 * sin((360.0/365.0)*(N-81)*DEG_TO_RAD);
  float b    = (360.0/364.0)*(N-81);
  float eq   = 9.87*sin(2*b*DEG_TO_RAD) - 7.53*cos(b*DEG_TO_RAD) - 1.5*sin(b*DEG_TO_RAD);
  float solarTime = utcHour + (4.0*lon/60.0) + (eq/60.0);
  float hourAngle = 15.0*(solarTime - 12.0);
  float latR = lat*DEG_TO_RAD, decR = decl*DEG_TO_RAD, hrR = hourAngle*DEG_TO_RAD;
  float sinEl = constrain(sin(latR)*sin(decR)+cos(latR)*cos(decR)*cos(hrR), -1.0f, 1.0f);
  *el = asin(sinEl)*RAD_TO_DEG;
  float cosEl = cos(asin(sinEl));
  float cosAz = (sin(decR)-sin(latR)*sinEl)/(cos(latR)*cosEl);
  float azRaw = acos(constrain(cosAz,-1.0f,1.0f))*RAD_TO_DEG;
  *az = (hourAngle > 0) ? (360.0f-azRaw) : azRaw;
}

void moveServo(int v, int h) {
  v = constrain(v, VERT_MIN, VERT_MAX);
  h = constrain(h, HOR_MIN,  HOR_MAX);
  if (v != currentVert) { currentVert = v; servoVert.write(v); }
  if (h != currentHor)  { currentHor  = h; servoHor.write(h);  }
}

void printMenu() {
  Serial.println();
  Serial.println(F("══════════════════════════════════"));
  Serial.println(F("  SOLAR TRACKER — ТЕСТОВЫЙ РЕЖИМ  "));
  Serial.println(F("══════════════════════════════════"));
  Serial.println(F("  1  Фоторезисторы (живые данные)"));
  Serial.println(F("  2  Серво (управление вручную)"));
  Serial.println(F("  3  RTC (время)"));
  Serial.println(F("  4  Температура DS18B20"));
  Serial.println(F("  5  Датчик ветра"));
  Serial.println(F("  6  Расчёт позиции солнца"));
  Serial.println(F("  7  Имитация режима SEARCH"));
  Serial.println(F("  0  Стоп / вернуться в меню"));
  Serial.println(F("══════════════════════════════════"));
  Serial.print(F("Выбор: "));
}


void setup() {
  Serial.begin(9600);
  servoVert.attach(SERVO_VERT_PIN);
  servoHor.attach(SERVO_HOR_PIN);
  moveServo(0, 90);
  ds.begin();

  // RTC init
  rtc.init();
  // !! УСТАНОВИТЬ СВОЁ ВРЕМЯ — раскомментируй, загрузи, закомментируй снова !!
  // Ds1302::DateTime setDt = {25, 3, 13, 4, 14, 15, 0};
  // year mo day dow hr  min sec  (dow: 1=пн..7=вс)
  // rtc.setDateTime(&setDt);

  printMenu();
}


void loop() {
  // Чтение команды
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read(); // сброс буфера

    if (cmd == '0') {
      activeTest = '0';
      Serial.println(F("\n[СТОП]"));
      printMenu();
    } else if (cmd >= '1' && cmd <= '7') {
      activeTest = cmd;
      Serial.println(cmd);
      Serial.println();

      // Инициализация при входе в тест
      if (cmd == '2') {
        Serial.println(F("── Тест СЕРВО ───────────────────────────────"));
        Serial.println(F("  V+/V-  : вертикаль +10° / -10°"));
        Serial.println(F("  H+/H-  : горизонталь +10° / -10°"));
        Serial.println(F("  VC/HC  : центр вертикаль(45) / горизонталь(90)"));
        Serial.println(F("  VN/HN  : минимум вертикаль(0) / горизонталь(0)"));
        Serial.println(F("  VX/HX  : максимум вертикаль(90) / горизонталь(180)"));
        Serial.println(F("  Vxx    : задать угол вертикали  (пример: V45)"));
        Serial.println(F("  Hxxx   : задать угол горизонтали (пример: H120)"));
        Serial.println(F("  0      : выход"));
        Serial.print(F("  Текущее положение: V="));
        Serial.print(currentVert); Serial.print(F("° H=")); Serial.print(currentHor); Serial.println(F("°"));
      } else if (cmd == '7') {
        Serial.println(F("── Тест SEARCH ──────────────────────────────"));
        Serial.println(F("  Сканирование HOR_MIN→HOR_MAX, VERT=30°"));
        Serial.println(F("  Стабильный сигнал 5с → сообщение «НАЙДЕНО»"));
        Serial.println(F("  0 — выход"));
        searchHor        = HOR_MIN;
        searchDir        = 1;
        searchFoundLight = false;
        searchStepTimer  = millis();
        moveServo(30, searchHor);
      }
    } else if (activeTest == '2') {
      // Серво: принимаем многосимвольные команды через флаг
      // (обрабатываем ниже отдельно — здесь уже в буфере)
    }
  }

  // Выполнение активного теста
  static unsigned long lastPrint = 0;

  switch (activeTest) {

    // 1: ФОТОРЕЗИСТОРЫ
    case '1': {
      if (millis() - lastPrint < 500) break;
      lastPrint = millis();
      int tl = readAvg(A1);
      int tr = readAvg(A3);
      int bl = readAvg(A0);
      int br = readAvg(A5);
      int avg = (tl+tr+bl+br)/4;
      Serial.print(F("TL:")); Serial.print(tl);
      Serial.print(F("  TR:")); Serial.print(tr);
      Serial.print(F("  BL:")); Serial.print(bl);
      Serial.print(F("  BR:")); Serial.print(br);
      Serial.print(F("  AVG:")); Serial.print(avg);
      // Разбаланс: показывает насколько датчики отличаются друг от друга
      int diffV = ((tl+tr)/2) - ((bl+br)/2);
      int diffH = ((tl+bl)/2) - ((tr+br)/2);
      Serial.print(F("  diffV:")); Serial.print(diffV);
      Serial.print(F("  diffH:")); Serial.println(diffH);
      break;
    }

    // 2: СЕРВО
    case '2': {
      // Ждём команду вида "V+", "H-", "V45", "H120", "VC", "HC" и т.д.
      if (!Serial.available()) break;
      String s = Serial.readStringUntil('\n');
      s.trim(); s.toUpperCase();
      if (s == "0") { activeTest = '0'; Serial.println(F("[СТОП]")); printMenu(); break; }

      int newV = currentVert, newH = currentHor;
      bool ok = true;

      if      (s == "V+") newV += 10;
      else if (s == "V-") newV -= 10;
      else if (s == "H+") newH += 10;
      else if (s == "H-") newH -= 10;
      else if (s == "VC") newV = 45;
      else if (s == "HC") newH = 90;
      else if (s == "VN") newV = VERT_MIN;
      else if (s == "HN") newH = HOR_MIN;
      else if (s == "VX") newV = VERT_MAX;
      else if (s == "HX") newH = HOR_MAX;
      else if (s.length() >= 2 && s[0] == 'V') newV = s.substring(1).toInt();
      else if (s.length() >= 2 && s[0] == 'H') newH = s.substring(1).toInt();
      else { Serial.println(F("  Неизвестная команда")); ok = false; }

      if (ok) {
        moveServo(newV, newH);
        Serial.print(F("  → V=")); Serial.print(currentVert);
        Serial.print(F("°  H=")); Serial.print(currentHor); Serial.println(F("°"));
      }
      Serial.print(F("Команда: "));
      break;
    }

    // 3: RTC
    case '3': {
      if (millis() - lastPrint < 1000) break;
      lastPrint = millis();
      rtc.getDateTime(&rtcDt);
      Serial.print(F("Дата: "));
      Serial.print(rtcDt.day); Serial.print(F("."));
      Serial.print(rtcDt.month); Serial.print(F("."));
      Serial.print(rtcDt.year); Serial.print(F("  Время: "));
      Serial.print(rtcDt.hour); Serial.print(F(":"));
      if (rtcDt.minute < 10) Serial.print(F("0"));
      Serial.print(rtcDt.minute); Serial.print(F(":"));
      if (rtcDt.second < 10) Serial.print(F("0"));
      Serial.println(rtcDt.second);
      break;
    }

    // 4: ТЕМПЕРАТУРА
    case '4': {
      if (millis() - lastPrint < 2000) break;
      lastPrint = millis();
      ds.requestTemperatures();
      float t = ds.getTempCByIndex(0);
      Serial.print(F("Температура: "));
      if (t == -127.0) Serial.println(F("ДАТЧИК НЕ НАЙДЕН"));
      else { Serial.print(t, 1); Serial.println(F(" °C")); }
      break;
    }

    // 5: ВЕТЕР
    case '5': {
      if (millis() - lastPrint < 300) break;
      lastPrint = millis();
      int w = analogRead(WIND_PIN);
      Serial.print(F("Ветер ADC: ")); Serial.print(w);
      Serial.print(F("  │ "));
      if      (w < 400) Serial.println(F("Тихо"));
      else if (w < 600) Serial.println(F("Умеренный"));
      else if (w < 800) Serial.println(F("Сильный"));
      else              Serial.println(F("!!! ШТОРМ !!!"));
      break;
    }

    // 6: РАСЧЁТ СОЛНЦА
    case '6': {
      if (millis() - lastPrint < 2000) break;
      lastPrint = millis();
      rtc.getDateTime(&rtcDt);
      float az, el;
      calculateSolarPosition(rtcDt.hour, rtcDt.minute,
                              rtcDt.day, rtcDt.month,
                              LATITUDE, LONGITUDE, &az, &el);
      Serial.print(F("Время: "));
      Serial.print(rtcDt.hour); Serial.print(F(":"));
      if (rtcDt.minute<10) Serial.print(F("0"));
      Serial.print(rtcDt.minute);
      Serial.print(F("  Азимут: ")); Serial.print(az, 1);
      Serial.print(F("°  Элевация: ")); Serial.print(el, 1); Serial.print(F("°"));
      // Переводим в позицию серво — удобно для проверки
      if (el > 0) {
        int targH = map(constrain((int)az, 90, 270), 90, 270, HOR_MIN, HOR_MAX);
        int targV = constrain((int)el, VERT_MIN, VERT_MAX);
        Serial.print(F("  →  TargV=")); Serial.print(targV);
        Serial.print(F("°  TargH=")); Serial.print(targH); Serial.println(F("°"));
      } else {
        Serial.println(F("  → НОЧЬ (el≤0)"));
      }
      break;
    }

    // 7: ИМИТАЦИЯ SEARCH
    case '7': {
      int avgLight = (readAvg(A1)+readAvg(A3)+readAvg(A0)+readAvg(A5))/4;

      if (avgLight < SEARCH_LIGHT_THR) {
        if (!searchFoundLight) {
          searchFoundLight  = true;
          searchStableStart = millis();
          Serial.println(F("  [ПОИСК] Свет обнаружен, жду стабильности 5 с..."));
        }
        long elapsed = millis() - searchStableStart;
        // Прогресс-бар раз в секунду
        if (millis() - lastPrint > 1000) {
          lastPrint = millis();
          Serial.print(F("  Стабильность: "));
          Serial.print(elapsed/1000); Serial.print(F("с / 5с  ADC="));
          Serial.println(avgLight);
        }
        if (elapsed >= SEARCH_STABLE_MS) {
          Serial.println(F(""));
          Serial.println(F("  ✓ СОЛНЦЕ НАЙДЕНО! Переход в TRACKING."));
          Serial.print(F("  Позиция: V=")); Serial.print(currentVert);
          Serial.print(F("°  H=")); Serial.print(currentHor); Serial.println(F("°"));
          activeTest = '0';
          printMenu();
        }
      } else {
        if (searchFoundLight) {
          searchFoundLight = false;
          Serial.println(F("  [ПОИСК] Сигнал пропал — продолжаю сканирование"));
        }
        if (millis() - searchStepTimer > SEARCH_STEP_MS) {
          searchStepTimer = millis();
          searchHor += searchDir;
          searchHor  = constrain(searchHor, HOR_MIN, HOR_MAX);
          moveServo(30, searchHor);

          // Прогресс раз в 500 мс
          if (millis() - lastPrint > 500) {
            lastPrint = millis();
            Serial.print(F("  Сканирую H=")); Serial.print(searchHor);
            Serial.print(F("°  ADC=")); Serial.println(avgLight);
          }

          if (searchHor >= HOR_MAX || searchHor <= HOR_MIN) {
            searchDir = -searchDir;
            Serial.println(F("  [ПОИСК] Край — разворот"));
            // Проверяем RTC
            rtc.getDateTime(&rtcDt);
            float az, el;
            calculateSolarPosition(rtcDt.hour, rtcDt.minute,
                                   rtcDt.day, rtcDt.month,
                                   LATITUDE, LONGITUDE, &az, &el);
            Serial.print(F("  RTC: el=")); Serial.print(el,1);
            if (el <= 0) Serial.println(F("° → НОЧЬ (выход)"));
            else         Serial.println(F("° → день, ищу дальше"));
            if (el <= 0) { activeTest = '0'; printMenu(); }
          }
        }
      }
      break;
    }

    default: break;
  }
}