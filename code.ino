#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <iarduino_RTC.h>
#include <EEPROM.h>


// Пины
#define SERVO_VERT_PIN  8
#define SERVO_HOR_PIN   6
#define ONE_WIRE_BUS    12
#define WIND_PIN        A0
// Фоторезисторы: TL=A1, BL=A3, TR=A2, BR=A4


// Направление осей
const bool INVERT_VERT = false;
const bool INVERT_HOR  = true;


// Механические ограничения
const int VERT_MIN =  0;
const int VERT_MAX = 90;
const int HOR_MIN  =  0;
const int HOR_MAX  = 180;


//Калибровка фоторезисторов
const int OFFSET_TL =   0;
const int OFFSET_TR =   0;
const int OFFSET_BL =   0;
const int OFFSET_BR = 0;


// Параметры фильтрации
const int   SAMPLES   = 8;     // замеров на одно считывание (убирает шум ~в 3 раза)
const float EMA_ALPHA = 0.15;  // 0.1 = очень плавно, 0.4 = быстрее реагирует


// Пороги и таймеры
const int  DARK_THRESHOLD = 980;   // ADC выше → считаем темно
const int  SERVO_SPEED_MS = 40;    // мс между шагами серво
const int  TRACK_INTERVAL = 250;   // мс между пересчётом цели трекинга
const long PRINT_INTERVAL = 2000;  // мс между выводом в терминал


// Параметры SEARCH
// Поиск: горизонтальное сканирование до нахождения стабильного сигнала.
const int  SEARCH_LIGHT_THR  = 700;   // ADC ниже -> свет найден
const long SEARCH_STABLE_MS  = 5000;  // мс стабильного сигнала -> переход в TRACKING
const int  SEARCH_STEP_MS    = 200;   // мс между шагами сканирования
const int  SEARCH_VERT_POS   = 30;    // фиксированный угол подъёма во время поиска (°)


// RTC / координаты
iarduino_RTC time(RTC_DS1302, 5, 4, 3);
const float LATITUDE  = 55.75;
const float LONGITUDE = 37.61;
const int   TIMEZONE  = 3;


// Объекты
OneWire          oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Servo servoVert;
Servo servoHor;


// Режимы работы
enum TrackerMode : uint8_t {
  MODE_SEARCH,     // поиск солнца горизонтальным сканированием
  MODE_TRACKING,   // активный трекинг по фоторезисторам
  MODE_WAIT_RTC,   // ждём 20 с после потери солнца перед RTC
  MODE_RTC,        // ведение по астрономическим формулам
  MODE_NIGHT,      // ночь — панель припаркована
  MODE_STORM,      // шторм — защитная горизонтальная позиция
  MODE_OVERHEAT    // перегрев — панель отворачивается
};

TrackerMode currentMode = MODE_SEARCH;


// Серво
int currentVert = 0;
int currentHor  = 90;
int targetVert  = 0;
int targetHor   = 90;


float emaTL = 512, emaTR = 512, emaBL = 512, emaBR = 512;


// Состояние MODE_SEARCH
int           searchDir        = 1;     // направление сканирования: +1 или -1
bool          searchFoundLight = false; // видим ли сейчас достаточно света
unsigned long searchStableStart = 0;    // момент, когда свет нашли
unsigned long searchStepTimer   = 0;    // таймер шага сканирования


// Состояние MODE_WAIT_RTC
unsigned long rtcDelayTimer = 0;


// Таймеры общие
unsigned long lastTempTime  = 0;
unsigned long lastPrintTime = 0;
unsigned long lastServoTime = 0;
unsigned long lastTrackTime = 0;


// Прочее
float       currentTemp       = -127.0;
int         eeAddress         = 0;
bool        lastWasCritical   = false;
TrackerMode lastSavedCritical = MODE_TRACKING; // не критичный режим = заглушка


// Прототипы
int  readAvg(uint8_t pin);
int  dayOfYear(int dy, int mo);
void saveToEEPROM(const char* msg);
void dumpEEPROM();
void calculateSolarPosition(int hr, int mn, int dy, int mo,
                             float lat, float lon,
                             float* az, float* el);
const __FlashStringHelper* modeToString(TrackerMode m);


// Многократное считывание: усредняем SAMPLES замеров — убирает случайный шум.
int readAvg(uint8_t pin) {
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) sum += analogRead(pin);
  return (int)(sum / SAMPLES);
}


// FIX: точный день года (старая формула 31*(mo-1) давала ошибку до ~10 дней)
int dayOfYear(int dy, int mo) {
  const int dim[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int n = dy;
  for (int i = 1; i < mo; i++) n += dim[i];
  return n;
}


void setup() {
  Serial.begin(9600);

  servoVert.attach(SERVO_VERT_PIN);
  servoHor.attach(SERVO_HOR_PIN);
  servoVert.write(currentVert);
  servoHor.write(currentHor);

  sensors.begin();

  EEPROM.get(1020, eeAddress);
  if (eeAddress < 0 || eeAddress > 1000) eeAddress = 0;

  // Инициализируем EMA реальными значениями
  emaTL = readAvg(A1);
  emaTR = readAvg(A2);
  emaBL = readAvg(A3);
  emaBR = readAvg(A4);

  targetVert = SEARCH_VERT_POS;
  targetHor  = HOR_MIN;

  Serial.println(F("Солнечный трекер запущен. Отправьте 'R', чтобы посмотреть EEPROM."));
}


void loop() {
  if (Serial.available() > 0) {
    if (Serial.read() == 'R') dumpEEPROM();
  }

  time.gettime();

  // Чтение и фильтрация датчиков
  int rawTL = readAvg(A1) + OFFSET_TL;
  int rawTR = readAvg(A2) + OFFSET_TR;
  int rawBL = readAvg(A3) + OFFSET_BL;
  int rawBR = readAvg(A4) + OFFSET_BR;

  emaTL = EMA_ALPHA * rawTL + (1.0 - EMA_ALPHA) * emaTL;
  emaTR = EMA_ALPHA * rawTR + (1.0 - EMA_ALPHA) * emaTR;
  emaBL = EMA_ALPHA * rawBL + (1.0 - EMA_ALPHA) * emaBL;
  emaBR = EMA_ALPHA * rawBR + (1.0 - EMA_ALPHA) * emaBR;

  int tl = (int)emaTL;
  int tr = (int)emaTR;
  int bl = (int)emaBL;
  int br = (int)emaBR;

  int wind     = analogRead(WIND_PIN);
  int avgLight = (tl + tr + bl + br) / 4;

  // Температура раз в 2 сек
  if (millis() - lastTempTime > 2000) {
    sensors.requestTemperatures();
    currentTemp  = sensors.getTempCByIndex(0);
    lastTempTime = millis();
  }

  // Критические режимы (наивысший приоритет)
  bool isCritical = false;

  if (wind >= 800) {
    // ШТОРМ: панель горизонтально и по центру горизонтали
    // FIX: теперь фиксируем обе оси, а не только вертикаль
    if (currentMode != MODE_STORM) {
      currentMode = MODE_STORM;
    }
    targetVert = 0;
    targetHor  = (HOR_MIN + HOR_MAX) / 2;
    isCritical = true;
  }
  else if (currentTemp > 80.0 && currentTemp != -127.0) {
    // ПЕРЕГРЕВ: отворачиваем панель
    if (currentMode != MODE_OVERHEAT) {
      currentMode = MODE_OVERHEAT;
    }
    targetVert = 20;
    isCritical = true;
  }
  else {
    // Выход из критического режима -> SEARCH (переориентация)
    // Панель могла сдвинуться — не знаем, где солнце -> ищем заново.
    if (currentMode == MODE_STORM || currentMode == MODE_OVERHEAT) {
      currentMode      = MODE_SEARCH;
      searchDir        = 1;
      searchFoundLight = false;
      targetVert       = SEARCH_VERT_POS;
      targetHor        = HOR_MIN;
    }

    // Конечный автомат режимов
    switch (currentMode) {

      // ПОИСК
      // Горизонтальное сканирование. Нашли стабильный свет 5 с -> TRACKING.
      // Дошли до края, RTC говорит ночь → NIGHT.
      case MODE_SEARCH: {
        targetVert = SEARCH_VERT_POS;

        if (avgLight < SEARCH_LIGHT_THR) {
          // Свет найден — запускаем таймер стабильности
          if (!searchFoundLight) {
            searchFoundLight  = true;
            searchStableStart = millis();
          }
          if (millis() - searchStableStart >= SEARCH_STABLE_MS) {
            // Стабильный сигнал 5 секунд -> переходим в TRACKING
            currentMode      = MODE_TRACKING;
            searchFoundLight = false;
          }
          // Пока стабилизируемся — не двигаемся по горизонтали
        } else {
          // Свет не найден или пропал → сбрасываем таймер, продолжаем сканирование
          searchFoundLight = false;

          if (millis() - searchStepTimer > SEARCH_STEP_MS) {
            searchStepTimer = millis();
            targetHor += searchDir;

            if (targetHor >= HOR_MAX || targetHor <= HOR_MIN) {
              // Дошли до края — проверяем RTC
              targetHor = constrain(targetHor, HOR_MIN, HOR_MAX);
              float az, el;
              calculateSolarPosition(time.Hours, time.minutes,
                                     time.day, time.month,
                                     LATITUDE, LONGITUDE, &az, &el);
              if (el <= 0) {
                // RTC говорит ночь -> паркуемся
                currentMode = MODE_NIGHT;
                targetVert  = 0;
                targetHor   = (HOR_MIN + HOR_MAX) / 2;
              } else {
                // День, но солнце не найдено (облако?) -> разворачиваемся
                searchDir = -searchDir;
              }
            }
          }
        }
        break;
      }

      // ТРЕКИНГ
      // Основной режим. Потеряли солнце -> WAIT_RTC.
      case MODE_TRACKING: {
        if (avgLight > DARK_THRESHOLD) {
          // Потеряли солнце — ждём перед переходом на RTC
          currentMode   = MODE_WAIT_RTC;
          rtcDelayTimer = millis();
          break;
        }

        if (millis() - lastTrackTime > TRACK_INTERVAL) {
          lastTrackTime = millis();

          int avt = (tl + tr) / 2;   // среднее верхних
          int avb = (bl + br) / 2;   // среднее нижних
          int avl = (tl + bl) / 2;   // среднее левых
          int avr = (tr + br) / 2;   // среднее правых

          int diffV = avt - avb;
          int diffH = avl - avr;

          // Динамический порог:
          // ярко (ADC мало) → tol мал → реагируем на малейшую разность
          // тускло (ADC велико) → tol велик → игнорируем шум
          int tol = map(constrain(avgLight, 50, 800), 50, 800, 5, 30);

          if (abs(diffV) > tol) {
            int step = constrain(abs(diffV) / 60, 1, 3);
            if (avt < avb) targetVert += INVERT_VERT ? -step :  step;
            else           targetVert += INVERT_VERT ?  step : -step;
          }
          if (abs(diffH) > tol) {
            int step = constrain(abs(diffH) / 60, 1, 3);
            if (avl < avr) targetHor += INVERT_HOR ?  step : -step;
            else           targetHor += INVERT_HOR ? -step :  step;
          }
        }
        break;
      }

      // ОЖИДАНИЕ RTC
      // 20 секунд после потери солнца: облако или закат?
      case MODE_WAIT_RTC: {
        if (avgLight <= DARK_THRESHOLD) {
          // Солнце вернулось раньше таймаута -> обратно в TRACKING
          currentMode = MODE_TRACKING;
          break;
        }
        if (millis() - rtcDelayTimer > 20000) {
          float az, el;
          calculateSolarPosition(time.Hours, time.minutes,
                                 time.day, time.month,
                                 LATITUDE, LONGITUDE, &az, &el);
          currentMode = (el > 0) ? MODE_RTC : MODE_NIGHT;
          if (currentMode == MODE_NIGHT) {
            targetVert = 0;
            targetHor  = (HOR_MIN + HOR_MAX) / 2;
          }
        }
        break;
      }

      // RTC
      // Ведение по формулам. Солнце снова видно -> TRACKING. Закат → NIGHT.
      case MODE_RTC: {
        if (avgLight <= DARK_THRESHOLD) {
          // Облако рассеялось — возвращаемся в TRACKING без поиска
          currentMode = MODE_TRACKING;
          break;
        }
        float az, el;
        calculateSolarPosition(time.Hours, time.minutes,
                               time.day, time.month,
                               LATITUDE, LONGITUDE, &az, &el);
        if (el <= 0) {
          currentMode = MODE_NIGHT;
          targetVert  = 0;
          targetHor   = (HOR_MIN + HOR_MAX) / 2;
        } else {
          targetHor  = map(constrain((int)az, 90, 270), 90, 270, HOR_MIN, HOR_MAX);
          targetVert = constrain((int)el, VERT_MIN, VERT_MAX);
        }
        break;
      }

      // НОЧЬ
      // Ждём рассвета по RTC -> SEARCH.
      case MODE_NIGHT: {
        float az, el;
        calculateSolarPosition(time.Hours, time.minutes,
                               time.day, time.month,
                               LATITUDE, LONGITUDE, &az, &el);
        if (el > 0) {
          // Рассвет — начинаем поиск с нуля
          currentMode      = MODE_SEARCH;
          searchDir        = 1;
          searchFoundLight = false;
          targetVert       = SEARCH_VERT_POS;
          targetHor        = HOR_MIN;
        }
        break;
      }

      default: break;
    }
  }

  // Ограничения
  targetVert = constrain(targetVert, VERT_MIN, VERT_MAX);
  targetHor  = constrain(targetHor,  HOR_MIN,  HOR_MAX);

  //  Плавное движение серво
  if (millis() - lastServoTime > SERVO_SPEED_MS) {
    int newVert = currentVert;
    int newHor  = currentHor;

    if      (currentVert < targetVert) newVert++;
    else if (currentVert > targetVert) newVert--;
    if      (currentHor  < targetHor)  newHor++;
    else if (currentHor  > targetHor)  newHor--;

    if (newVert != currentVert) { currentVert = newVert; servoVert.write(currentVert); }
    if (newHor  != currentHor)  { currentHor  = newHor;  servoHor.write(currentHor);  }

    lastServoTime = millis();
  }

  // Отладка
  if (millis() - lastPrintTime > PRINT_INTERVAL) {
    Serial.print(F("Mode:")); Serial.print(modeToString(currentMode));
    Serial.print(F(" | TL:")); Serial.print(tl);
    Serial.print(F(" TR:"));   Serial.print(tr);
    Serial.print(F(" BL:"));   Serial.print(bl);
    Serial.print(F(" BR:"));   Serial.print(br);
    Serial.print(F(" | V:"));  Serial.print(currentVert);
    Serial.print(F("->"));     Serial.print(targetVert);
    Serial.print(F(" H:"));    Serial.print(currentHor);
    Serial.print(F("->"));     Serial.println(targetHor);

    if (currentMode == MODE_RTC   || currentMode == MODE_WAIT_RTC ||
        currentMode == MODE_NIGHT || currentMode == MODE_SEARCH) {
      float az, el;
      calculateSolarPosition(time.Hours, time.minutes,
                             time.day, time.month,
                             LATITUDE, LONGITUDE, &az, &el);
      Serial.print(F("  Time: "));
      Serial.print(time.day);   Serial.print(F("."));
      Serial.print(time.month); Serial.print(F(" "));
      Serial.print(time.Hours); Serial.print(F(":"));
      if (time.minutes < 10) Serial.print(F("0"));
      Serial.print(time.minutes);
      Serial.print(F(" | Az:")); Serial.print(az, 1);
      Serial.print(F(" El:"));   Serial.println(el, 1);
    }

    lastPrintTime = millis();
  }

  // EEPROM лог
  if (isCritical) {
    if (!lastWasCritical || currentMode != lastSavedCritical) {
      char buf[24];
      snprintf(buf, sizeof(buf), "%02d:%02d %s",
               time.Hours, time.minutes,
               (currentMode == MODE_STORM) ? "STORM" : "OVERHEAT");
      saveToEEPROM(buf);
      lastSavedCritical = currentMode;
    }
    lastWasCritical = true;
  } else {
    lastWasCritical = false;
  }
}


void saveToEEPROM(const char* msg) {
  for (int i = 0; msg[i] != '\0'; i++) {
    EEPROM.update(eeAddress, msg[i]);
    if (++eeAddress >= 1000) eeAddress = 0;
  }
  EEPROM.update(eeAddress, '\n');
  if (++eeAddress >= 1000) eeAddress = 0;
  EEPROM.put(1020, eeAddress);
}

void dumpEEPROM() {
  Serial.println(F("=== EEPROM LOG ==="));
  for (int i = 0; i < 1000; i++) {
    char c = EEPROM.read(i);
    if (c != 255) Serial.print(c);
  }
  Serial.println(F("=== END ==="));
}


const __FlashStringHelper* modeToString(TrackerMode m) {
  switch (m) {
    case MODE_SEARCH:   return F("SEARCH");
    case MODE_TRACKING: return F("TRACKING");
    case MODE_WAIT_RTC: return F("WAIT_RTC");
    case MODE_RTC:      return F("RTC");
    case MODE_NIGHT:    return F("NIGHT");
    case MODE_STORM:    return F("STORM");
    case MODE_OVERHEAT: return F("OVERHEAT");
    default:            return F("UNKNOWN");
  }
}

// Расчёт позиции солнца
void calculateSolarPosition(int hr, int mn, int dy, int mo,
                             float lat, float lon,
                             float* az, float* el) {
  float utcHour = hr - TIMEZONE + (mn / 60.0);
  if (utcHour < 0) utcHour += 24.0;

  int   N    = dayOfYear(dy, mo);
  float decl = 23.45 * sin((360.0 / 365.0) * (N - 81) * DEG_TO_RAD);
  float b    = (360.0 / 364.0) * (N - 81);
  float eq   = 9.87 * sin(2 * b * DEG_TO_RAD)
             - 7.53 * cos(b * DEG_TO_RAD)
             - 1.5  * sin(b * DEG_TO_RAD);

  float solarTime = utcHour + (4.0 * lon / 60.0) + (eq / 60.0);
  float hourAngle = 15.0 * (solarTime - 12.0);

  float latR = lat * DEG_TO_RAD;
  float decR = decl * DEG_TO_RAD;
  float hrR  = hourAngle * DEG_TO_RAD;

  float sinEl = sin(latR) * sin(decR) + cos(latR) * cos(decR) * cos(hrR);
  sinEl = constrain(sinEl, -1.0f, 1.0f);
  *el = asin(sinEl) * RAD_TO_DEG;

  float cosEl = cos(asin(sinEl));
  float cosAz = (sin(decR) - sin(latR) * sinEl) / (cos(latR) * cosEl);
  float azRaw = acos(constrain(cosAz, -1.0f, 1.0f)) * RAD_TO_DEG;
  *az = (hourAngle > 0) ? (360.0f - azRaw) : azRaw;
}
