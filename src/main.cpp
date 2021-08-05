//пины подключения настраиваем в файле LCDSoftI2C.cpp
#define I2C_PULLUP 1
#define I2C_FASTMODE 1
#define SCL_PORT PORTB
#define SDA_PORT PORTB
#define SCL_PIN 2       // Arduino Pin 10
#define SDA_PIN 3       // Arduino Pin 11
#include <SH1106Lib.h>
#include "glcdfont.h"
#define yFONT 12
//#define xFONT 7
#define rpVal 66
#define tRate 30
#define tpVal 75
SH1106Lib lcd;
#include <EEPROM.h>                           // Стандартная библиотека
#include "GyverEncoder.h"                     // Библиотеки с сайта 
#include "directTimers.h"                     // https://codeload.github.com/AlexGyver/GyverLibs/zip/master
#include "Wire.h"
#include "uo.h"
//--------------------------- Настройка железа ---------------------------------------------
// Подключение энкодера
#define SW2 8                             // Сигнал энкодера 1 // Если кручение влево/вправо перепутано,
#define SW1 7                             // Сигнал энкодера 2 // можно поменять местами 
#define KEY 6                             // Кнопка энкодера
Encoder enc1(SW1, SW2, KEY, TYPE2);

// Подключение DRV8825
#define DIR 2
#define STEP 3                                // Тактовые импульсы на драйвер ШД
#define DRV_EN 9

// Внешнее управление
#define TMAS  A0                              // Сюда подключаем управление от TMAS, низкий уровень вызовет паузу
#define PIEZO A1                              // Здесь пьезопищалка
#define LED   A2                              // Светодиод на панели
#define soundA B00000101                      // Ритм автостопа
#define soundB B01011101                     // Ритм внешнего стопа



//--------------------------------------------------------------------------------------
// Наименования режимов работы
//----------------------------------------------
#define RUNNING               0               //
#define PAUSED                1               //
#define PAUSED_BY_EXT         2               //
#define PAUSED_BY_VOL         3               //
#define SELECT_VOL            4               //
#define TUNING                10              //
#define TUNING_PAUSE          11              //
//----------------------------------------------
// Константы
//----------------------------------------------
#define maximumRate           5000            //  Максимальная скорость отбора мл/час
#define rateStep              10              //  Мелкий шаг регулировки отбора
#define rateMidStep           50              //  Средний шаг регулировки отбора
#define rateBigStep           100             //  Крупный шаг регулировки отбора
#define volStep               10              //  Шаг регулировки дозатора
#define volMidStep            50              //  Средний шаг регулировки дозатора
#define volBigStep            500             //  Крупный шаг регулировки дозатора
#define maxDrinkVol           10000            //  Максимальный объем, выдаваемый дозатором
#define defaultDrink          0               //  Объем дозатора по умолчанию. 0 - без ограничения.
#define defaultpValsFor100ml  150000          //  Количество шагов потребное для отбора 100 мл по умолчанию при шаге/8
#define minStepsFor100ml      50000          //  Минимально возможное число шагов для отбора 100 мл при шаге/8
#define maxStepsFor100ml      1500000
//----------------------------------------------
// Переменные
//----------------------------------------------
volatile unsigned long stepsFor100ml;                  //  Количество шагов потребное для отбора 100 мл
volatile unsigned int  stepsForOneMl;                  //  Количество шагов потребное для отбора 1 мл
volatile unsigned long stepsCount     = 0;             //  Счетчик количества шагов
volatile unsigned long drinkBackCounter = 0;           //  Обратный счётчик шагов дозатора
volatile unsigned long temp           = 0;             //  Временная переменная для разнообразных нужд
volatile unsigned long tempA          = 0;             //  Временная переменная для разнообразных нужд
volatile unsigned long OCR1ABase      = 0;             //  Коэффициент для расчета OCR1A
volatile unsigned int  timer1EndValue = 65535;         //  Значение при котором происходит прерывание от таймера 1 (11739)
volatile byte          T1ClockDivider = PRESCALER_8;    //
volatile byte          thousandth     = 0;             //
volatile byte          tenth          = 0;             //
volatile byte          counterA       = 0;             //
volatile byte          counterB       = 0;             //
volatile byte          pattern        = 0;             //
volatile byte          currentMode    = RUNNING;       // Текущий режим работы
volatile uint16_t      rate           = 0;             // Скорость отбора мл/час 0 - maximumRate
volatile uint16_t      totalVolume    = 0;             // Общий объем отбора мл
volatile uint16_t      drinkVolume    = 0;             // Шаг рОзлива
volatile uint16_t      remainVolume   = 0;
volatile bool          newSecond      = false;
volatile bool          newTenth       = false;
volatile bool          stepEnabled    = false;
volatile bool          tmasStop       = false;
volatile bool          drinkStop      = false;
volatile bool          newDrinkVol    = false;
volatile bool          sndFlag        = false;

//--------------------------------------------------------------------------------------
// Сообщения
#define STRING_00 "--- NORMAL  MODE ---"  //
#define STRING_01 "---    PAUSED    ---"  //  
#define STRING_02 "--- PAUSED by OP ---"  //  
#define STRING_03 "-- PAUSED by EXT  --"  //  
#define STRING_04 "-- PAUSED by AUTO --"  //  
#define STRING_05 "--- SELECT  STEP ---"  //  
#define STRING_10 "--- TUNING  MODE ---"  //   
#define STRING_11 "--- TUNING PAUSE ---"  //  
#define STRING_12 "-Counter too little-"  //  
#define STRING_13 "-Counter too large -"  //  
#define STRING_14 "-- 100 ml UPDATED --"  //  
#define STRING_15 "--  Total ZEROED  --"  //  
//--------------------------------------------------------------------------------------
//void calcOCR1A();
String formatNum(uint32_t Number, int lenth);
void tryToSaveStepsFor100ml();
void decreaseRate();
void increaseRate();
void calcTotalVolume();
void calcOCR1A();
void oneTenthSub();
void oneSecSub();
void resumeRun();
void printTuneScreen();
void printMainScreen();
void printValues();
void setMaximumRate();
void setMinimumRate();
void  tryToTune();
//--------------------------------------------------------------------------------------


void pauseRun(){
  stepEnabled = false;
  currentMode = PAUSED;
  lcd.setCursor(0, 0);
  lcd.print(STRING_01);//02
}
void requestEvent()
{
  volatile dozer_data_t dz;
  dz.rate = rate;
  dz.currentMode = currentMode;
  dz.drinkVolume = drinkVolume;
  dz.totalVolume = totalVolume;
  dz.crc = calcCRC16((uint8_t *)&dz,sizeof(dz)-sizeof(uint16_t));
  Wire.write((uint8_t *)&dz, sizeof(dz));
}
void receiveEvent(int howMany)
{
  master_cmd_t cmd;
  uint16_t crc16;
  uint8_t data[howMany];
  int x = 0;

  while (Wire.available() > 0)
  {                        // loop through all but the last
    data[x] = Wire.read(); // receive byte as a character
    x++;
  }
  if ((howMany) == sizeof(cmd))
  {
    memcpy(&cmd, &data, sizeof(cmd));
    crc16 = calcCRC16((uint8_t *)&cmd,sizeof(cmd)-sizeof(uint16_t));
    if (crc16!=cmd.crc)
    return;
    if (cmd.cmd < DOZER_SET_MAX)
    {
      switch (cmd.cmd)
      {
        case DOZER_SET_DRINK_VOLUME:
        {
          drinkVolume=cmd.val;
          drinkBackCounter = (uint32_t)(drinkVolume * stepsForOneMl - 1);
        }
        break;

        case DOZER_SET_RATE:
        {
          rate=cmd.val;
          if (rate > maximumRate)
            rate = maximumRate;
          calcOCR1A();
          resumeRun();
        }
        break;

        case DOZER_SET_TOTAL_VOLUME:
        {
          totalVolume = cmd.val;
          stepsCount = totalVolume / stepsForOneMl; //dz.totalVolume = stepsCount / stepsForOneMl;
        }
        break;

        case DOZER_SET_MODE:
        {
          if (cmd.val<DOZER_MODE_MAX)
          {
            currentMode = cmd.val;
            if (currentMode == DOZER_MODE_PAUSED) {
              lcd.setCursor(0, 0);
              lcd.print(STRING_02);
              pauseRun();
            }
            else if(currentMode == DOZER_MODE_RUNNING) resumeRun();
          }
        }
        break;
      }
    }
  }
  calcTotalVolume();
}


//--------------------------------------------------------------------------------------
void setup()
{
  // Настройка входов и выходов
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(DRV_EN, OUTPUT);
  pinMode(PIEZO, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(SW1,  INPUT_PULLUP);
  pinMode(SW2,  INPUT_PULLUP);
  pinMode(KEY,  INPUT_PULLUP);
  pinMode(TMAS, INPUT_PULLUP);
  digitalWrite(DRV_EN, LOW);
  digitalWrite(DIR, LOW);
  digitalWrite(STEP, 1);
  digitalWrite(PIEZO, 1);
  digitalWrite(LED, 1);

  // Настройка таймера 1, он задаёт частоту шагания двигателя
  TIMER1_setClock(T1ClockDivider);             // Частота тактирования таймера 1: 16/32 = 0.5 МГц при шаге/8
  TIMER1_setMode(CTC_MODE);                 // Режим работы таймера - сравнение со значением, прерывание и рестарт
  TIMER1_COMPA_setValue(timer1EndValue);    // Значение для сравнения
  TIMER1_attach_COMPA();                    // Прерывание от таймера 1 по сравнению

  // Настройка таймера 2, он отсчитывает временные интервалы для работы программы
  TIMER2_setClock(PRESCALER_128);           // Частота тактирования таймера 2: 16/128 = 0.125 МГц
  TIMER2_setMode(CTC_MODE);                 // включаем сброс таймера по совпадению
  TIMER2_COMPA_setValue(124);               // настраиваем следование прерываний с частотой 1000 гц
  TIMER2_attach_COMPA();                    // прерывание с опросом энкодера
  //
  // Настройка дисплея
    lcd.initialize();
  lcd.clearDisplay();

  lcd.setFont(font, 5, 7);
  lcd.setTextWrap(true);
  lcd.setTextColor(WHITE, SOLID);
  lcd.clear();

  Wire.begin(DOZER_I2C_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  Wire.setTimeout(500);

  rate = 0;
  totalVolume = 0;
  stepsCount = 0;
  stepEnabled = false;

  // Попытка прочитать установки из EEPROM
  EEPROM.get(0, stepsFor100ml);
  if ((stepsFor100ml < minStepsFor100ml) or (stepsFor100ml > maxStepsFor100ml)) // Если в EEPROM не установлено число шагов на 100 мл, то
  {
    stepsFor100ml = defaultpValsFor100ml;                     // Установим число шагов на 100 мл по умолчанию
  }
  //stepsForOneMl = round((float)stepsFor100ml / 100);
  stepsForOneMl = stepsFor100ml / 100;
  OCR1ABase = 360000000000LL / stepsFor100ml; //round((float)3600000000 / stepsForOneMl);
  calcOCR1A();
  drinkVolume = defaultDrink;
  drinkBackCounter = (uint32_t)drinkVolume * stepsForOneMl - 1;
  if (!digitalRead(KEY)) {
    currentMode = TUNING;
    printTuneScreen();
    enc1.isHolded();
  }
  else {
    currentMode = RUNNING;
    printMainScreen();
  }
}
//--------------------------------------------------------------------------------------
//---------------------- Начало основного цикла ----------------------------------------
void loop()
{
  if (TWAR != TWI_SA) TWAR = TWI_SA;
  if (newSecond) {
    newSecond = false;
    oneSecSub();
  }
  if (newTenth)  {
    newTenth = false;
    oneTenthSub();
  }
  //---------------------------------------------------------------
  switch (currentMode)
  {
    //-------------------------------------------------------------
    // Режим калибровки насоса основной экран ---------------------
    //-------------------------------------------------------------
    case TUNING:            // Настройка количества шагов на 100 мл.
      {
        //--------------------
        if (enc1.isRight()) // Поворот по часовой стрелке увеличивает частоту шагов
        {
          increaseRate();
          stepEnabled = true;
          lcd.setCursor(tRate, 1*yFONT);
          lcd.print(formatNum(rate, 5));
        }
        //--------------------
        if (enc1.isLeft()) // Поворот против часовой стрелки уменьшает частоту шагов
        {
          decreaseRate();
          if (rate != 0)
          {
            stepEnabled = true;
          }
          else
          {
            stepEnabled = false;
          }
          lcd.setCursor(tRate,1*yFONT);
          lcd.print(formatNum(rate, 5));
        }
        //--------------------
        if (enc1.isClick())
        {
          stepEnabled = false;
          currentMode = TUNING_PAUSE;
          lcd.setCursor(0, 0);
          lcd.print(STRING_11);
        }
        //--------------------
        if (enc1.isHolded())
        {
          lcd.clear();
          lcd.setCursor(0, 1*yFONT);
          lcd.print("-- RELEASE BUTTON --");
          while (!digitalRead(KEY));
          delay(1000);
          //lcd.clear();
          setup();
        }
        //--------------------
      }
      break;
    //-------------------------------------------------------------
    // Пауза в режиме калибровки насоса ---------------------------
    //-------------------------------------------------------------
    case TUNING_PAUSE:
      {
        //--------------------
        if (enc1.isRight()) // Поворот по часовой стрелке увеличивает частоту шагов
        {
          increaseRate();
          lcd.setCursor(tRate, 1*yFONT);
          lcd.print(formatNum(rate, 5));
        }
        //--------------------
        if (enc1.isLeft()) // Поворот против часовой стрелки уменьшает частоту шагов
        {
          decreaseRate();
          lcd.setCursor(tRate, 1*yFONT);
          lcd.print(formatNum(rate, 5));
        }
        //--------------------
        if (enc1.isClick())
        {
          if (rate != 0)
          {
            stepEnabled = true;
          }
          else
          {
            stepEnabled = false;
          }
          currentMode = TUNING;
          lcd.setCursor(0, 0);
          lcd.print(STRING_10);
        }
        //--------------------
        if (enc1.isHolded())    // Здесь сброс в 0 счетчика шагов и общего объема
        {
          stepsCount = 0;
          totalVolume = 0;
        }
        //--------------------
        if (enc1.isRightH())    // Пытаемся запомнить количество шагов на 100 мл в EEPROM
        {
          tryToSaveStepsFor100ml();
        }
        //--------------------
        if (enc1.isLeftH())    // Пытаемся запомнить количество шагов на 100 мл в EEPROM
        {
          tryToSaveStepsFor100ml();
        }
        //--------------------
      }
      break;
    //-------------------------------------------------------------
    // Основной режим. Идёт отбор. --------------------------------
    //-------------------------------------------------------------
    case RUNNING:
      {
        //--------------------
        if (enc1.isClick()) // Клик на кнопку, переход в режим паузы
        {
          pauseRun();
        }
        //--------------------
        if (tmasStop)      // Внешний сигнал, переход в режим паузы по внешнему сигналу
        {
          stepEnabled = false;
          currentMode = PAUSED_BY_EXT;
          lcd.setCursor(0, 0);
          lcd.print(STRING_03);
          pattern = soundB;
          sndFlag = true;
        }
        //--------------------
        if (drinkStop)     // Налит запрошенный объем, переход в паузу.
        {
          stepEnabled = false;
          drinkStop = false;
          currentMode = PAUSED_BY_VOL;
          drinkBackCounter = (uint32_t)drinkVolume * stepsForOneMl - 1;
          lcd.setCursor(0, 0);
          lcd.print(STRING_04);
          pattern = soundA;
          sndFlag = true;
        }
        //--------------------
        if (enc1.isRight())     // Вращение по часовой (режим RUNNING)
        {
          increaseRate();
          stepEnabled = true;
          lcd.setCursor(rpVal, 1*yFONT);
          lcd.print(formatNum(rate, 5));
        }
        //--------------------
        if (enc1.isLeft())      // Вращение против часовой (режим RUNNING)
        {
          decreaseRate();
          if (rate != 0)
          {
            stepEnabled = true;
          }
          else
          {
            stepEnabled = false;
          }
          lcd.setCursor(rpVal, 1*yFONT);
          lcd.print(formatNum(rate, 5));
        }
        //--------------------
        if (enc1.isHolded())    // Удержание кнопки, сброс общего счетчика
        {
          if ((rate == 0) and (stepsCount != 0))
          {
            stepsCount = 0;
            totalVolume = 0;
            drinkBackCounter = (uint32_t)drinkVolume * stepsForOneMl - 1;
            lcd.clear();
            lcd.setCursor(0, 1*yFONT);
            lcd.print(STRING_15);
            delay(2000);
            lcd.clear();
            printMainScreen();
            printValues();
          }
        }
        //--------------------
        if (enc1.isRightH())
        {
          if ((rate == 0) and (stepsCount == 0))
          {
            tryToTune();
          }
          else
          {
            setMaximumRate();
            stepEnabled = true;
          }
        }
        //--------------------
        if ( enc1.isLeftH())
        {
          if ((rate == 0) and (stepsCount == 0))
          {
            tryToTune();
          }
          else
          {
            setMinimumRate();
          }
        }
        //-------------------
        if (enc1.isPress())
        {
          counterA = 0;
        }
      }
      break;
    //-------------------------------------------------------------
    // Остановка по нажатию кнопки --------------------------------
    //-------------------------------------------------------------
    case PAUSED:
      {
        //--------------------
        //--------------------
        if (enc1.isClick())
        {
          resumeRun();
        }
        //--------------------
        if (enc1.isRight())
        {
          increaseRate();
          lcd.setCursor(rpVal, 1*yFONT);
          lcd.print(formatNum(rate, 5));
        }
        //--------------------
        if (enc1.isLeft())
        {
          decreaseRate();
          lcd.setCursor(rpVal, 1*yFONT);
          lcd.print(formatNum(rate, 5));
        }
        //--------------------
        if (enc1.isHolded())
        {
          currentMode = SELECT_VOL;
          lcd.setCursor(0, 0);
          lcd.print(STRING_05);
          remainVolume = drinkVolume;
        }
        //--------------------
        if (enc1.isRightH())
        {
          setMaximumRate();
        }
        //--------------------
        if (enc1.isLeftH())
        {
          setMinimumRate();
        }
      }
      break;
    //-------------------------------------------------------------
    // Остановка по внешнему сигналу ------------------------------
    //-------------------------------------------------------------
    case PAUSED_BY_EXT:
      {
        if (!tmasStop)
        {
          sndFlag = false;
          resumeRun();
        }
        //--------------------
        if (enc1.isRight() || enc1.isLeft() || enc1.isClick())
        {
          sndFlag = false;
        }
        //--------------------
      }
      break;
    //-------------------------------------------------------------
    // Остановка по достижению заданного разового объема отбора ---
    //-------------------------------------------------------------
    case PAUSED_BY_VOL:
      {
        //--------------------
        if (enc1.isHolded())
        {
          currentMode = SELECT_VOL;
          lcd.setCursor(0, 0);
          lcd.print(STRING_05);
          sndFlag = false;                  // Заткнуть пищалку
        }
        //--------------------
        if (enc1.isClick())
        {
          if (sndFlag)
          {
            sndFlag = false;                  // Заткнуть пищалку
            //currentMode = PAUSED;
          }
          else
          {
            resumeRun();
            //currentMode = PAUSED;
          }
        }
        //--------------------
        if (enc1.isRight())
        {
          if (sndFlag)
          {
            sndFlag = false;                  // Заткнуть пищалку
          }
          else
          {
            increaseRate();
            lcd.setCursor(rpVal, 1*yFONT);
            lcd.print(formatNum(rate, 5));
          }
        }
        //--------------------
        if (enc1.isLeft())
        {
          if (sndFlag)
          {
            sndFlag = false;                  // Заткнуть пищалку
          }
          else
          {
            decreaseRate();
            lcd.setCursor(rpVal, 1*yFONT);
            lcd.print(formatNum(rate, 5));
          }
        }
        //--------------------.
        if (enc1.isRightH())
        {
          if (sndFlag) sndFlag = false;
          setMaximumRate();
        }
        //--------------------.
        if (enc1.isLeftH())
        {
          if (sndFlag) sndFlag = false;
          setMinimumRate();
        }
        //--------------------.
      }
      break;
    //-------------------------------------------------------------
    // Режим выбора разового объема -------------------------------
    //-------------------------------------------------------------
    case SELECT_VOL:
      {
        //--------------------
        if (enc1.isHolded())
        {
          if (newDrinkVol)
          {
            drinkBackCounter = (uint32_t)drinkVolume * stepsForOneMl - 1;
          }
          newDrinkVol = false;
          resumeRun();
        }
        //--------------------
        if (enc1.isClick())
        {
          currentMode = PAUSED;
          if (newDrinkVol)
          {
            drinkBackCounter = (uint32_t)drinkVolume * stepsForOneMl - 1;
          }
          newDrinkVol = false;
          lcd.setCursor(0, 0);
          lcd.print(STRING_01);
        }
        //--------------------
        if (enc1.isRight())
        {
          if (drinkVolume >= 1000) {
            drinkVolume = drinkVolume + volBigStep;
          } else {
            if (drinkVolume >= 100) {
              drinkVolume = drinkVolume + volMidStep;
            } else {
              drinkVolume = drinkVolume + volStep;
            }
          }
          if (drinkVolume >= maxDrinkVol) drinkVolume = maxDrinkVol;
          lcd.setCursor(rpVal, 3*yFONT);
          lcd.print(formatNum(drinkVolume, 5));
          newDrinkVol = true;
        }
        //--------------------
        if (enc1.isLeft())
        {
          if (drinkVolume >= (1000 + volBigStep)) {
            drinkVolume = drinkVolume - volBigStep;
          } else {
            if (drinkVolume >= (100 + volMidStep)) {
              drinkVolume = drinkVolume - volMidStep;
            } else {
              //drinkVolume = drinkVolume - volStep;
              if (drinkVolume > volStep) {
                drinkVolume = drinkVolume - volStep;
              }
              else drinkVolume = 0;
            }
          }
          //if (drinkVolume <= 0) drinkVolume = 0;
          lcd.setCursor(rpVal, 3*yFONT);
          lcd.print(formatNum(drinkVolume, 5));
          newDrinkVol = true;
        }
        //--------------------
        if (enc1.isRightH())
        {
          drinkVolume = maxDrinkVol;
          lcd.setCursor(rpVal, 3*yFONT);
          lcd.print(formatNum(drinkVolume, 5));
          newDrinkVol = true;
        }
        if (enc1.isLeftH())
        {
          drinkVolume = 0;
          lcd.setCursor(rpVal, 3*yFONT);
          lcd.print(formatNum(drinkVolume, 5));
          newDrinkVol = true;
        }
      }
      break;
    //-------------------------------------------------------------
    default:
      //-------------------------------------------------
      break;
  }
  //-------------------------------------------------
}
//---------------------- Конец основного цикла -----------------------------------------
//--------------------------------------------------------------------------------------
void resumeRun()
{
  lcd.setCursor(0, 0);
  lcd.print(STRING_00);
  currentMode = RUNNING;
  if (rate != 0)
  {
    stepEnabled = true;
  }
  else
  {
    stepEnabled = false;
  }
}
//--------------------------------------------------------------------------------------
// Попытаться перейти к калибровке насоса
void tryToTune()
{
  counterA++;
  if (counterA > 20)
  {
    counterA = 0;
    currentMode = TUNING;
    printTuneScreen();
  }
}
//--------------------------------------------------------------------------------------
// Установить максимальную скорость отбора
void setMaximumRate()
{
  rate = maximumRate;
  calcOCR1A();
  lcd.setCursor(rpVal, 1*yFONT);
  lcd.print(formatNum(rate, 5));
}
//--------------------------------------------------------------------------------------
// Установить отбор = 0
void setMinimumRate()
{
  rate = 0;
  calcOCR1A();
  lcd.setCursor(rpVal, 1*yFONT);
  lcd.print(formatNum(rate, 5));
}
//--------------------------------------------------------------------------------------
// Увеличение скорости отбора ----------------------------------------------------------
void increaseRate()
{
  if (rate >= 1000) {
    rate = rate + rateBigStep;
  } else {
    if (rate >= 200) {
      rate = rate + rateMidStep;
    } else {
      rate = rate + rateStep;
    }
  }
  if (rate >= maximumRate) rate = maximumRate;
  calcOCR1A();
}
//--------------------------------------------------------------------------------------
// Уменьшение скорости отбора ----------------------------------------------------------
void decreaseRate()
{
  if (rate >= (1000 + rateBigStep)) {
    rate = rate - rateBigStep;
  } else {
    if (rate >= (200 + rateMidStep)) {
      rate = rate - rateMidStep;
    } else {
      //rate = rate - rateStep;
      if (rate > rateStep) {
        rate = rate - rateStep;
      }
      else rate = 0;
    }
  }
  calcOCR1A();
}
//--------------------------------------------------------------------------------------
// Вывод на экран текущих значений в рабочем режиме
void printValues()
{
  lcd.setCursor(rpVal, 2*yFONT);
  lcd.print(formatNum((totalVolume % 100000), 5));
  if (currentMode != SELECT_VOL)
  {
    if (drinkVolume != 0)
    {
      lcd.setCursor(rpVal, 3*yFONT);
      lcd.print(formatNum((remainVolume % 100000), 5));
    }
    else
    {
      lcd.setCursor(rpVal, 3*yFONT);
      lcd.print(" ----");
    }
  }
}
//--------------------------------------------------------------------------------------
void printTuneScreen()
{
  lcd.setCursor(0, 0);
  lcd.print(STRING_10);
  lcd.setCursor(0, 1*yFONT);
  lcd.print("Rate:    0 Vol:     ");
  lcd.setCursor(0, 2*yFONT);
  lcd.print("Step counter:       ");
  lcd.setCursor(tpVal, 2*yFONT);
  lcd.print(formatNum(stepsCount, 7));
  lcd.setCursor(0, 3*yFONT);
  lcd.print("Current 100 :       ");
  lcd.setCursor(tpVal, 3*yFONT);
  lcd.print(formatNum(stepsFor100ml, 7));
  delay(5000);
}
//--------------------------------------------------------------------------------------
void printMainScreen()
{
  lcd.setCursor(0, 0);
  lcd.print(STRING_00);
  lcd.setCursor(0, 1*yFONT);
  lcd.print("  Speed  :     0 mlh");
  lcd.setCursor(0, 2*yFONT);
  lcd.print("  Total  :     0 ml ");
  lcd.setCursor(0, 3*yFONT);
  lcd.print("Autostop :     0 ml ");
}
//--------------------------------------------------------------------------------------
// То, что выполняется каждую секунду --------------------------------------------------
void oneSecSub()
{
  calcTotalVolume();
  if (currentMode < TUNING)
  {
    printValues();
  }
  else
  {
    lcd.setCursor(tpVal+13, 1*yFONT);
    lcd.print(formatNum((totalVolume % 100000), 5));
  }
}
//--------------------------------------------------------------------------------------
// То, что выполняется каждую десятую долю секунды -------------------------------------
void oneTenthSub()
{
  if (currentMode < TUNING)
  {
    tmasStop = !digitalRead(TMAS);

    if (sndFlag)
    {
      if (counterB < 8)
      {
        if (bitRead(pattern,counterB))
        {
          digitalWrite(PIEZO,0);
          digitalWrite(LED,0);
        }
        else
        {
          digitalWrite(PIEZO,1);
          digitalWrite(LED,1);
        }
        counterB++;
      }
      else
      {
        if (counterB > 16) counterB = 0;
        else counterB++;
      }
    }
    else
    {
      digitalWrite(PIEZO,1);
      digitalWrite(LED,1);
      counterB = 0;
    }
  }
  // ----------
  else
  {
    lcd.setCursor(tpVal, 2*yFONT);
    lcd.print(formatNum(stepsCount, 7));
  }
}
//--------------------------------------------------------------------------------------
// Прерывание таймера 2 (1000 раз в секунду) -------------------------------------------
ISR_T2_COMPA
{
  enc1.tick();
  thousandth++;                                         // Счетчик миллисекунд
  if (thousandth >= 100)                                // Началась новая десятая доля секунды
  { thousandth = 0; tenth++; newTenth = true;           // Счетчик десятых долей секунды
    if (tenth >= 10) {
      tenth = 0;  // Началась новая секунда
      newSecond = true;
    }
  }
}

//--------------------------------------------------------------------------------------
// Прерывание по совпадению А в таймере 1 ----------------------------------------------
ISR_T1_COMPA
{
  if ((stepEnabled) or (!(stepEnabled) and !(digitalRead(STEP))))
    // Если шаги разрешены или запрещены и уровень на тактовом выходе низкий, то:
  {
    digitalWrite(STEP, !digitalRead(STEP));   // Меняем уровень на шагательной ноге на противоположный
    if (digitalRead(STEP))                    // Если был переход 0->1,
    {
      stepsCount++;                           // то увеличиваем счетчик шагов
      TIMER1_COMPA_setValue(timer1EndValue);  // и перезагружаем значение сравнения счетчика.
      TIMER1_setClock(T1ClockDivider);
      if (drinkVolume != 0)                   // Если задан ненулевой объем наливайки
      {
        if (drinkBackCounter == 0)            // и налито, сколько запрошено,
        {
          stepEnabled = false;                // останавливаем отбор
          drinkStop = true;                   // и устанавливаем флаг готовности дринка. (обрабатывается в основном цикле)
        }
        else
        {
          drinkBackCounter--;                 // Иначе уменьшаем дринковый счетчик.
        }
      }
    }
  }
}

//--------------------------------------------------------------------------------------
// Расчет числа, загружаемого в OCR1A в зависимости от нужной скорости отбора ----------
void calcOCR1A()
{
  stepEnabled = false;
  //temp = round((float)OCR1ABase/ rate);             // Fcpu = 16000000 Гц, N - количество шагов на 1 мл,
  temp = OCR1ABase / rate;
  tempA = PRESCALER_8;
  if (temp > 65535)
  {
    temp = temp / 8; tempA = PRESCALER_64;
    if (temp > 65535)
    {
      temp = temp / 8; tempA = PRESCALER_256;
    }
  }
  temp = temp - 1;                              // K - делитель перед счетчиком Т1 (64), R - скорость отбора мл/час
  if (temp > 65535)
  {
    rate = 0;
  }
  else
  {
    noInterrupts();                               // Обеспечиваем атомарность действия
    timer1EndValue = temp;                        //
    T1ClockDivider = tempA;
    interrupts();                                 // Восстанавливаем прерывания
  }
}
//--------------------------------------------------------------------------------------
// Пересчёт количества шагов в миллилитры ----------------------------------------------
void calcTotalVolume()
{
  totalVolume = round((float)stepsCount / stepsForOneMl);
  //totalVolume = round(stepsCount / stepsForOneMl);
  remainVolume = round((float)(drinkBackCounter + 1) / stepsForOneMl);
  //remainVolume = round((drinkBackCounter + 1) / stepsForOneMl);
}
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
void tryToSaveStepsFor100ml()
{
  counterA++;
  if (counterA > 10)
  {
    counterA = 0;
    if (stepsCount < minStepsFor100ml)
    {
      lcd.setCursor(0, 0);
      lcd.print(STRING_12);
      delay(2000);
      lcd.setCursor(0, 0);
      lcd.print(STRING_11);
    }
    else
    {
      if (stepsCount > maxStepsFor100ml)
      {
        lcd.setCursor(0, 0);
        lcd.print(STRING_13);
        delay(2000);
        lcd.setCursor(0, 0);
        lcd.print(STRING_11);
      }
      else
      {
        //stepsCount = 100*(stepsCount/100);
        EEPROM.put(0, stepsCount);
        EEPROM.get(0, stepsFor100ml);
        //stepsForOneMl = round((float)stepsFor100ml / 100);
        stepsForOneMl = stepsFor100ml / 100;
        calcOCR1A();
        rate = 0;
        stepEnabled = false;
        lcd.setCursor(tpVal, 3*yFONT);
        lcd.print(formatNum(stepsFor100ml, 7));
        lcd.setCursor(0, 0);
        lcd.print(STRING_14);
        delay(2000);
        lcd.setCursor(0, 0);
        lcd.print(STRING_11);
        //setup();
      }
    }
  }
}
//--------------------------------------------------------------------------------------
String formatNum(uint32_t Number, int lenth)
{
  String temp = String(Number);
  int j = temp.length();
  String prefix = "";
  for (int i = 0; i < lenth - j; i++)
  {
    prefix = " " + prefix;
  }
  String(temp1) = prefix + String(temp);
  return temp1;
}
//--------------------------------------------------------------------------------------
