#include "config.h"
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
//#include <U8g2lib.h>
//#include <EEPROM.h>                           // Стандартная библиотека
#include <avr/eeprom.h>
#include "GyverEncoder.h"                     // Библиотеки с сайта 
#include "directTimers.h"                     // https://codeload.github.com/AlexGyver/GyverLibs/zip/master
#include "Wire.h"
#include <TMCStepper.h>
#include "uo.h"
#ifdef USE_DS18B20
#include "ow_main.h"
float curr_temp,set_temp;
bool temp_check_enable=false;
bool ds_found=false;
uint32_t alarm_timer;
uint8_t ds_delta=2;
#endif
uint32_t ee_stepsFor100ml  EEMEM=0;
//U8G2_SSD1309_128X64_NONAME2_1_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ A0, /* reset=*/ A1);

// Select your stepper driver type
//TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);

using namespace TMC2208_n;
Encoder enc1(SW1, SW2, KEY, TYPE2);
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

#define PAUSED_BY_TEMP        20
//----------------------------------------------
// Константы
//----------------------------------------------
#define maximumRate           4500            //  Максимальная скорость отбора мл/час
#define rateStep              10              //  Мелкий шаг регулировки отбора
#define rateMidStep           50              //  Средний шаг регулировки отбора
#define rateBigStep           100             //  Крупный шаг регулировки отбора
#define volStep               10              //  Шаг регулировки дозатора
#define volMidStep            50              //  Средний шаг регулировки дозатора
#define volBigStep            500             //  Крупный шаг регулировки дозатора
#define maxDrinkVol           10000            //  Максимальный объем, выдаваемый дозатором
#define defaultDrink          0               //  Объем дозатора по умолчанию. 0 - без ограничения.
#define defaultpValsFor100ml  600000          //  Количество шагов потребное для отбора 100 мл по умолчанию при шаге/8
#define minStepsFor100ml      50000          //  Минимально возможное число шагов для отбора 100 мл при шаге/8
#define maxStepsFor100ml      1500000
//----------------------------------------------
// Переменные
//----------------------------------------------
volatile uint32_t stepsFor100ml;                  //  Количество шагов потребное для отбора 100 мл
volatile unsigned int  stepsForOneMl;                  //  Количество шагов потребное для отбора 1 мл
volatile uint32_t stepsCount     = 0;             //  Счетчик количества шагов
volatile uint32_t drinkBackCounter = 0;           //  Обратный счётчик шагов дозатора
volatile uint32_t temp           = 0;             //  Временная переменная для разнообразных нужд
volatile uint32_t tempA          = 0;             //  Временная переменная для разнообразных нужд
volatile uint32_t OCR1ABase      = 0;             //  Коэффициент для расчета OCR1A
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
//const char STRING_00[]="--- NORMAL  MODE ---";
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

#define STRING_20 "TEMP MODE 00.0->00.0"
#define STRING_21 "-- PAUSED by Temp --"
//--------------------------------------------------------------------------------------

typedef struct 
{
  uint16_t BigStep;
  uint16_t MidStep;
  uint8_t Step;
} step_t;

typedef struct 
{
  bool *disabled;
  volatile uint16_t *var;
  uint16_t MaxVal;
  step_t *Step;
} inc_dec_t;

bool flagAcceleration=false;
uint16_t rateAcceleration=START_ACCEL;
uint16_t new_rate;
//void calcOCR1A();
String formatNum(uint32_t Number, int lenth);

step_t st_rate;
inc_dec_t id_rate,id_vol;
void decreaseVal(inc_dec_t *v);
void increaseVal(inc_dec_t *v);
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
void tryToTune();
uint16_t calcCRC16(uint8_t const *buf, uint32_t len);
//uint16_t prescalers[]={0,1,8,64,256,1024};
//--------------------------------------------------------------------------------------
//void calcBaseOCR1A(){
  //
  //uint8_t prescaler=TCCR1B &((1 << CS12) | (1 << CS11) | (1 << CS10));
  //BaseOCR1A=round((float)F_CPU*3600/2/prescalers[prescaler]/ stepsForOneMl);
//}
//--------------------------------------------------
/*
inline void print_state2()
{
  char buff[12];
  float data;
  String s;
  if (!driver.test_connection()){
  data = (float)driver.rms_current()/1000;
  dtostrf(data,3,1,&buff[0]);
  s=String(buff)+":";
  s=s+formatNum(driver.microsteps(),3)+":";
  }else{
    s="drv ERR:";
  }
  s=s+String(prescalers[(uint8_t)TCCR1B &((1 << CS12) | (1 << CS11) | (1 << CS10))])+":";
  lcd.setCursor(0, 4*yFONT);
  lcd.print(s);
}
*/
inline void print_name(String s)
{
  lcd.setCursor(0, 0);
  lcd.print(s);
}
//MIN_RATE минимальный отбор тела, ниже не опускаемся
#ifdef AUTO_RATE //процент уменьшения отбора
void auto_rate(){
  uint16_t dec_rate;
  if (rate > MIN_RATE){ //пропускаем головы и подголовки 
    dec_rate=rate/100*AUTO_RATE;
    if ((rate-dec_rate)<MIN_RATE)rate=MIN_RATE;
    else rate-=dec_rate; 
    calcOCR1A();
  }
}
#endif

#ifdef USE_DS18B20
void onReadyTemp(int16_t raw){
  curr_temp = float(raw/ 16);
  if(temp_check_enable){
    if((set_temp+ds_delta/10)>curr_temp){
      stepEnabled = false;
      currentMode = PAUSED_BY_TEMP;
      print_name(STRING_21);
      //alarm_timer=millis();
    }
  }  
}

inline void print_temp_pause()
{
  char buff[12];
  String s;
  dtostrf(curr_temp,4,1,&buff[0]);
  //buff[5]=":";
  s=String(buff)+":";
  dtostrf(set_temp+ds_delta/10,4,1,buff);
  s+=String(buff);
  lcd.setCursor(10, 0);
  lcd.print(s);
}

#endif


void pauseRun(){
  digitalWrite(DRV_EN, HIGH);
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
          drinkBackCounter = (uint32_t)drinkVolume * (uint32_t)stepsForOneMl - 1;
        }
        break;

        case DOZER_SET_RATE:
        {
          rate=cmd.val;
          if (rate > maximumRate)
            rate = maximumRate;
          calcOCR1A();
          resumeRun();
          lcd.setCursor(rpVal, 1*yFONT);
          lcd.print(formatNum(rate, 5));
        }
        break;

        case DOZER_SET_TOTAL_VOLUME:
        {
          totalVolume = (uint16_t)cmd.val;
          stepsCount = (uint32_t)totalVolume * (uint32_t)stepsForOneMl; 
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
  //u8g2.begin();
  //u8g2.enableUTF8Print();		// enable UTF8 support for the Arduino print()
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

  #ifdef USE_DS18B20
  ow_setup();
  set_onReadyTemp(onReadyTemp);
  #endif
  //drv
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  //driver.rms_current(1200); // mA
   driver.rms_current(RMS_CURRENT); // mA
  driver.microsteps(MICROSTEPS);//8
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);
  driver.push();
  /*
    Serial.begin(115200);
    Serial.print(F("\nTesting connection..."));
    uint8_t result = driver.test_connection();
    if (result) {
        Serial.println(F("failed!"));
        Serial.print(F("Likely cause: "));
        switch(result) {
            case 1: Serial.println(F("loose connection")); break;
            case 2: Serial.println(F("Likely cause: no power")); break;
        }
        Serial.println(F("Fix the problem and reset board."));
        abort();
    }
    Serial.println(F("OK"));
    Serial.println(driver.microsteps());
    Serial.println(driver.rms_current());
*/
    
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
  stepsFor100ml=eeprom_read_dword(&ee_stepsFor100ml);//EEPROM.get(0,stepsFor100ml);
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
  //Serial.println(stepsFor100ml);

  st_rate = (step_t) {rateBigStep, rateMidStep, rateMidStep};
  id_rate = (inc_dec_t){&flagAcceleration,&rate,maximumRate,&st_rate};
  id_vol =  (inc_dec_t){NULL,&drinkVolume,maxDrinkVol,&st_rate};
}
//--------------------------------------------------------------------------------------
//---------------------- Начало основного цикла ----------------------------------------
void loop()
{
  if (TWAR != TWI_SA) TWAR = TWI_SA;
  #ifdef USE_DS18B20  
  ow_loop();
  #endif 

  if (newSecond) {
    newSecond = false;
    oneSecSub();
  }
  if (newTenth)  {
    newTenth = false;
    oneTenthSub();
    if (flagAcceleration){
      if(rate>=new_rate){
        flagAcceleration=false;
        rate=new_rate;

      }
      else{
        rate+=200;
        if(rate>=new_rate){
          rate=new_rate;
          flagAcceleration=false;
        }
      }
      calcOCR1A();
      stepEnabled = true;
    }    
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
          #ifdef AUTO_RATE
          auto_rate();
          #endif
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
    // Остановка по превышению температуры-------------------------
    //-------------------------------------------------------------
    #ifdef USE_DS18B20
    case PAUSED_BY_TEMP:
      {
        if ((curr_temp<=set_temp))
        {
          sndFlag = false;
          #ifdef AUTO_RATE
          auto_rate();
          #endif 
          resumeRun();
        }
        //--------------------
        if (enc1.isRight()){
          ds_delta++;
          if (ds_delta>20) ds_delta=20;
          print_temp_pause();
        }
        if (enc1.isLeft()){
          ds_delta--;
          if (ds_delta<1)ds_delta=1;
          print_temp_pause();
        }        
        if((millis()-alarm_timer)>DS_ALARM_TIME){
          pattern = soundB;
          sndFlag = true;
        }
        if (enc1.isClick())
        {
          sndFlag = false;
          alarm_timer=millis();
        }
        //--------------------
      }
      break;
      #endif
    //-------------------------------------------------------------

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
          /*
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
          */
          increaseVal(&id_vol);
          lcd.setCursor(rpVal, 3*yFONT);
          lcd.print(formatNum(drinkVolume, 5));
          newDrinkVol = true;
        }
        //--------------------
        if (enc1.isLeft())
        {
          /*
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
          */
          decreaseVal(&id_vol);
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
  digitalWrite(DRV_EN, LOW);
  //lcd.setCursor(0, 0);
  //lcd.print(STRING_00);
  #ifdef USE_DS18B20
  if (temp_check_enable)print_name(STRING_20);
  else print_name(STRING_00);
  #else
  print_name(STRING_00);
  #endif  
  currentMode = RUNNING;
    if (!flagAcceleration){
      if (rate>START_ACCEL){
        flagAcceleration=true;
        new_rate=rate;
        rate=START_ACCEL;
        calcOCR1A();
      }
      else{
        flagAcceleration=false;
      }
    }  
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
  if (flagAcceleration)return;
  rate = maximumRate;
  resumeRun();// calcOCR1A();
  lcd.setCursor(rpVal, 1*yFONT);
  lcd.print(formatNum(rate, 5));
}
//--------------------------------------------------------------------------------------
// Установить отбор = 0
void setMinimumRate()
{
  if (flagAcceleration)return;
  rate = 0;
  calcOCR1A();
  lcd.setCursor(rpVal, 1*yFONT);
  lcd.print(formatNum(rate, 5));
}
//--------------------------------------------------------------------------------------
// Увеличение скорости отбора ----------------------------------------------------------
void increaseRate()
{
  /*
  if (flagAcceleration)return;
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
  */
  increaseVal(&id_rate);
  calcOCR1A();
}
//--------------------------------------------------------------------------------------
// Уменьшение скорости отбора ----------------------------------------------------------
void decreaseRate()
{
  /*
  if (flagAcceleration)return;
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
  */
  decreaseVal(&id_rate);
  calcOCR1A();
}

void increaseVal(inc_dec_t *v)
{
  if (v->disabled!=NULL && *v->disabled) return;
  if (*v->var >= 1000) {
    *v->var = *v->var + v->Step->BigStep;
  } else {
    if (*v->var >= 200) {
      *v->var = *v->var + v->Step->MidStep;
    } else {
      *v->var = *v->var + v->Step->Step;
    }
  }
  if (*v->var >= v->MaxVal) *v->var = v->MaxVal;
  
}
//--------------------------------------------------------------------------------------
// Уменьшение скорости отбора ----------------------------------------------------------
void decreaseVal(inc_dec_t *v)
{
  if (v->disabled!=NULL && *v->disabled) return;
  if (*v->var >= (1000 + v->Step->BigStep)) {
    *v->var = *v->var - v->Step->BigStep;
  } else {
    if (*v->var >= (200 + v->Step->MidStep)) {
      *v->var = *v->var - v->Step->MidStep;
    } else {
      //rate = rate - rateStep;
      if (*v->var > v->Step->Step) {
        *v->var = *v->var - v->Step->Step;
      }
      else *v->var = 0;
    }
  }
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
  if ((stepEnabled) or (!(stepEnabled) and bit_is_clear(STEP_PORT,STEP_PIN)))// !(digitalRead(STEP))))
    // Если шаги разрешены или запрещены и уровень на тактовом выходе низкий, то:
  {
    STEP_PORT^=(1<<STEP_PIN);// digitalWrite(STEP, !digitalRead(STEP));   // Меняем уровень на шагательной ноге на противоположный
    if (bit_is_set(STEP_PORT,STEP_PIN))//   if (digitalRead(STEP))                    // Если был переход 0->1,
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
        eeprom_write_dword(&ee_stepsFor100ml,stepsCount);//EEPROM.put(0, stepsCount);
        stepsFor100ml=eeprom_read_dword(&ee_stepsFor100ml);//EEPROM.get(0, stepsFor100ml);
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
  char buf[lenth+1];
  dtostrf(Number,lenth,0,&buf[0]);
  /*
  String temp = String(Number);
  int j = temp.length();
  String prefix = "";
  for (int i = 0; i < lenth - j; i++)
  {
    prefix = " " + prefix;
  }
  //String(temp1) = prefix + String(temp);
  return prefix + String(temp);
  */
  return String(buf);
}
//--------------------------------------------------------------------------------------
uint16_t calcCRC16(uint8_t const *buf, uint32_t len)
{
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < len; i++)
    {
        temp = temp ^ buf[i];
        for (unsigned char j = 1; j <= 8; j++)
        {
            flag = temp & 0x0001;
            temp >>=1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return temp;
}