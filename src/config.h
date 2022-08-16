#ifndef _CONFIG_H_ 
#define _CONFIG_H_

#define USE_DS18B20

#define AUTO_RATE 7 //процент уменьшения отбора после внешнего стопа.
#define MIN_RATE 1000 //минимальный отбор тела, ниже которого не снижаемся
#define RMS_CURRENT 1200
#define MICROSTEPS 8
 
// Подключение TMC2209
#define DIR 2
#define STEP 3                                // Тактовые импульсы на драйвер ШД
#define STEP_PIN         3 // Step
#define SW_RX            5 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            4 // 1kOm TMC2208/TMC2224 SoftwareSerial transmit pin
// Подключение энкодера
#define KEY 6                             // Кнопка энкодера    
#define SW2 7                             // Сигнал энкодера 1 // Если кручение влево/вправо перепутано,
#define SW1 8                            // Сигнал энкодера 2 // можно поменять местами 
#define DRV_EN 9                               // Тактовые импульсы на драйвер ШД
#define START_ACCEL 2000

#define STALL_VALUE     100 // [0..255]
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// Внешнее управление
#define TMAS  A0                              // Сюда подключаем управление от TMAS, низкий уровень вызовет паузу
#define PIEZO A1                              // Здесь пьезопищалка
#define LED   A2                              // Светодиод на панели
#define soundA B00000101                      // Ритм автостопа
#define soundB B01011101                     // Ритм внешнего стопа
                              // Светодиод на панели


 

//#define DS_ALARM_TIME (10*60*1000L)    //время , после которого в режиме стопа по температуре включает пищалку
//#define DS_TIME_CHECK 1500
#endif //_CONFIG_H_