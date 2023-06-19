#ifndef _CONFIG_H_
#define _CONFIG_H_

//#define USE_DS18B20
#define DS_ALARM_TIME (1000*10)
//#define AUTO_RATE 7 //процент уменьшения отбора после внешнего стопа.
#define MIN_RATE 900 //минимальный отбор тела, ниже которого не снижаемся
#define RMS_CURRENT 1200
#define MICROSTEPS 8

// Подключение TMC2209
#define DIR 2
#define STEP 3      // PD3 Тактовые импульсы на драйвер ШД
#define STEP_PORT PORTD
#define STEP_PIN  3
#define SW_TX            4 // 1kOm TMC2208/TMC2224 SoftwareSerial transmit pin
#define SW_RX            5 // TMC2208/TMC2224 SoftwareSerial receive pin

#define DRV_EN 9
// Подключение энкодера
#define KEY 6                             // Кнопка энкодера
#define SW2 7                             // Сигнал энкодера 1 // Если кручение влево/вправо перепутано,
#define SW1 8                            // Сигнал энкодера 2 // можно поменять местами
                             // Тактовые импульсы на драйвер ШД
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
#define WAIT_I2C_QUERY 60000

/*
Alright, I figured it out looking at raspberry pi drivers

the formula is rps / 0.715 * steps * microsteps

rps is revolutions per second
0.715 is the built in pulse generator
steps is the number of steps on the motor
microsteps is the microsteps

so 1 rev per second on a 200 step motor set to 16 micro steps is

1 / 0.715 * 200 * 16 = 4476

And I've tested it and it works. Super cool.
void initStallGuard():

		print("initialization");
		stall = 5;
		motor_microsteps = 2;	# 1 2 4 8 16 32 64 128
		tcools = 200;
		current_ma = 100;
		vactual = 768;

		driver.set_toff(4) ;										# For operation with StealthChop, this parameter is not used, but it is required to enable the motor. In case of operation with StealthChop only, any setting is OK
		driver.set_blank_time(24); 								# Recommended blank time select value
		driver.set_iscale_analog(False); 							# Disbaled to use the extrenal current sense resistors 
		driver.set_internal_rsense(False) ;						# Use the external Current Sense Resistors. Do not use the internal resistor as it can't handle high current.
		driver.set_mstep_resolution_reg_select(True); 			# Microstep resolution selected by MSTEP register and NOT from the legacy pins.
		driver.set_current(current_ma) ;							# Set the current in milliamps. 
		driver.set_stallguard_threshold(stall);					# Set the stall value from 0-255. Higher value will make it stall quicker.
		driver.set_microstepping_resolution(motor_microsteps);	# Set the number of microsteps. Due to the "MicroPlyer" feature, all steps get converterd to 256 microsteps automatically. However, setting a higher step count allows you to more accurately more the motor exactly where you want.
		driver.set_coolstep_threshold(tcools) ;					# Minimum speed at which point to turn on StallGuard. StallGuard does not work as very low speeds such as the beginning of acceleration so we need to keep it off until it reaches a reliable speed.
		driver.set_tpwmthrs(0); 									# DisableStealthChop PWM mode/ Page 25 of datasheet
		driver.set_smart_current_min_threshold(0);				# Turn off smart current control, known as CoolStep. It's a neat feature but is more complex and messes with StallGuard.
		driver.set_spreadcycle(False);	# false means its going to use stealth chop
		driver.set_motor_enabled(True);
		driver.set_vactual(vactual);
*/


#endif //_CONFIG_H_