#ifndef _UO_H_
#define _UO_H_
#ifdef __cplusplus
extern "C" {
#endif


#include "stdint.h"

enum DOZER_MODE_STATE{
  DOZER_MODE_RUNNING=0,
  DOZER_MODE_PAUSED,
  DOZER_MODE_PAUSED_BY_EXT,
  DOZER_MODE_PAUSED_BY_VOL,
  DOZER_MODE_SELECT_VOL,
  DOZER_MODE_TUNING,
  DOZER_MODE_TUNING_PAUSE,
  DOZER_MODE_MAX
};
enum dozer_cmd_t
{
  DOZER_SET_MODE=0,         //Текущий режим работы
  DOZER_SET_RATE,           // Скорость отбора мл/час 0 - maximumRate 
  DOZER_SET_DRINK_VOLUME,   // Шаг рОзлива
  DOZER_SET_TOTAL_VOLUME,   // Общий объем отбора мл
  DOZER_SET_MAX
};

#pragma pack(push, 1)
typedef struct 
{
   uint8_t cmd;
   uint16_t val;
   uint16_t crc;
} master_cmd_t;
#pragma pack(pop)

typedef struct 
{
   uint16_t currentMode;
   uint16_t rate;
   uint16_t drinkVolume    ;             // Шаг рОзлива
   uint16_t totalVolume    ;             // Общий объем отбора мл
   uint16_t crc;
}dozer_data_t;
//#define DOZER_I2C_ADDR (0x1e)
#define DOZER_I2C_ADDR (0x1e)
#define TWI_SA (DOZER_I2C_ADDR << 1)


#ifdef __cplusplus
}
#endif
#endif