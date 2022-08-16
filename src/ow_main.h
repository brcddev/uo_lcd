#ifndef _OW_MAIN_H_
#define _OW_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif
typedef void (*ReadyTemp_t)(int16_t);
void ow_setup();
void ow_loop();
void set_onReadyTemp(ReadyTemp_t fn);
#ifdef __cplusplus
}
#endif
#endif