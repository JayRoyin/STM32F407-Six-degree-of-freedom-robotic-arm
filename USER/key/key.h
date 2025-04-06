#ifndef __key_H__
#define __key_H__

#include "stm32f4xx_hal.h"

uint8_t KEY_SCAN(void);
void KEY_ROW_MOD_IN(void);
void KEY_COL_MOD_IN(void);
void KEY_ROW_MOD_OUT(void);
void KEY_COL_MOD_OUT(void);
void KEY_ROW_OUT(uint8_t i);
void KEY_COL_OUT(uint8_t i);
uint8_t KEY_COL_RED(void);
uint8_t KEY_ROW_RED(void);
#endif
