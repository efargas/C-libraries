#ifndef LIBTHERMOCLICK_H_INCLUDED
#define LIBTHERMOCLICK_H_INCLUDED
#include <stdint.h>
static float getTempV(void);
static float getJunctionTempV(void);
static uint8_t CheckError(void);
static void SetUpSpi(int*);
static void PrintData(void);
#endif // LIBTHERMOCLICK_H_INCLUDED

