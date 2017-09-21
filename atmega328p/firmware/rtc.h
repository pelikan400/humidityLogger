#ifndef DE_EDBA_ARLINGTON_RTC_H
#define DE_EDBA_ARLINGTON_RTC_H

#include <stdint.h>

#define RTC_TIMER_FREQUENCY   20
#define TICKS_RESOLUTION  100

void initializeRealTimeClock( void );
uint16_t getTicks( void );

#endif
