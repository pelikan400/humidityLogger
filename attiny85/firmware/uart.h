#ifndef __UART_H__
#define __UART_H__

#include "RingBuffer.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

class UARTBitBangTimer0 {
public:
   uint8_t currentByte;
   uint8_t bitCounter;
   RingBuffer txBuffer;
   
public:
   UARTBitBangTimer0();
   void init();
   int sendc( char c );
};

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

#define PUTS( x ) puts_P( PSTR( x ) )

extern UARTBitBangTimer0 uart;


#endif
