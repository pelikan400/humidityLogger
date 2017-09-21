////////////////////////////////////////////////////////////////////////////////////////////////////
//
// 
//
//
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef USART_ATMEGA8_H
#define USART_ATMEGA8_H

#include "RingBuffer.h"

class USART {
 public:
  RingBuffer  usartReceiveBuffer;
  RingBuffer  usartTransmitBuffer;

 public:
  void init();
  void sendString( const char * p );
  int sendc( char c );
  char getc();
  void sleep();
  void sendHexByte( uint8_t );
  void dumpHexBuffer( uint8_t * p, uint16_t size );
};

extern USART usart;

#endif
