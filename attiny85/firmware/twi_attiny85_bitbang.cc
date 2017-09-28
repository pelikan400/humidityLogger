////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <avr/io.h>
#include <avr/interrupt.h>  /* for sei() */

#include "twi.h"

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

#define SCL _BV( PORTB2 )
#define SDA _BV( PORTB0 )
#define TWI_TIMING 15u

#define SDA_LOW      DDRB |= SDA
#define SDA_HIGH     DDRB &= ~SDA
#define SCL_LOW      DDRB |= SCL
#define SCL_HIGH     DDRB &= ~SCL

////////////////////////////////////////////////////////////////////////////////////////////////////

bool TWI::timeTWIBit( bool monitorSCL ) {
   _NOP();
   _NOP();
   _NOP();
   while( monitorSCL && !( PINB & SCL ) ) {
      _NOP();
   }
   for( uint8_t i = 0; i < TWI_TIMING; ++i ) {
      _NOP();
   }
   while( monitorSCL && !( PINB & SCL ) ) {
      _NOP();
   }

   return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TWI::TWI() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void TWI::init() {
   SCL_HIGH;
   SDA_HIGH;
   PORTB &= ~SCL;  // do not activate pull-up because the 3.3V side allready have it active
   PORTB &= ~SDA;  // do not activate pull-up because the 3.3V side allready have it active
   timeTWIBit( false );
}



////////////////////////////////////////////////////////////////////////////////////////////////////

void TWI::sendStart() {
   // assume, that SCL is already HIGH because the I2C bus is idle
   SDA_LOW;
   timeTWIBit( false );
   SCL_LOW;
   timeTWIBit( false );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t TWI::writeBit( uint8_t bit ) {
   // assume SCL is LOW
   if( bit ) {
      SDA_HIGH; 
   }
   else {
      SDA_LOW;
   }
   timeTWIBit( false );
   SCL_HIGH;
   timeTWIBit( true );
   SCL_LOW;
   timeTWIBit( false );
   
   return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t TWI::readBit() {
   SDA_HIGH;
   timeTWIBit( false );
   SCL_HIGH;
   timeTWIBit( true );
   uint8_t x = ( PINB & SDA ) ? 1 : 0;
   SCL_LOW;
   timeTWIBit( false );

   return x;
}


////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t TWI::sendByte( uint8_t x ) {
   for( uint8_t i = 0; i < 8; ++i ) {
      writeBit( x & 0x80 );
      x = x << 1;
   }

   // read ACK bit
   uint8_t ack = readBit();
   return ack;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t TWI::receiveByte( uint8_t * r, uint8_t ack ) {
   uint8_t x = 0;
   for( uint8_t i = 0; i < 8; ++i ) {
      x = x << 1 | readBit();
   }

   *r = x;
   // read ACK bit
   uint8_t ack1 = writeBit( ack );
   return ack1;
}
   
////////////////////////////////////////////////////////////////////////////////////////////////////

void TWI::sendStop() {
   // assume SCL is LOW
   SDA_LOW;
   timeTWIBit( false );
   SCL_HIGH;
   timeTWIBit( true );
   SDA_HIGH;
   timeTWIBit( false );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t TWI::sendMessage( uint8_t adr, uint8_t* msg, uint16_t size ) {
   sendStart();
   uint8_t ack = sendByte( adr << 1 );
   
   for( uint16_t c = 0; c < size; ++c ) {
      ack = sendByte( *msg++ );
   }
   sendStop();
   
   return ack;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t TWI::receiveMessage( uint8_t adr, uint8_t* msg, uint16_t size ) {
   sendStart();
   uint8_t ack = sendByte( adr << 1  | 1 );
   
   for( uint16_t c = 0; c < size; ++c ) {
      ack = receiveByte( msg++, c+1 < size ? 0 : 1 );
   }
   sendStop();
   
   return ack;
}


////////////////////////////////////////////////////////////////////////////////////////////////////

TWI twi;

