////////////////////////////////////////////////////////////////////////////////////////////////////
//
// 
//
//
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "twi_atmega328p.h"
#include "sleep.h"
#include "usart.h"
#include "rtc-atmega8.h"

#include "i2c.h"

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

TWI twi;

////////////////////////////////////////////////////////////////////////////////////////////////////

TWI::TWI() : 
  statusCode( 0 ),
  busy( false ),
  keepLineBusy( false ),
  debug( false )
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void TWI::init() {
   PUTS( "Init TWI" );
   cli();
   // TWSR = 0;
   statusCode = 0;
   busy = false;
   keepLineBusy = false;
   TWCR = _BV( TWIE ) | _BV( TWEN ) | _BV( TWEA );
   TWBR = ( ( F_CPU / TWI_FREQ ) - 16 ) / 2;
   TWSR = 0;
   sei();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

static void inline sendStop(  uint8_t twcr ) {
   if( !twi.keepLineBusy ) {
      TWCR = ( twcr | _BV( TWSTO ) );
      if( twi.debug) {
         usart.sendc( 'p' );
      }
   }
   twi.busy = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#define TWCR_DEFAULT ( _BV( TWEN ) | _BV( TWINT ) | _BV( TWIE ) )
    
ISR( TWI_vect ) {
   PORTD ^= _BV( PORTD4 );    /* toggle the LED */
   uint8_t statusCode = TWSR & 0xf8;
   twi.statusCode = statusCode;
   switch( statusCode ) {
   case TWI_STATUS_CODE_START_TRANSMITTED: /* 0x08 */
   case TWI_STATUS_CODE_REPEAT_START_TRANSMITTED: /* 0x10 */
      TWDR = ( twi.destinationAddress << 1 ) | twi.direction;
      TWCR = TWCR_DEFAULT;
      if( twi.debug ) {
         usart.sendc( 's' );
      }
      break;
   case TWI_STATUS_CODE_SLA_W_TRANSMITTED_ACK:
   case TWI_STATUS_CODE_DATA_TRANSMITTED_ACK:
      if( twi.counter < twi.msgSize ) {
         TWDR = *( twi.msg + twi.counter );
         ++twi.counter;
	 TWCR = TWCR_DEFAULT;
         if( twi.debug ) {
            usart.sendc( 'w' );
         }
      }
      else {
         sendStop( TWCR_DEFAULT );
      }
      break;
   case TWI_STATUS_CODE_DATA_RECEIVED_ACK:
     *( twi.msg + twi.counter ) = TWDR;
     ++twi.counter;
     if( twi.debug ) {
        usart.sendc( 'd' );
     }
   case TWI_STATUS_CODE_SLA_R_TRANSMITTED_ACK:
     if( twi.msgSize == 0 ) {
        sendStop( TWCR_DEFAULT );
     }
     if( twi.counter + 1 < twi.msgSize ) {
       TWCR = TWCR_DEFAULT | _BV( TWEA );
     }
     else {
        TWCR = TWCR_DEFAULT & ~_BV( TWEA );
     }
     break;
   case TWI_STATUS_CODE_DATA_RECEIVED_NOT_ACK:
     *( twi.msg + twi.counter ) = TWDR;
     ++twi.counter;
     if( twi.debug ) {
        usart.sendc( 'd' );
     }
     sendStop( TWCR_DEFAULT );
     break;
   case TWI_STATUS_CODE_ARBITRATION_LOST:
      sendStop( TWCR_DEFAULT );
      break;
   default:
      sendStop( TWCR_DEFAULT );
      break;
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool TWI::sendMessage( uint8_t adr, uint8_t * p, uint16_t  s, bool klb ) {
   bool success = false;

   while( TWCR & _BV( TWSTO ) ) {}
   while( busy ) {}

   
   // load parameters 
   statusCode = 0xf8;
   direction = TWI_DIRECTION_SEND;
   destinationAddress = adr;
   msg = p;
   msgSize = s;
   counter = 0;
   keepLineBusy = klb;
   busy = true;
   
   // send start condition
   TWCR  = TWCR_DEFAULT | _BV( TWSTA );

   uint32_t t0 = getTicks();
   uint32_t t1 = getTicks();
   while( busy ) {
      t1 = getTicks();
      if( t1 - t0 > 300l ) {
         busy = false;
         putHex( TWSR );
         PUTS( " Timeout I2C Send" );
         break;
      }
   }

   success = ( s > 0 ) ? statusCode == TWI_STATUS_CODE_DATA_TRANSMITTED_ACK 
     : statusCode == TWI_STATUS_CODE_SLA_W_TRANSMITTED_ACK;

   if( !success && statusCode != TWI_STATUS_CODE_SLA_W_TRANSMITTED_NOT_ACK ) {
      putHex( statusCode );
      PUTS( " I2C Send StatusCode" );
   }
   return success;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool TWI::receiveMessage( uint8_t adr, uint8_t * p, uint16_t s, bool klb ) {
   bool success = false;

   while( TWCR & _BV( TWSTO ) ) {}
   while( busy ) {}

   // load parameters 
   statusCode = 0;
   direction = TWI_DIRECTION_READ;
   destinationAddress = adr;
   msg = p;
   msgSize = s;
   counter = 0;
   keepLineBusy = klb;
   busy = true;
   
   // send start condition
   TWCR  = TWCR_DEFAULT | _BV( TWSTA );

   uint32_t t0 = getTicks();
   uint32_t t1 = getTicks();
   while( busy ) {
      t1 = getTicks();
      if( t1 - t0 > 300l ) {
         busy = false;
         putHex( TWSR );
         PUTS( " Timeout I2C Receive" );
         break;
      }
   }

   success = ( s > 0 ) ? statusCode == TWI_STATUS_CODE_DATA_RECEIVED_NOT_ACK
      : statusCode == TWI_STATUS_CODE_SLA_R_TRANSMITTED_ACK;

   if( !success && statusCode != TWI_STATUS_CODE_SLA_R_TRANSMITTED_NOT_ACK ) {
      putHex( statusCode );
      PUTS( " I2C Receive StatusCode" );
   }
   return success;
}
