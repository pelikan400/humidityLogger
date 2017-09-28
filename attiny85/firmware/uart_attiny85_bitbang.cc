
#include <avr/io.h>
#include <avr/interrupt.h>  /* for sei() */
#include <stdio.h>
#include "uart.h"

UARTBitBangTimer0 uart;

////////////////////////////////////////////////////////////////////////////////////////////////////

UARTBitBangTimer0::UARTBitBangTimer0() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

static void initializeTimer0For9600Hz( ) {
  // initialize Timer0 
  cli();
  // halt the Timer0 and prescaler
  GTCCR = _BV( TSM );
  // WGM mode normal and set OC0A on compare match
  TCCR0A = _BV( WGM01 );
  // set prescaler to clk/64
  TCCR0B = _BV( CS01 ) | _BV( CS00 );
  TCNT0 = 0;
  // for 9600 baud
  OCR0A = 27; 
  OCR0B = 0;
  TIMSK = 0; 
  // TIMSK = _BV( OCIE0A ); 
  // start counter
  GTCCR = 0;
  sei();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

ISR( TIMER0_COMPA_vect ) {
   if( uart.bitCounter < 8 ) {
      if( uart.currentByte & 0x01 ) {
         // send 1
         PORTB  |= _BV( PORTB1 );  
      }
      else {
         // send 0
         PORTB &= ~_BV( PORTB1 );  
      }
      uart.currentByte >>= 1;
      ++uart.bitCounter;
   }
   else if( uart.bitCounter == 8 ) {
      // send stop bit
      PORTB  |= _BV( PORTB1 );  
      ++uart.bitCounter;
   }
   else {
      if( uart.txBuffer.empty() ) {
        // disable interrupts
	TIMSK &=  ~_BV( OCIE0A );
      }
      else {
         // send start bit
         uart.currentByte = uart.txBuffer.pop();
         uart.bitCounter = 0;
         PORTB &= ~_BV( PORTB1 );  
      }
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

static FILE stream;

////////////////////////////////////////////////////////////////////////////////////////////////////

static int uart0_putc( char c, FILE *stream )
{
   return uart.sendc( c );
}


////////////////////////////////////////////////////////////////////////////////////////////////////

void UARTBitBangTimer0::init() {
  PORTB  |= _BV( PORTB1 );  
  initializeTimer0For9600Hz();
  currentByte = 0xff;
  bitCounter = 8;

  fdev_setup_stream( &stream, uart0_putc, NULL, _FDEV_SETUP_WRITE );
  stdout = &stream;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int UARTBitBangTimer0::sendc( char c ) {
   cli();
   while( txBuffer.full() ) {
      sei();
      for( uint8_t i = 0; i < 100; ++i ) {
         _NOP();
      }
      // busy waiting
      cli();
   }
   txBuffer.push( c );
   // enable interrupt only if disabled
   if( !( TIMSK & _BV( OCIE0A ) ) ) {
     // TCNT0 = 0;
     TIFR |= _BV( OCF0A ); // clear interrupt flag
     TIMSK = _BV( OCIE0A ); 
   }
   sei();
   
   return 0;
}

