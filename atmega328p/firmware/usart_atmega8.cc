////////////////////////////////////////////////////////////////////////////////////////////////////
//
// 
//
//
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include "usart_atmega8.h"
#include "sleep.h"

#define MYUBBR 34 // 57600 kb/s
// #define MYUBBR 832   // 2400 kb/s
// #define MYUBBR 1666   // 1200  kb/s

////////////////////////////////////////////////////////////////////////////////////////////////////

USART usart;

static FILE stream;

////////////////////////////////////////////////////////////////////////////////////////////////////

static int usart0_putc( char c, FILE *stream )
{
   return usart.sendc( c );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

static void inline putHalfByte( uint8_t hb ) {
   if( hb > 9 ) {
      usart.sendc( 'a' - 10 + hb );
   }
   else {
     usart.sendc( '0' + hb );
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void putHex( uint8_t byte ) {
   usart.sendc( '0' );
   usart.sendc( 'x' );
   putHalfByte( ( byte >> 4 ) & 0x0f );
   putHalfByte( byte & 0x0f );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void USART::init() {
   // fixed speed of 57600 kbit/s
   cli();
   
   UBRR0H = (uint8_t) (MYUBBR >> 8);
   UBRR0L = (uint8_t) MYUBBR;

   // sampler 8x 
   UCSR0A = (1 << U2X0 );
      
   /* Enable receiver and transmitter */
   UCSR0B = _BV( RXCIE0 ) | _BV( RXEN0 ) | _BV( TXEN0 );
   
   /* Set frame format: 8data, 2stop bit */
   UCSR0C = _BV( USBS0 ) | ( 3 << UCSZ00 );

   fdev_setup_stream( &stream, usart0_putc, NULL, _FDEV_SETUP_WRITE );
   stdout = &stream;
    
   sei();
} 

////////////////////////////////////////////////////////////////////////////////////////////////////

ISR( USART_RX_vect )  {
   // PORTD ^= _BV( PORTD5 );    /* toggle D5 */ 
   char c = UDR0;
   usart.usartReceiveBuffer.push( c );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

ISR( USART_UDRE_vect ) {
   PORTD ^= _BV( PORTD5 );    /* toggle D5 */ 
   if( !usart.usartTransmitBuffer.empty() ) {
      UDR0 = usart.usartTransmitBuffer.pop();
   }
   else {
      UCSR0B = _BV( RXCIE0 ) | _BV( RXEN0 ) | _BV( TXEN0 );
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int USART::sendc( char c ) {
   cli();

   while( usartTransmitBuffer.full() ) {
     sei();
     sleepIdle();
     cli();
   }
   usartTransmitBuffer.push( c );
   UCSR0B = _BV( RXCIE0 ) | _BV( UDRIE0 ) | _BV( RXEN0 ) | _BV( TXEN0 );

   sei();
   
   return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

char USART::getc() {
   char c = 0;
   cli();
   c = usartReceiveBuffer.pop();
   sei();
   return c;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void USART::sendString( const char* p ) {
   while( *p != 0 ) {
      sendc( *p );
      ++p;
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void USART::sendHexByte( uint8_t byte ) {
   char txt[ 10 ];
   utoa( byte, txt, 16 );
   sendString( "0x" );
   sendString( txt );
   sendc( ' ' );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void USART::dumpHexBuffer( uint8_t * p, uint16_t size ) {
   for( uint16_t i = 0; i < size; i++ ) {
      sendHexByte( p[ i ] );
   }
}
