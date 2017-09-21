///
//
//

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "rtc-atmega8.h"

volatile uint32_t globalTicks;

#define TIMER_FREQUENCY 50   // 50 Hz
#define TIMER1_PRESCALE 8
#define TICKS_PER_TIMER1_IRQ ( 1000 / TIMER_FREQUENCY )
#define OUTPUT_COMPARE_REGISTER_A ( F_CPU / TIMER1_PRESCALE / TIMER_FREQUENCY )


////////////////////////////////////////////////////////////////////////////////////////////////////

ISR( TIMER1_COMPA_vect ) 
{
   globalTicks += TICKS_PER_TIMER1_IRQ;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void initializeRealTimeClock()
{
   cli();
   DDRB |= _BV( DDB1 ); 
   TCCR1A = _BV( COM1A0 );
   // CTC Mode 4 and 8 Prescaler
   TCCR1B = (1 << WGM12 ) | ( 1 << CS11 );
   OCR1A = OUTPUT_COMPARE_REGISTER_A;
   TCNT1 = 0;
   TIMSK1 = (1 << OCIE1A );
   sei();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t getTicks()
{
   cli();
   uint32_t x = globalTicks;
   sei();
   return x;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void initLEDPort()
{
  PORTD &= ~ _BV( PORTD4 );
  DDRD |= _BV(DDD4 );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void blinkLED()
{
  PORTD = PORTD ^ ( 1 << PD4 );
}

// void main()
// {
//    uint32_t t0 = getTicks();
//    set_sleep_mode_idle();
//    while( 1 ) {
//       uint32_t t1 = getTicks() - t0;
//       if( t1 > ( 10 * 1000 ) ) {
//          PORTB = PORTB ^ ( 1 << PB6 );
//          t0 = t1;
//       }
//       sleep();
//    }
// }

