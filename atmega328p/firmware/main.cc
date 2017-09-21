
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#include "usart.h"
#include "twi.h"
#include "rtc-atmega8.h"
#include "sleep.h"
#include "crc.h"


char textBuffer[ 100 ];

////////////////////////////////////////////////////////////////////////////////////////////////////

void waitMilliseconds( uint32_t ms ) {
   uint32_t t0 = getTicks();
   uint32_t t1 = t0;
   while( t1 - t0 < ms ) {
      sleepIdle();
      t1 = getTicks();
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void scanTwiDevices() {
   uint8_t commandBuffer[ 2 ];
   PUTS( "Scan for I2C write ports" );
   for( uint8_t adr = 1; adr < 112; ++adr ) {
      bool success = twi.sendMessage( adr, commandBuffer, 0 );
      // usart.sendHexByte( adr );
      // PUTS( " scan" );
      if( success ) {
         usart.sendHexByte( adr );
         PUTS( " device found" );
      }
   }
   PUTS( "Scan for I2C read ports" );
   for( uint8_t adr = 1; adr < 112; ++adr ) {
      bool success = twi.receiveMessage( adr, commandBuffer, 0 );
      if( success ) {
         usart.sendHexByte( adr );
         PUTS( " device found" );
      }
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#define I2C_DEVICE_ADDRESS_HTU21D    0x40
#define I2C_DEVICE_ADDRESS_AT24C32   0x50
#define I2C_DEVICE_ADDRESS_DC1307    0x68

////////////////////////////////////////////////////////////////////////////////////////////////////

void getRTCTime() {
   uint8_t sendCommand[ 1 ] = { 0x00 };
   twi.sendMessage( I2C_DEVICE_ADDRESS_DC1307, sendCommand, 1 );
   uint8_t receiveResult[ 10 ];
   twi.receiveMessage( I2C_DEVICE_ADDRESS_DC1307, receiveResult, 8 );
   usart.dumpHexBuffer( receiveResult, 8 );
   PUTS( "DS1307 receive success " );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void getRTCTime_Original() {
   uint8_t sendCommand[ 1 ] = { 0x00 };
   if( twi.sendMessage( I2C_DEVICE_ADDRESS_DC1307, sendCommand, 1 ) ) {
      uint8_t receiveResult[ 10 ];
      if( twi.receiveMessage( I2C_DEVICE_ADDRESS_DC1307, receiveResult, 8 ) ) {
         usart.dumpHexBuffer( receiveResult, 8 );
         PUTS( "DS1307 receive success " );
      }
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint16_t getTemperature() {
   CRC8 crc;
   int32_t temperature = 0;
   uint8_t sendCommand[ 1 ] = { 0xE3 };
   // PUTS( "HTU21D get temperature" );
   if( twi.sendMessage( I2C_DEVICE_ADDRESS_HTU21D, sendCommand, 1 ) ) {
      waitMilliseconds( 100 );
      uint8_t receiveResult[ 4 ];
      if( twi.receiveMessage( I2C_DEVICE_ADDRESS_HTU21D, receiveResult, 3 ) ) {
         usart.dumpHexBuffer( receiveResult, 3 );
         PUTS( "HTU21D temperature receive success " );

	 uint8_t crcSum = crc.calc( receiveResult, 3 );

         temperature = ( receiveResult[ 0 ] << 8 ) + receiveResult[ 1 ];
         temperature = temperature * 1757;
         temperature = temperature >> 16;
         temperature = temperature - 468;
         
         char txt[ 10 ];
         itoa( (int16_t) temperature, txt, 10 );
         PUTS( "Temperature " );
         puts( txt );

         if( crcSum != 0 ) {
            utoa( (uint16_t) crcSum, txt, 16 );
            PUTS( "CRC: " );
            usart.sendHexByte( crcSum );
            PUTS( " " );
         }
      }
   }
   else {
      PUTS( "Error" );
   }
   PUTS( " " );

   return temperature;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint16_t getHumidity() {
   CRC8 crc;
   int32_t humidity  = 0;
   uint8_t sendCommand[ 1 ] = { 0xE5 };
   if( twi.sendMessage( I2C_DEVICE_ADDRESS_HTU21D, sendCommand, 1 ) ) {
      // PUTS( "HTU21D get humidity" );
      waitMilliseconds( 100 );
      uint8_t receiveResult[ 4 ];
      if( twi.receiveMessage( I2C_DEVICE_ADDRESS_HTU21D, receiveResult, 3 ) ) {
         usart.dumpHexBuffer( receiveResult, 3 );
         PUTS( "HTU21D humidity receive success " );
         
	 uint8_t crcSum = crc.calc( receiveResult, 3 );

         humidity  = ( (uint16_t)receiveResult[ 0 ] << 8 ) + receiveResult[ 1 ];
         humidity = humidity * 1250l;
         humidity = humidity >> 16;
         humidity = humidity - 60;
         
         char txt[ 10 ];
         itoa( (int16_t) humidity, txt, 10 );
         PUTS( "Humidity: " );
         puts( txt );

         if( crcSum != 0 ) {
            utoa( (uint16_t) crcSum, txt, 16 );
            PUTS( "CRC: " );
            usart.sendHexByte( crcSum );
            PUTS( " " );
         }
      }
   }
   PUTS( "\n" );
   
   return humidity;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void wallClock() {
  // char buffer[ 10 ];
   for( int hours = 0; ; ++hours ){
      for( int minutes = 0; minutes < 60; ++minutes ) {
         for( int seconds = 0; seconds < 60; ++seconds ) {
            // utoa( hours, buffer, 10 );
            // puts( buffer );
            // PUTS( ":" );
            // 
            // utoa( minutes, buffer, 10 );
            // PUTS( buffer );
            // PUTS( ":" );
            // 
            // utoa( seconds, buffer, 10 );
            // PUTS( buffer );
            // PUTS( "\n" );
            waitMilliseconds( 1000l );
            getTemperature();
            waitMilliseconds( 1000l );
            getHumidity();
            waitMilliseconds( 1000l );
            getRTCTime();
            // char x = usart.getc();
            // while( x != 0 ) {
            //    switch( x ) {
            //    case ' ':
            //       seconds = 0;
            //       break;
            //    case 'w' :
            //       hours++;
            //       break;
            //    case 's' :
            //       hours--;
            //       break;
            //    case 'a' :
            //       minutes--;
            //       break;
            //    case 'f' :
            //       minutes++;
            //       break;
            //    }
            //    x = usart.getc();
            // }
         }
      }
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void)
{
   PORTD |= _BV( PORTD4 ) | _BV( PORTD5 ) ;  
   DDRD = _BV( DDD4 ) | _BV( DDD5 );  // make D4 (LED) and D5 output
   PORTD |= _BV( PORTD4 ) | _BV( PORTD5 ) ;  

   // char longBuffer[20];
   usart.init();
   twi.init();
   initializeRealTimeClock();

   scanTwiDevices();
   wallClock();
   // uint32_t counter = 0;
   // for(;;){
   //    // uint8_t i;
   //    // for(i = 0; i < 10; i++){
   //    //     _delay_ms(30);  /* max is 262.14 ms / F_CPU in MHz */
   //    // }
   //    // PORTD ^= 1 << 4;    /* toggle the LED */
   //    PUTS( "Hello world! " );
   //    ltoa( counter++, longBuffer, 10 );
   //    ltoa( getTicks(), longBuffer, 10 );
   //    waitMilliseconds( 1000l );
   //    puts( longBuffer );
   //    PUTS( "\n" );
   // }
   
   return 0;               /* never reached */
}
