/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <avr/sleep.h>

#include <util/delay.h>     /* for _delay_ms() */
#include <avr/eeprom.h>

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"

#include "RingBuffer.h"
#include "uart.h"
#include "twi.h"

#include <stdio.h>


#define TEST_WATCHDOG  0
#define TEST_USB 0
#define TEST_UART 0
#define TEST_TWI 1
#define DEBUG 0

// #include "oddebug.h"        /* This is also an example for using debug macros */

#define DELAY_CLOCKS 100000l

#if DEBUG 
#define LOG_DEBUG( x ) PUTS( x )
#else 
#define LOG_DEBUG( x ) 
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

#define I2C_DEVICE_ADDRESS_HTU21D    0x40
#define I2C_DEVICE_ADDRESS_AT24C32   0x50
#define I2C_DEVICE_ADDRESS_DC1307    0x68

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t writeI2C( uint8_t i2cAddress, uint8_t * buffer, uint8_t bufferSize ) {
  return twi.sendMessage( i2cAddress, buffer, bufferSize );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t writeI2CCommand( uint8_t i2cAddress, uint8_t * buffer, uint8_t bufferSize, uint8_t command ) {
  uint8_t commandBuffer[ 1 ];
  commandBuffer[ 0 ] = command;
  twi.sendMessage( i2cAddress, commandBuffer, 1 );
  return writeI2C( i2cAddress, buffer, bufferSize );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t writeI2CTwoByteCommand( uint8_t i2cAddress, uint8_t * buffer, uint8_t bufferSize, uint16_t command ) {
  uint8_t commandBuffer[ 2 ];
  commandBuffer[ 0 ] = command & 0xff;
  commandBuffer[ 1 ] = ( command >> 8 ) & 0xff;
  twi.sendMessage( i2cAddress, commandBuffer, 2 );
  return writeI2C( i2cAddress, buffer, bufferSize );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t readI2C( uint8_t i2cAddress, uint8_t * buffer, uint8_t bufferSize ) {
  return twi.receiveMessage( i2cAddress, buffer, bufferSize );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t readI2CCommand( uint8_t i2cAddress, uint8_t * buffer, uint8_t bufferSize, uint8_t command ) {
  uint8_t commandBuffer[ 1 ];
  commandBuffer[ 0 ] = command;
  twi.sendMessage( i2cAddress, commandBuffer, 1 );
  return readI2C( i2cAddress, buffer, bufferSize );
}

uint8_t readI2CTwoByteCommand( uint8_t i2cAddress, uint8_t * buffer, uint8_t bufferSize, uint16_t command ) {
  uint8_t commandBuffer[ 2 ];
  commandBuffer[ 0 ] = command & 0xff;
  commandBuffer[ 1 ] = ( command >> 8 ) & 0xff;
  twi.sendMessage( i2cAddress, commandBuffer, 2 );
  return readI2C( i2cAddress, buffer, bufferSize );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// #define I2C_DEVICE_ADDRESS_HTU21D    0x40
// #define I2C_DEVICE_ADDRESS_AT24C32   0x50
// #define I2C_DEVICE_ADDRESS_DC1307    0x68

#define HTU21D_TEMPERATURE  0xe3
#define HTU21D_HUMIDITY     0xe5

void testLoopHumidity() {
  uint8_t timeBuffer[ 8 ];
  uint8_t buffer[ 32 ];
  while( 1 ) {
    // first read actual time 
    LOG_DEBUG( "# Read time from DC1307" );
    readI2CCommand( I2C_DEVICE_ADDRESS_DC1307, timeBuffer, 8, 0 ); 

    LOG_DEBUG( "# Read EEPROM content" );
    for( uint16_t adr = 0; adr < 4096; adr += 32 ) {
      readI2CTwoByteCommand( I2C_DEVICE_ADDRESS_AT24C32, buffer, 32, adr ); 
    }

    LOG_DEBUG( "# Read temperature" );
    readI2CCommand( I2C_DEVICE_ADDRESS_HTU21D, timeBuffer, 8, HTU21D_TEMPERATURE ); 
    
    LOG_DEBUG( "# Read humidity" );
    readI2CCommand( I2C_DEVICE_ADDRESS_HTU21D, timeBuffer, 8, HTU21D_HUMIDITY ); 

    for( uint32_t delayCounter = 0; delayCounter < DELAY_CLOCKS;  ) {
      ++delayCounter;
    }
  }
}


/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM const uint8_t usbHidReportDescriptor[22] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x80,                    //   REPORT_COUNT (128)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};
/* Since we define only one feature report, we don't use report-IDs (which
 * would be the first byte of the report). The entire report consists of 128
 * opaque data bytes.
 */

/* The following variables store the status of the current data transfer */
static uchar    currentAddress;
static uchar    bytesRemaining;


////////////////////////////////////////////////////////////////////////////////////////////////////

/* ------------------------------------------------------------------------- */

/* usbFunctionRead() is called when the host requests a chunk of data from
 * the device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionRead( uchar *data, uchar len ) {
    if(len > bytesRemaining)
        len = bytesRemaining;
    eeprom_read_block(data, (uchar *)0 + currentAddress, len);
    currentAddress += len;
    bytesRemaining -= len;
    return len;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/* usbFunctionWrite() is called when the host sends a chunk of data to the
 * device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionWrite( uchar *data, uchar len ) {
    if(bytesRemaining == 0)
        return 1;               /* end of transfer */
    if(len > bytesRemaining)
        len = bytesRemaining;
    eeprom_write_block(data, (uchar *)0 + currentAddress, len);
    currentAddress += len;
    bytesRemaining -= len;
    return bytesRemaining == 0; /* return 1 if this was the last chunk */
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup( uchar data[8] ) {
   usbRequest_t    *rq = (usbRequest_t *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* HID class request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 128;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionRead() to obtain data */
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 128;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionWrite() to receive data from host */
        }
    }else{
        /* ignore vendor type requests, we don't use any */
    }
    return 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////////

ISR( WDT_vect ) {
  // do nothing, just wake up from sleep 
  // PORTB ^= _BV( PORTB1 );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void testWatchdog() {
   while( 1 ) {
      // wdt_reset();
      cli();
      wdt_enable( WDTO_8S );
      // enable interrupts, otherwise the chip will reset on WDT timeout
      WDTCR |= _BV( WDIE );
      set_sleep_mode( SLEEP_MODE_PWR_DOWN );
      sleep_enable();
      sei();
      sleep_cpu();
      sleep_disable();
      PORTB ^= _BV( PORTB1 );
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/* ------------------------------------------------------------------------- */

int main(void) {
   PORTB &= ~_BV( PORTB1 );  
   DDRB = _BV( DDB1 ) | _BV( DDB0 ) | _BV( DDB2 );  // make PB1 (LED), PB0, PB2 output
   PORTB ^= _BV( PORTB1 );  

#if TEST_WATCHDOG   // use watchdog to wake up from power down
   testWatchdog();
   // wdt_enable( WDTO_1S );
#endif
   
   wdt_disable();
    /* If you don't use the watchdog, replace the call above with a wdt_disable().
     * On newer devices, the status of the watchdog (on/off, period) is PRESERVED
     * OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
    // odDebugInit();
    // DBG1(0x00, 0, 0);       /* debug output: main starts */

   // TODO : find out if MCU is connected to USB

#if TEST_USB
   uint8_t i;

   usbInit();
   usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
   i = 0;
   while(--i){             /* fake USB disconnect for > 250 ms */
      wdt_reset();
      _delay_ms( 1 );
   }
   usbDeviceConnect();
   
   sei();
   bool hasUsbConnection = false;
   if( hasUsbConnection ) {
      // communicate with PC 
      for(;;){                /* main event loop */
         wdt_reset();
         PORTB ^= _BV( PORTB0 );  
         usbPoll();
         PORTB &= ~_BV( PORTB1 );  
      }
   }
#endif

   uart.init();
#if TEST_UART
   while( 1 ) {
      PUTS( "# Hello world" );
       for( uint32_t delayCounter = 0; delayCounter < 300000l;  ) {
          ++delayCounter;
       }
   }
#endif

#if TEST_TWI
   testLoopHumidity();
#endif
   
    return 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////////

int mainOld(void)
{
   PORTB &= ~_BV( PORTB1 );  
   DDRB = _BV( DDB1 ) ;  // make PB1 (LED) output
   PORTB ^= _BV( PORTB1 );  

    return 0;   /* never reached */
}



/* ------------------------------------------------------------------------- */

