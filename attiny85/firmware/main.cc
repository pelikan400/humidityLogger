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

#include <stdio.h>

// #include "oddebug.h"        /* This is also an example for using debug macros */

#define DELAY_CLOCKS 100000l

#define SCL _BV( PORTB2 )
#define SDA _BV( PORTB0 )
#define TWI_TIMING 15u

#define SDA_LOW      DDRB |= SDA
#define SDA_HIGH     DDRB &= ~SDA
#define SCL_LOW      DDRB |= SCL
#define SCL_HIGH     DDRB &= ~SCL

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

#define PUTS( x ) puts_P( PSTR( x ) )

////////////////////////////////////////////////////////////////////////////////////////////////////

// TWI using USI
class TWI {
public:
   uint8_t state;
   
public:
   TWI();
   void init();
   void sendStart();
   void sendStop();
   uint8_t sendMessage( uint8_t adr, uint8_t* msg, uint16_t size );
   uint8_t receiveMessage( uint8_t adr, uint8_t* msg, uint16_t size );
private:
   bool timeTWIBit( bool monitorSCL );
   uint8_t sendByte( uint8_t x );
   uint8_t receiveByte( uint8_t * r, uint8_t ack );
   uint8_t writeBit( uint8_t bit );
   uint8_t readBit();
};

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

////////////////////////////////////////////////////////////////////////////////////////////////////

#define I2C_DEVICE_ADDRESS_HTU21D    0x40
#define I2C_DEVICE_ADDRESS_AT24C32   0x50
#define I2C_DEVICE_ADDRESS_DC1307    0x68

uint8_t setTimeDS1307() {
   static bool firstTime = true;
   if( firstTime ) {
      uint8_t buffer[ 9 ] = { 0x00, 0x00,  0x51, 0x01, 0x07, 0x16, 0x09, 0x17, 0x00 };
      twi.sendMessage( I2C_DEVICE_ADDRESS_DC1307, buffer, 8 );
   }
   return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t readDS1307() {
   uint8_t buffer[ 8 ] = { 0, 0, 0, 0, 0, 0, 0, 0 };
   twi.sendMessage( I2C_DEVICE_ADDRESS_DC1307, buffer, 1 );
   twi.receiveMessage( I2C_DEVICE_ADDRESS_DC1307, buffer, 8 );
   return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

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


ISR( WDT_vect ) {
  // do nothing, just wake up from sleep 
  PORTB ^= _BV( PORTB1 );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

class USARTBitBangTimer0 {
public:
   uint8_t currentByte;
   uint8_t bitCounter;
   RingBuffer txBuffer;
   
public:
   USARTBitBangTimer0();
   void init();
   int sendc( char c );
};

USARTBitBangTimer0 usart;

////////////////////////////////////////////////////////////////////////////////////////////////////

USARTBitBangTimer0::USARTBitBangTimer0() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

static void initializeTimer0For9600Hz( ) {
  // initialize Timer0 
  cli();
  // halt the Timer0 and prescaler
  GTCCR = _BV( TSM );
  // WGM mode normal and set OC0A on compare match
  TCCR0A = _BV( WGM01 );
  // set prescaler to clk/8 
  TCCR0B = _BV( CS01 );
  TCNT0 = 0;
  // for 9600 baud
  OCR0A = 215; 
  OCR0B = 0;
  TIMSK = 0; 
  // TIMSK = _BV( OCIE0A ); 
  // start counter
  GTCCR = 0;
  sei();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

ISR( TIMER0_COMPA_vect ) {
   if( usart.bitCounter < 8 ) {
      if( usart.currentByte & 0x01 ) {
         // send 1
         PORTB  |= _BV( PORTB1 );  
      }
      else {
         // send 0
         PORTB &= ~_BV( PORTB1 );  
      }
      usart.currentByte >>= 1;
      ++usart.bitCounter;
   }
   else if( usart.bitCounter == 8 ) {
      // send stop bit
      PORTB  |= _BV( PORTB1 );  
      ++usart.bitCounter;
   }
   else {
      if( usart.txBuffer.empty() ) {
         // disable interrupts
         TIMSK = 0; 
      }
      else {
         // send start bit
         usart.currentByte = usart.txBuffer.pop();
         usart.bitCounter = 0;
         PORTB &= ~_BV( PORTB1 );  
      }
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

static FILE stream;

////////////////////////////////////////////////////////////////////////////////////////////////////

static int usart0_putc( char c, FILE *stream )
{
   return usart.sendc( c );
}


////////////////////////////////////////////////////////////////////////////////////////////////////

void USARTBitBangTimer0::init() {
  PORTB  |= _BV( PORTB1 );  
  initializeTimer0For9600Hz();
  currentByte = 0xff;
  bitCounter = 8;

  fdev_setup_stream( &stream, usart0_putc, NULL, _FDEV_SETUP_WRITE );
  stdout = &stream;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int USARTBitBangTimer0::sendc( char c ) {
   cli();
   while( txBuffer.full() ) {
      sei();
      for( uint8_t i = 0; i < TWI_TIMING; ++i ) {
         _NOP();
      }
      // busy waiting
      cli();
   }
   txBuffer.push( c );
   TIMSK = _BV( OCIE0A ); 
   sei();
   
   return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/* ------------------------------------------------------------------------- */

int main(void) {
   uint8_t i;

   PORTB &= ~_BV( PORTB1 );  
   DDRB = _BV( DDB1 ) | _BV( DDB0 ) | _BV( DDB2 );  // make PB1 (LED), PB0, PB2 output
   PORTB ^= _BV( PORTB1 );  

    wdt_enable( WDTO_1S );
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
    else {
       wdt_disable();

       usart.init();
       while( 1 ) {
          PUTS( "Hello world" );
       }
      // communicate with humidity sensor
      while( 0 ) {
	wdt_reset();
	wdt_enable( ( WDTO_8S | _BV( WDIE ) ) );
	cli();
	set_sleep_mode( SLEEP_MODE_PWR_DOWN );
	sleep_enable();
	sei();
	sleep_cpu();
	sleep_disable();
      }
    }
    
    return 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////////

int mainOld(void)
{
   PORTB &= ~_BV( PORTB1 );  
   DDRB = _BV( DDB1 ) ;  // make PB1 (LED) output
   PORTB ^= _BV( PORTB1 );  

   setTimeDS1307(); 
    /* insert your hardware initialization here */
    for(;;){
       readDS1307();
       for( uint32_t delayCounter = 0; delayCounter < DELAY_CLOCKS;  ) {
          ++delayCounter;
       }
    }
    
    return 0;   /* never reached */
}



/* ------------------------------------------------------------------------- */

