/** Copyright (c) 2009 John Myers
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 
*/
/** 
 *  \file   twi_demo.c 
 *  \brief  Demo App
 *
 *  \author John Myers
 *  \version 0.02
 */
#include <stdio.h>
//#include <stdlib.h>
#include <avr/pgmspace.h>
//#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "i2c.h"
#include "rbuff.h"


/*
 *  Function Prototype Declarations
 */
/// Searches for and prints the address of I2C devices found on the bus
static void display_i2c_devices (void);
static void setup_ds75_sensor (void); ///< Configures the DS75
static int usart0_putc (char c, FILE *stream);
static uint8_t usart0_receive(void);
/* 
 *  Variables
 */   
uint8_t I2cRxRing[I2C_RX_RING_BUFFER_SIZE] = {0x55,0xAA,0x55};
static FILE stream;

/*===============================================================*
                            main()
 *===============================================================*/
int main (void) __attribute__((OS_main));
int main (void)
{
    uint8_t DS75temp[] = {0x90,0x00};
    uint8_t EEpromWrite[] = {0xA8,0x00,0x00,0xA5};
    uint8_t EEpromRead;
    uint16_t RawTemp;


   PORTD |= _BV( PORTD4 ) | _BV( PORTD5 ) ;  
   DDRD = _BV( DDD4 ) | _BV( DDD5 );  // make D4 (LED) and D5 output
   PORTD |= _BV( PORTD4 ) | _BV( PORTD5 ) ;  

/*
 * Initialize the UART 
 */
    UBRR0H = 0x00;      //-38400 Baud 8n1
    UBRR0L = 0x0C;
    UCSR0B = _BV(TXEN0)|_BV(RXEN0);
    UCSR0C = _BV(UCSZ01)|_BV(UCSZ00);

    fdev_setup_stream( &stream, usart0_putc, NULL, _FDEV_SETUP_WRITE );
    stdout = &stream;
    stderr = &stream;
    
/*
 * Setup the I2C port
 */
    i2c_init_(0x45);                        //- F_TWI=100KHz
    ring_buff_init (&I2cRxBuff,I2cRxRing);  //- Slave-mode RX/TX
    i2c_set_address_(0x5E);                 //- 8-bit address
    i2c_enable_gc_();
/***
 ***/    
    sei();                      //-Enable Interrupts

    puts_P (PSTR("Hello World\n"));
  
    display_i2c_devices ();     //- Find and display I2C devices
    
    setup_ds75_sensor ();       //- Initialize digital temp sensor
 

/*  
 *  Main task loop  
 */
    puts_P (PSTR("Entering Main Loop\n"));
    i2c_write(DS75temp, sizeof DS75temp);   //-Point to Temperature register

    while (1) 
    {
        //-Press spacebar to continue
        while (0x20 != usart0_receive ());
    /*
     * Read EEPROM
     */
        //-Point to memory location
        i2c_write(&EEpromWrite, sizeof EEpromWrite-1);
        //-Read memory location
        if (i2c_read (0xA8, &EEpromRead, 1))
            puts_P (PSTR("EEPROM read failed\n"));
        else
            printf ("EEPROM value: 0x%x \n", EEpromRead);



        //-Press spacebar to continue
        while (0x20 != usart0_receive ());
    /*
     * Write EEPROM
     */
        if (i2c_write(&EEpromWrite, sizeof EEpromWrite))
             puts_P (PSTR("EEPROM write failed\n"));
        else
            puts_P (PSTR("EEPROM write succesful\n"));

        if (EEpromWrite[2] >= 0x40)
        {
            EEpromWrite[2] = 0x00;
            EEpromWrite[3] = 0xA5;
        }
        else
        {
            ++EEpromWrite[2];
            --EEpromWrite[3];
        }


        //-Press spacebar to continue
        while (0x20 != usart0_receive ());
    /*
     * Read Temperature sensor
     */
        i2c_read(0x90, DS75temp, sizeof DS75temp);
        RawTemp = DS75temp[1];
        RawTemp |= (DS75temp[0]<<8);
        printf ("Temperature: 0x%x \n", RawTemp );

    }
////////////////
}//END OF MAIN


/*================================================================*
                        display_i2c_devices ()
 *================================================================*/
static void display_i2c_devices (void)
{
    register uint8_t idx;
    
    if ( i2c_dev_search (0x90, I2C_DEV_ARRAY_SIZE))
        puts_P (PSTR("I2C: Search Error"));
    else
    {
        puts_P (PSTR("Found I2C device(s) at: ")); 
        if (I2cDevArray[0])
        {
            putchar ('\t');
            for (idx=0; idx < I2C_DEV_ARRAY_SIZE; ++idx)
            {
                if ( !(I2cDevArray[idx]) )
                    break;
                printf ("<%#x> ",I2cDevArray[idx]);     
            }
            puts_P (PSTR("*"));
        } 
        else
            puts_P (PSTR("<none>\n")); 
    }

}

/*================================================================*
                        i2c_get_address ()
                <General Call addressing feature>    
    The I2C ISR uses this function for getting the hardware address
    of this device. `TWAR` needs to be set inside this function.
    The return value is not used by the ISR and is USER definable.
 *================================================================*/
//uint8_t i2c_get_address ()
//{
//   return 1;
//}

/*================================================================*
                        setup_ds75_sensor ()
 *================================================================*/
static void setup_ds75_sensor ()
{
    uint8_t DS75Cfg[] = {0x90,0x01,0x7A};  //-12bit,intrpt,O.S(act_low)
    uint8_t DS75Thyst[] = {0x90,0x02,0x4D,0x20};//-77.125c 
    uint8_t DS75Tos[] = {0x90,0x03,0x50,0x00};//-80c
    uint8_t tmp[2] = {0x90,0x00};
    uint8_t temp[] = {0x90,0x00,0x00};

    puts_P (PSTR("DS75:"));
    if ( i2c_write (DS75Cfg, sizeof DS75Cfg) )
        puts_P (PSTR("\tConfig failed"));
    else
    {
        _delay_ms (6);
        if ( i2c_write (DS75Thyst, sizeof DS75Thyst) )
            puts_P (PSTR("\tThyst failed")); 
        else 
            _delay_ms (6);     
        if ( i2c_write (DS75Tos, sizeof DS75Tos) )
            puts_P (PSTR("\tT-os failed"));
        else
            _delay_ms (6); 
        if ( i2c_write (tmp, sizeof tmp) )
            puts_P (PSTR("\tWrite failed"));
        else 
            _delay_ms (6);
        if ( i2c_read ( I2C_ADDRESS_IN_ARRAY, temp, sizeof temp) )
            puts_P (PSTR("\ttemperature read failed"));
        else
            puts_P (PSTR("\tConfig successfull"));
    }
}

/*================================================================*
 *================================================================*/
static int usart0_putc (char c, FILE *stream)
{
   return 0;
    if (c == '\n')
    {
        loop_until_bit_is_set(UCSR0A, UDRE0);
        UDR0 = '\r';
    }
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

/*================================================================*
 *================================================================*/
static uint8_t usart0_receive(void)
{
   return 0;
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}

/*~~END OF FILE~~*/
