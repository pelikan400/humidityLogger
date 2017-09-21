
#ifndef DE_EDBA_ARLINGTON_UART_H
#define DE_EDBA_ARLINGTON_UART_H

#include <stdint.h>

void uartInitialize( void );
void uartSendByte( void );
void uartSendPacket( uint8_t* data, uint8_t size );
void uartReceivePacket( uint8_t* data, uint8_t maximumSize );

#endif
