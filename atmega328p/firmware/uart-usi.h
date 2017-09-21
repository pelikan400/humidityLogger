
#ifndef DE_EDBA_ARLINGTON_UART_USI_H
#define DE_EDBA_ARLINGTON_UART_USI_H

#define TRUE  1
#define FALSE 0

/// USI UART works only half-duplex 
struct {
  uint8_t 
  uint8_t busy = TRUE;
  uint8_t* 
} usiStatus;

void uartInitialize()
{
}

// interrupt driven receive bytes
void uartSendByte()
void uartSendPacket( uint8_t* data, uint8_t size )
{
  while( usiStatus.busy ) {
    sleep_mode();
  }
}

void uartReceivePacket( uint8_t* data, uint8_t maximumSize )
{
}

#endif
