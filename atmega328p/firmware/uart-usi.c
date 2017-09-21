#include <avr/io.h>

// Packet format : start byte, addr, size, msg, crc-16 (2 bytes)
#ifndef byte
typedef unsigned char byte;
#endif

bool check_crc16( const byte * message )
{
  return true;
}

ISR( USI__vector )
{
}

void main()
{
}
