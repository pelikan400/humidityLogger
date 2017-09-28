#ifndef __TWI_H__
#define __TWI_H__

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

extern TWI twi;

#endif
