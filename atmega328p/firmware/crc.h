////////////////////////////////////////////////////////////////////////////////////////////////////
//
// 
//
//
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __CRC_H__
#define __CRC_H__

#include <avr/io.h>

class CRC8 {
 private: 
  uint8_t polynom;
  uint8_t reg;

 public: 
  CRC8();
  void init();
  uint8_t calc( uint8_t b );
  uint8_t calc( uint8_t* p, uint16_t size );
};

#endif
