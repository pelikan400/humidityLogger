////////////////////////////////////////////////////////////////////////////////////////////////////
//
// 
//
//
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __TWI_ATMEGA328P_H__
#define __TWI_ATMEGA328P_H__

#define TWI_DIRECTION_SEND 0
#define TWI_DIRECTION_READ 1

#define TWI_STATUS_CODE_IDLE                            0x00
#define TWI_STATUS_CODE_START_TRANSMITTED               0x08
#define TWI_STATUS_CODE_REPEAT_START_TRANSMITTED        0x10
#define TWI_STATUS_CODE_SLA_W_TRANSMITTED_ACK           0x18
#define TWI_STATUS_CODE_SLA_W_TRANSMITTED_NOT_ACK       0x20
#define TWI_STATUS_CODE_DATA_TRANSMITTED_ACK            0x28
#define TWI_STATUS_CODE_DATA_TRANSMITTED_NOT_ACK        0x30
#define TWI_STATUS_CODE_ARBITRATION_LOST                0x38
#define TWI_STATUS_CODE_SLA_R_TRANSMITTED_ACK           0x40
#define TWI_STATUS_CODE_SLA_R_TRANSMITTED_NOT_ACK       0x48
#define TWI_STATUS_CODE_DATA_RECEIVED_ACK               0x50
#define TWI_STATUS_CODE_DATA_RECEIVED_NOT_ACK           0x58

#ifndef TWI_FREQ
#define TWI_FREQ 100000L
#endif


class TWI {
 public:
  volatile uint8_t   statusCode;
  volatile bool      busy;
  bool               interruptDriven;
  uint8_t            direction;
  uint8_t            destinationAddress;
  uint8_t  *         msg;
  uint16_t           msgSize;
  bool               keepLineBusy;
  uint16_t           counter;
  bool               debug;

 public:
  void init();
  bool sendMessage( uint8_t adr, uint8_t * msg, uint16_t  msgSize, bool keepLineBusy = false );
  bool receiveMessage( uint8_t adr, uint8_t * p, uint16_t msgSize, bool keepLineBusy = false );

  TWI();
};

extern TWI twi;

#endif
