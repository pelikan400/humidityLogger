////////////////////////////////////////////////////////////////////////////////////////////////////
//
// 
//
//
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#define RING_BUFFER_SIZE 64
#define RING_BUFFER_MASK 0x3F

class RingBuffer {
public:
   char buffer[ RING_BUFFER_SIZE ];
   uint8_t writePointer;
   uint8_t readPointer;

   RingBuffer() {
      writePointer = 0;
      readPointer = 0;
   }

   inline bool empty() {
      return writePointer == readPointer;
   }

   inline bool full() {
     return ( ( writePointer + 1 ) & RING_BUFFER_MASK ) == readPointer; 
      // return ( writePointer + 1 ) % RING_BUFFER_SIZE == readPointer; 
   }

   inline void push( char c ) {
      if( full() ) {
         return;
      }
      buffer[ writePointer ] = c;
      writePointer = ( writePointer + 1 ) & RING_BUFFER_MASK;
      // writePointer = ( writePointer + 1 ) % RING_BUFFER_SIZE;
   }

   inline char pop() {
      char c = 0;
      if( !empty() ) {
         c = buffer[ readPointer ];
         readPointer = ( readPointer + 1 ) & RING_BUFFER_MASK;
         // readPointer = ( readPointer + 1 ) % RING_BUFFER_SIZE;
      }
      return c;
   }
};


#endif
