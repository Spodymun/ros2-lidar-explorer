/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  // Arduino Nano Every encoder pins
  // Using interrupt-capable pins for encoders
  #define LEFT_ENC_PIN_A 2    // Digital pin 2 (interrupt)
  #define LEFT_ENC_PIN_B 5    // Digital pin 5
  
  #define RIGHT_ENC_PIN_A 7   // Digital pin 7
  #define RIGHT_ENC_PIN_B 11  // Digital pin 11
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void initEncoders();
