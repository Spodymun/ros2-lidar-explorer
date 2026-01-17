/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
#ifdef ARDUINO_ENC_COUNTER
  // Arduino Nano Every encoder pins - 4 encoders for 4 motors
  
  // Motor 1 (Left Front) Encoder
  #define ENC_MOTOR1_PIN_A 2    // Digital pin 2 (interrupt)
  #define ENC_MOTOR1_PIN_B 5    // Digital pin 5
  
  // Motor 2 (Left Rear) Encoder
  #define ENC_MOTOR2_PIN_A 0    // Digital pin 0 (interrupt)
  #define ENC_MOTOR2_PIN_B 14   // Digital pin 14
  
  // Motor 3 (Right Front) Encoder
  #define ENC_MOTOR3_PIN_A 7    // Digital pin 7 (interrupt)
  #define ENC_MOTOR3_PIN_B 11   // Digital pin 11
  
  // Motor 4 (Right Rear) Encoder
  #define ENC_MOTOR4_PIN_A 1    // Digital pin 1 (interrupt)
  #define ENC_MOTOR4_PIN_B 15   // Digital pin 15
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void initEncoders();
