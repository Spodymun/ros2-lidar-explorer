/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  
  // Encoder interrupt handlers for Arduino Nano Every
  // Left encoder on pins 2 and 3
  void leftEncoderEvent() {
    if (digitalRead(LEFT_ENC_PIN_A) == digitalRead(LEFT_ENC_PIN_B)) {
      left_enc_pos++;
    } else {
      left_enc_pos--;
    }
  }
  
  // Right encoder on pins 4 and 11
  void rightEncoderEvent() {
    if (digitalRead(RIGHT_ENC_PIN_A) == digitalRead(RIGHT_ENC_PIN_B)) {
      right_enc_pos++;
    } else {
      right_enc_pos--;
    }
  }
  
  /* Initialize encoders */
  void initEncoders() {
    pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
    
    // Attach interrupts - Arduino Nano Every supports interrupts on most pins
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderEvent, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), rightEncoderEvent, CHANGE);
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif
