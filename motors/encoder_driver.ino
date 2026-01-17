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
  // 4 Encoder positions for 4 motors
  volatile long motor1_enc_pos = 0L;
  volatile long motor2_enc_pos = 0L;
  volatile long motor3_enc_pos = 0L;
  volatile long motor4_enc_pos = 0L;
  
  // Encoder interrupt handlers for 4 motors
  void motor1EncoderEvent() {
    if (digitalRead(ENC_MOTOR1_PIN_A) == digitalRead(ENC_MOTOR1_PIN_B)) {
      motor1_enc_pos++;
    } else {
      motor1_enc_pos--;
    }
  }
  
  void motor2EncoderEvent() {
    if (digitalRead(ENC_MOTOR2_PIN_A) == digitalRead(ENC_MOTOR2_PIN_B)) {
      motor2_enc_pos++;
    } else {
      motor2_enc_pos--;
    }
  }
  
  void motor3EncoderEvent() {
    if (digitalRead(ENC_MOTOR3_PIN_A) == digitalRead(ENC_MOTOR3_PIN_B)) {
      motor3_enc_pos++;
    } else {
      motor3_enc_pos--;
    }
  }
  
  void motor4EncoderEvent() {
    if (digitalRead(ENC_MOTOR4_PIN_A) == digitalRead(ENC_MOTOR4_PIN_B)) {
      motor4_enc_pos++;
    } else {
      motor4_enc_pos--;
    }
  }
  
  /* Initialize all 4 encoders */
  void initEncoders() {
    // Motor 1 encoder pins
    pinMode(ENC_MOTOR1_PIN_A, INPUT_PULLUP);
    pinMode(ENC_MOTOR1_PIN_B, INPUT_PULLUP);
    
    // Motor 2 encoder pins
    pinMode(ENC_MOTOR2_PIN_A, INPUT_PULLUP);
    pinMode(ENC_MOTOR2_PIN_B, INPUT_PULLUP);
    
    // Motor 3 encoder pins
    pinMode(ENC_MOTOR3_PIN_A, INPUT_PULLUP);
    pinMode(ENC_MOTOR3_PIN_B, INPUT_PULLUP);
    
    // Motor 4 encoder pins
    pinMode(ENC_MOTOR4_PIN_A, INPUT_PULLUP);
    pinMode(ENC_MOTOR4_PIN_B, INPUT_PULLUP);
    
    // Attach interrupts for all 4 motor encoders
    attachInterrupt(digitalPinToInterrupt(ENC_MOTOR1_PIN_A), motor1EncoderEvent, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_MOTOR2_PIN_A), motor2EncoderEvent, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_MOTOR3_PIN_A), motor3EncoderEvent, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_MOTOR4_PIN_A), motor4EncoderEvent, CHANGE);
  }
  
  /* Wrap the encoder reading function - supports 0-3 for 4 motors */
  long readEncoder(int i) {
    switch(i) {
      case MOTOR1: return motor1_enc_pos;
      case MOTOR2: return motor2_enc_pos;
      case MOTOR3: return motor3_enc_pos;
      case MOTOR4: return motor4_enc_pos;
      default: return 0L;
    }
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    switch(i) {
      case MOTOR1: motor1_enc_pos = 0L; break;
      case MOTOR2: motor2_enc_pos = 0L; break;
      case MOTOR3: motor3_enc_pos = 0L; break;
      case MOTOR4: motor4_enc_pos = 0L; break;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function for all 4 motors */
void resetEncoders() {
  resetEncoder(MOTOR1);
  resetEncoder(MOTOR2);
  resetEncoder(MOTOR3);
  resetEncoder(MOTOR4);
}

#endif
