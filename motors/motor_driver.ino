/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   
#ifdef POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined L298_MOTOR_DRIVER
  void initMotorController() {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == LEFT) { 
      if      (reverse == 0) { analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if      (reverse == 0) { analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined MDD10A_MOTOR_DRIVER
  /* MDD10A Motor Driver Implementation - 4 Motors */
  void initMotorController() {
    // Motor 1 & 2 pins (Driver 1)
    pinMode(MOTOR1_PWM, OUTPUT);
    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR2_PWM, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);
    
    // Motor 3 & 4 pins (Driver 2)
    pinMode(MOTOR3_PWM, OUTPUT);
    pinMode(MOTOR3_DIR, OUTPUT);
    pinMode(MOTOR4_PWM, OUTPUT);
    pinMode(MOTOR4_DIR, OUTPUT);
    
    // Initialize all motors to stopped state
    analogWrite(MOTOR1_PWM, 0);
    analogWrite(MOTOR2_PWM, 0);
    analogWrite(MOTOR3_PWM, 0);
    analogWrite(MOTOR4_PWM, 0);
    digitalWrite(MOTOR1_DIR, LOW);
    digitalWrite(MOTOR2_DIR, LOW);
    digitalWrite(MOTOR3_DIR, LOW);
    digitalWrite(MOTOR4_DIR, LOW);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
    
    if (spd < 0) {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    // Handle 4 motors (0-3)
    switch(i) {
      case MOTOR1:
        digitalWrite(MOTOR1_DIR, reverse);
        analogWrite(MOTOR1_PWM, spd);
        break;
      case MOTOR2:
        digitalWrite(MOTOR2_DIR, reverse);
        analogWrite(MOTOR2_PWM, spd);
        break;
      case MOTOR3:
        digitalWrite(MOTOR3_DIR, reverse);
        analogWrite(MOTOR3_PWM, spd);
        break;
      case MOTOR4:
        digitalWrite(MOTOR4_DIR, reverse);
        analogWrite(MOTOR4_PWM, spd);
        break;
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Legacy compatibility: control left side (Motors 1&2) and right side (Motors 3&4)
    setMotorSpeed(MOTOR1, leftSpeed);
    setMotorSpeed(MOTOR2, leftSpeed);
    setMotorSpeed(MOTOR3, rightSpeed);
    setMotorSpeed(MOTOR4, rightSpeed);
  }
#else
  #error A motor driver must be selected!
#endif

#endif
