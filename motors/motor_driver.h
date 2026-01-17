/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#elif defined MDD10A_MOTOR_DRIVER
  // MDD10A Motor Driver Pin Configuration for Arduino Nano Every
  // PWM pins for speed control
  #define LEFT_MOTOR_PWM   3    // PWM1A - Left motor speed
  #define RIGHT_MOTOR_PWM  6    // PWM1B - Right motor speed
  // Direction pins
  #define LEFT_MOTOR_DIR   4    // DIR1 - Left motor direction
  #define RIGHT_MOTOR_DIR  8    // DIR2 - Right motor direction
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
