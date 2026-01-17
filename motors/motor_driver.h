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
  // 4-Motor configuration with 2x MDD10A drivers
  
  // Motor Driver 1 (MDD10A) - LEFT SIDE
  #define MOTOR1_PWM   3    // Motor 1 (Left Front) - speed control
  #define MOTOR1_DIR   4    // Motor 1 - direction control
  #define MOTOR2_PWM   9    // Motor 2 (Left Rear) - speed control
  #define MOTOR2_DIR   12   // Motor 2 - direction control
  
  // Motor Driver 2 (MDD10A) - RIGHT SIDE
  #define MOTOR3_PWM   6    // Motor 3 (Right Front) - speed control
  #define MOTOR3_DIR   8    // Motor 3 - direction control
  #define MOTOR4_PWM   10   // Motor 4 (Right Rear) - speed control
  #define MOTOR4_DIR   13   // Motor 4 - direction control
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
