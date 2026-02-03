/*****
Motor Bridge with 4 Motors + 4 Encoders and PID Control for Arduino Nano Every + 2x MDD10A
Supports LIDAR Explorer ROS2 Interface - COMPLETE IMPLEMENTATION
Full 4-motor configuration with independent encoder tracking
*****/

#define USE_BASE
#define ARDUINO_ENC_COUNTER
#define MDD10A_MOTOR_DRIVER
#define BAUDRATE     57600
#define MAX_PWM      255

// Motor Driver 1 (MDD10A) - LEFT SIDE
#define MOTOR1_PWM   3
#define MOTOR1_DIR   4
#define MOTOR2_PWM   9
#define MOTOR2_DIR   12

// Motor Driver 2 (MDD10A) - RIGHT SIDE
#define MOTOR3_PWM   6
#define MOTOR3_DIR   8
#define MOTOR4_PWM   10
#define MOTOR4_DIR   13

// Encoder Pins (4 total)
#define ENC_MOTOR1_PIN_A   2
#define ENC_MOTOR1_PIN_B   5
#define ENC_MOTOR2_PIN_A   16
#define ENC_MOTOR2_PIN_B   14
#define ENC_MOTOR3_PIN_A   7
#define ENC_MOTOR3_PIN_B   11
#define ENC_MOTOR4_PIN_A   17
#define ENC_MOTOR4_PIN_B   15

#define MOTOR1 0
#define MOTOR2 1
#define MOTOR3 2
#define MOTOR4 3

#define READ_ENCODERS     'e'
#define READ_MOTOR_DATA   'f'
#define MOTOR_SPEEDS      'm'   // legacy: direct PWM (left,right)
#define MOTOR_SPEED_TICKS 'v'   // NEW: ticks/frame (left,right)
#define RESET_ENCODERS    'r'
#define GET_BAUDRATE      'b'

// ============ GLOBAL VARIABLES ============
volatile long motor1_enc_pos = 0L;
volatile long motor2_enc_pos = 0L;
volatile long motor3_enc_pos = 0L;
volatile long motor4_enc_pos = 0L;

const int motor_pwm_pins[4] = {MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM};
const int motor_dir_pins[4] = {MOTOR1_DIR, MOTOR2_DIR, MOTOR3_DIR, MOTOR4_DIR};

struct SetPointInfo {
  int  TargetTicksPerFrame;
  long Encoder;
  long PrevEnc;
  int  output;
  int  PrevInput;
  int  ITerm;   // (unused currently)
};

SetPointInfo motor1PID, motor2PID, motor3PID, motor4PID;

// NOTE: This is really PD + bias (Ko). Keeping your values, but making "stop" truly stop.
int Kp = 30, Kd = 15, Ki = 0, Ko = 80;

char cmd, argv1[16], argv2[16];
long arg1, arg2;
int parsing_state = 0;  // 0=cmd, 1=arg1, 2=arg2

// ============ MOTOR CONTROL ============
void initMotorController() {
  for (int i = 0; i < 4; i++) {
    pinMode(motor_pwm_pins[i], OUTPUT);
    pinMode(motor_dir_pins[i], OUTPUT);
    analogWrite(motor_pwm_pins[i], 0);
    digitalWrite(motor_dir_pins[i], LOW);
  }
}

void setMotorSpeed(int motor_id, int spd) {
  unsigned char reverse = 0;
  if (spd < 0) {
    spd = -spd;
    reverse = 1;
  }
  if (spd > MAX_PWM) spd = MAX_PWM;
  digitalWrite(motor_dir_pins[motor_id], reverse);
  analogWrite(motor_pwm_pins[motor_id], spd);
}

void stopAllMotors() {
  setMotorSpeed(MOTOR1, 0);
  setMotorSpeed(MOTOR2, 0);
  setMotorSpeed(MOTOR3, 0);
  setMotorSpeed(MOTOR4, 0);
}

// ============ PID CONTROL ============
void doPID(SetPointInfo * p) {
  // ticks during last frame
  int input = (int)(p->Encoder - p->PrevEnc);
  p->PrevEnc = p->Encoder;

  // Hard stop if target is zero (prevents Ko bias from moving robot)
  if (p->TargetTicksPerFrame == 0) {
    p->output = 0;
    p->PrevInput = 0;
    return;
  }

  long Perror = (long)p->TargetTicksPerFrame - (long)input;
  long Derror = (long)Kd * (Perror - (long)p->PrevInput);

  long output = (long)Ko + (long)Kp * Perror + Derror;

  p->PrevInput = (int)Perror;

  // clamp to [-MAX_PWM, MAX_PWM]
  if (output > MAX_PWM) output = MAX_PWM;
  if (output < -MAX_PWM) output = -MAX_PWM;

  p->output = (int)output;
}

void resetPID() {
  motor1PID.TargetTicksPerFrame = motor1PID.output = 0;
  motor2PID.TargetTicksPerFrame = motor2PID.output = 0;
  motor3PID.TargetTicksPerFrame = motor3PID.output = 0;
  motor4PID.TargetTicksPerFrame = motor4PID.output = 0;

  motor1PID.PrevEnc = motor2PID.PrevEnc = motor3PID.PrevEnc = motor4PID.PrevEnc = 0;
  motor1PID.PrevInput = motor2PID.PrevInput = motor3PID.PrevInput = motor4PID.PrevInput = 0;
}

void updatePID() {
  motor1PID.Encoder = readEncoder(MOTOR1);
  motor2PID.Encoder = readEncoder(MOTOR2);
  motor3PID.Encoder = readEncoder(MOTOR3);
  motor4PID.Encoder = readEncoder(MOTOR4);

  doPID(&motor1PID);
  doPID(&motor2PID);
  doPID(&motor3PID);
  doPID(&motor4PID);

  setMotorSpeed(MOTOR1, motor1PID.output);
  setMotorSpeed(MOTOR2, motor2PID.output);
  setMotorSpeed(MOTOR3, motor3PID.output);
  setMotorSpeed(MOTOR4, motor4PID.output);
}

bool anyTargetNonZero() {
  return (motor1PID.TargetTicksPerFrame != 0 ||
          motor2PID.TargetTicksPerFrame != 0 ||
          motor3PID.TargetTicksPerFrame != 0 ||
          motor4PID.TargetTicksPerFrame != 0);
}

// ============ ENCODER HANDLING ============
void motor1EncoderEvent() { motor1_enc_pos += (digitalRead(ENC_MOTOR1_PIN_A) == digitalRead(ENC_MOTOR1_PIN_B)) ? 1 : -1; }
void motor2EncoderEvent() { motor2_enc_pos += (digitalRead(ENC_MOTOR2_PIN_A) == digitalRead(ENC_MOTOR2_PIN_B)) ? 1 : -1; }
void motor3EncoderEvent() { motor3_enc_pos += (digitalRead(ENC_MOTOR3_PIN_A) == digitalRead(ENC_MOTOR3_PIN_B)) ? 1 : -1; }
void motor4EncoderEvent() { motor4_enc_pos += (digitalRead(ENC_MOTOR4_PIN_A) == digitalRead(ENC_MOTOR4_PIN_B)) ? 1 : -1; }

void initEncoders() {
  pinMode(ENC_MOTOR1_PIN_A, INPUT_PULLUP);
  pinMode(ENC_MOTOR1_PIN_B, INPUT_PULLUP);
  pinMode(ENC_MOTOR2_PIN_A, INPUT_PULLUP);
  pinMode(ENC_MOTOR2_PIN_B, INPUT_PULLUP);
  pinMode(ENC_MOTOR3_PIN_A, INPUT_PULLUP);
  pinMode(ENC_MOTOR3_PIN_B, INPUT_PULLUP);
  pinMode(ENC_MOTOR4_PIN_A, INPUT_PULLUP);
  pinMode(ENC_MOTOR4_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_MOTOR1_PIN_A), motor1EncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_MOTOR2_PIN_A), motor2EncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_MOTOR3_PIN_A), motor3EncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_MOTOR4_PIN_A), motor4EncoderEvent, CHANGE);
}

long readEncoder(int motor_id) {
  if (motor_id == MOTOR1) return motor1_enc_pos;
  else if (motor_id == MOTOR2) return motor2_enc_pos;
  else if (motor_id == MOTOR3) return motor3_enc_pos;
  else return motor4_enc_pos;
}

void resetEncoders() {
  motor1_enc_pos = motor2_enc_pos = motor3_enc_pos = motor4_enc_pos = 0;
}

// ============ SETUP & LOOP ============
void setup() {
  Serial.begin(BAUDRATE);
  initEncoders();
  initMotorController();
  resetEncoders();
  resetPID();
  stopAllMotors();
}

void loop() {
  // -------- Serial command parser --------
  while (Serial.available() > 0) {
    char chr = Serial.read();

    if (chr == '\r' || chr == '\n') {
      // end of command
      if (parsing_state >= 2) arg2 = atoi(argv2);

      if (cmd == GET_BAUDRATE) {
        Serial.println(BAUDRATE);
      }
      else if (cmd == READ_ENCODERS) {
        Serial.print(readEncoder(MOTOR1)); Serial.print(" ");
        Serial.print(readEncoder(MOTOR2)); Serial.print(" ");
        Serial.print(readEncoder(MOTOR3)); Serial.print(" ");
        Serial.println(readEncoder(MOTOR4));
      }
      else if (cmd == READ_MOTOR_DATA) {
        Serial.print(readEncoder(MOTOR1)); Serial.print(",");
        Serial.print(readEncoder(MOTOR2)); Serial.print(",");
        Serial.print(readEncoder(MOTOR3)); Serial.print(",");
        Serial.print(readEncoder(MOTOR4)); Serial.print(",");
        Serial.print(motor1PID.output); Serial.print(",");
        Serial.print(motor2PID.output); Serial.print(",");
        Serial.print(motor3PID.output); Serial.print(",");
        Serial.println(motor4PID.output);
      }
      else if (cmd == RESET_ENCODERS) {
        resetEncoders();
        resetPID();
        stopAllMotors();
        Serial.println("OK");
      }
      else if (cmd == MOTOR_SPEEDS) {
        // legacy direct PWM: arg1 = left PWM, arg2 = right PWM
        setMotorSpeed(MOTOR1, (int)arg1);
        setMotorSpeed(MOTOR2, (int)arg1);
        setMotorSpeed(MOTOR3, (int)arg2);
        setMotorSpeed(MOTOR4, (int)arg2);
        Serial.println("OK");
      }
      else if (cmd == MOTOR_SPEED_TICKS) {
        // NEW: closed-loop speed targets in ticks/frame
        motor1PID.TargetTicksPerFrame = (int)arg1;
        motor2PID.TargetTicksPerFrame = (int)arg1;
        motor3PID.TargetTicksPerFrame = (int)arg2;
        motor4PID.TargetTicksPerFrame = (int)arg2;

        Serial.print("GOT_V targets L=");
        Serial.print(motor1PID.TargetTicksPerFrame);
        Serial.print(" R=");
        Serial.println(motor3PID.TargetTicksPerFrame);
      }

      // reset parser state
      parsing_state = 0;
      cmd = 0;
      arg1 = 0;
      arg2 = 0;
      memset(argv1, 0, 16);
      memset(argv2, 0, 16);
    }
    else if (chr == ' ') {
      if (parsing_state == 0) {
        parsing_state = 1;
        arg1 = 0;
      } else if (parsing_state == 1) {
        arg1 = atoi(argv1);
        parsing_state = 2;
        memset(argv1, 0, 16);
      }
    }
    else if (parsing_state == 0) {
      cmd = chr;
    }
    else if (parsing_state == 1) {
      size_t len = strlen(argv1);
      if (len < 15) argv1[len] = chr;
    }
    else if (parsing_state == 2) {
      size_t len = strlen(argv2);
      if (len < 15) argv2[len] = chr;
    }
  }

  // -------- Stable 30Hz PID update --------
  static unsigned long lastPID = 0;
  unsigned long now = millis();

  if (now - lastPID >= 33) {  // ~30Hz
    lastPID = now;

    if (anyTargetNonZero()) {
      updatePID();
    } else {
      stopAllMotors();
    }
  }
}
