#define FULL_CYCLE_TIME 500 // ms
#define SOUND_SPEED 0.0343

// pin definitions
const int trigger_pin = 9;
const int echo_pin = 10;
const int servo_pin = 11;

enum State {READ, PROCESS, WRITE};
enum Time {CURRENT, LAST, TIME_SIZE};

// function signatures
int get_cycle_time();
void read_sensor();
void process_data();
void write_output();

// global variables
int state = 0;
float distance_in_cm = 0;
float setpoint_in_cm = 0;
float error[TIME_SIZE] = {0,0};
float error_integral = 0;
float kp = 0, ki = 0, kd = 0;
float control_action = 0;

void setup() {
  pinMode(trigger_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(servo_pin, OUTPUT);
}

void loop() {
  unsigned long endCycleTime = millis() + get_cycle_time();

  switch(state) {
    case READ:
      read_sensor();
      state = PROCESS;
      break;
    case PROCESS:
      process_data();
      state = WRITE;
      break;
    case WRITE:
      write_output();
    default:
      state = READ;
  }

  delay(endCycleTime - millis());
}


int get_cycle_time() {
  int cycle_time = 0;

  switch(state) {
    case READ:
      return 150;
    case PROCESS:
      return 200;
    case WRITE:
      return 150;
    default:
      return 0;
  }
}

void update_params() {
  
}

void read_sensor() {
  // clear trigger pin
  digitalWrite(trigger_pin, LOW);
  delayMicroseconds(2);

  // generates tigger pulse of 10us
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);

  // get echo pulse duration
  long duration = pulseIn(echo_pin, HIGH);

  // transform duration into centimeters using: distance = (time * sound_speed) / 2
  distance_in_cm = (duration * SOUND_SPEED) / 2;
}

void process_data() {
  error[LAST] = error[CURRENT];
  error[CURRENT] = setpoint_in_cm - distance_in_cm;

  error_integral += error[CURRENT];
  float error_derivative = (error[CURRENT] - error[LAST])/FULL_CYCLE_TIME;

  float P = kp*error[CURRENT];
  float I = ki*error_integral;
  float D = kd*error_derivative;

  control_action = P+I+D;
}

void write_output() {
  analogWrite(servo_pin, control_action);
}
