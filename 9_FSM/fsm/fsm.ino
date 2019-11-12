#include "encoders.h"
#include "pid.h"
#include "line_sensors.h"

// Motor pins
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define DIR_FWD LOW
#define DIR_BKD HIGH

// Buzzer pin
#define BUZZER_PIN 6

// Line sensor pins
#define LINE_LEFT_PIN   A4
#define LINE_CENTRE_PIN A3
#define LINE_RIGHT_PIN  A2

// Constants
#define NO_OF_BEATS 25;
#define BAUD_RATE 9600

// PID Values
// #define kp 0.11
#define kp 0.15
#define ki 0.0
#define kd 0.0

// States
#define STATE_INITIAL        0
#define STATE_DRIVE_FORWARDS 1
#define STATE_FOUND_LINE     2
#define STATE_FOLLOW_LINE    3
#define STATE_END_OF_LINE    4

// This holds what state the robot is in.
int STATE;

// Initialisation
LineSensor line_left(LINE_LEFT_PIN); 
LineSensor line_centre(LINE_CENTRE_PIN);
LineSensor line_right(LINE_RIGHT_PIN);

PID pid( kp, ki, kd );

// Timestamps
float update_ts, buzzer_ts, incrementer_ts, init_ts;

// PID arguments
float demand;
float forward_speed;
float line_threshold;
float confidence_threshold;

// States
bool on_line;
bool first_line_found;
bool buzzer_on;
bool song_complete;

// Line Confidences
float left_line_confidence, centre_line_confidence, right_line_confidence;

// Song
unsigned long intervals[25] = { 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000 };
unsigned long buzz_interval;
int no_of_beats;
int interval_index;


void setup() {

  setupMotors();
  setupLeftEncoder();
  setupRightEncoder();
  setupTimestamps();
  setupBuzzer();
  setupSong();
  
  // Calibrate and setup line sensors
  setupLineSensors();
  
  // Calibration complete
  // play_delay_tone(20, 100);

  demand = 0.0;
  forward_speed = 20.0;
  line_threshold = 300.0;
  confidence_threshold = 50.0;

  // Initialise serial for debugging output
  Serial.begin(9600);

  // Set initial state, before robot begins to operate.
  STATE = STATE_INITIAL;
}


void loop() {

  unsigned long update_elapsed = millis() - update_ts;
  unsigned long buzzer_elapsed = micros() - buzzer_ts;
  unsigned long incrementer_elapsed = micros() - incrementer_ts;

  updateLineConfidence();
  on_line = checkForLine();

  Serial.print(line_left.readCalibrated());
  Serial.print(", ");
  Serial.print(line_centre.readCalibrated());
  Serial.print(", ");
  Serial.print(line_right.readCalibrated());
  Serial.print(", ");
  Serial.print(checkForLine());
  Serial.print(", ");
  Serial.println(STATE);

  // Based on the value of STATE variable,
  // run code for the appropriate robot behaviour.
  switch( STATE ) {
    case STATE_INITIAL:
      initialisingBeeps(); 
      break;
    case STATE_DRIVE_FORWARDS:
      driveForwards();     
      break;
    case STATE_FOUND_LINE:
      foundLineBeep();
      break;
    case STATE_FOLLOW_LINE:
      if (update_elapsed > 10) {
        update_ts = millis();
        followLine();
      }
      break;
    case STATE_END_OF_LINE:
      endOfLineBeep(buzzer_elapsed, incrementer_elapsed);
      break;
    default:
      Serial.println("System Error, Unknown state!");
      break;
  }
}

// Beep 5 times across 5 seconds
void initialisingBeeps() {

  for (int i = 0; i < 2; i++) {
    play_delay_tone(5, 100);
    delay(900);
  }
   
  // Update state
  STATE = STATE_DRIVE_FORWARDS;
}

// Command robot to drive forwards straight until it detects the line
void driveForwards() {

  if(!on_line) {
    updateSpeed(forward_speed + 1.0, forward_speed);
  } else {
    // Update state
    STATE = STATE_FOUND_LINE;
  }

}

// Stop and beep when line found
void foundLineBeep() {

  updateSpeed(0, 0);

  // play_delay_tone(10, 100);
  
  left_line_confidence = 100.0;
  centre_line_confidence = 100.0;
  right_line_confidence = 100.0;

  STATE = STATE_FOLLOW_LINE; 
}

void followLine() {

  // If line is detected, update PID and speeds and send to motor
  if (on_line) {
    updatePID();
  } 
  else {
    STATE = STATE_END_OF_LINE;
  }
}

void endOfLineBeep(unsigned long buzzer_elapsed, unsigned long incrementer_elapsed) {
  updateSpeed(0, 0);

  play_delay_tone(20, 100);

  if (!song_complete) {
    play_song(buzzer_elapsed, incrementer_elapsed);
  }
}

void updateSpeed(float left_new_speed, float right_new_speed) {
 
  if (left_new_speed < 0) {
    left_new_speed = left_new_speed * -1;
    digitalWrite( L_DIR_PIN, DIR_BKD );
  } else {
    digitalWrite( L_DIR_PIN, DIR_FWD );
  }

  if (right_new_speed < 0) {
    right_new_speed = right_new_speed * -1;
    digitalWrite( R_DIR_PIN, DIR_BKD );
  } else {
    digitalWrite( R_DIR_PIN, DIR_FWD );
  }

  analogWrite( L_PWM_PIN, left_new_speed );
  analogWrite( R_PWM_PIN, right_new_speed );
}

void updateLineConfidence() {
  bool left_above_threshold = line_left.readCalibrated() > line_threshold;
  bool centre_above_threshold = line_left.readCalibrated() > line_threshold;
  bool right_above_threshold = line_left.readCalibrated() > line_threshold;

  // If sensor is above threshold, increase confidence
  if (left_above_threshold) {
    left_line_confidence += 1.0;
  }
  else {
    left_line_confidence -= 1.0;
  }

  if (centre_above_threshold) {
    centre_line_confidence += 1.0;
  }
  else {
    centre_line_confidence -= 1.0;
  }

  if (right_above_threshold) {
    right_line_confidence += 1.0;
  }
  else {
    right_line_confidence -= 1.0;
  }

  left_line_confidence = constrain(left_line_confidence, 0.0, 100.0);
  centre_line_confidence = constrain(centre_line_confidence, 0.0, 100.0);
  right_line_confidence = constrain(right_line_confidence, 0.0, 100.0);
  
  return;
}

bool checkForLine() {
  bool left_on_line = left_line_confidence > confidence_threshold;
  bool centre_on_line = centre_line_confidence > confidence_threshold;
  bool right_on_line = right_line_confidence > confidence_threshold;
  
  if (left_on_line || centre_on_line || right_on_line) {
    if (!first_line_found) {
      first_line_found = true;
    }
    return true;
  }

  return false;
}

void updatePID() {
  // Get weighted line sensing ratio
  float line_centre_value = getLineCentre();

  // output_signal <----PID---- demand, measurement
  float pid_output = pid.update(demand, line_centre_value);

  float left_speed = forward_speed - pid_output;
  float right_speed = forward_speed + pid_output;

  left_speed = constrain(left_speed, -254, 254);
  right_speed = constrain(right_speed, -254, 254);

  updateSpeed(left_speed, right_speed);
}

float getLineCentre() {
  float left_value = line_left.readCalibrated();
  float centre_value = line_centre.readCalibrated();
  float right_value = line_right.readCalibrated();

  float I_total = left_value + centre_value + right_value;

  float p_1 = left_value / I_total;
  float p_2 = centre_value / I_total;
  float p_3 = right_value / I_total;

  float line_centre = (p_1 * 1000) + (p_2 * 2000) + (p_3 * 3000);

  line_centre = line_centre - 2000;
  
  line_centre = constrain(line_centre, -2000, 2000);

  return line_centre;
}

void play_delay_tone(int volume, int duration) {
  analogWrite(BUZZER_PIN, volume);
  delay(duration);
  analogWrite(BUZZER_PIN, 0);
}

void play_tone(int volume) {
  digitalWrite(BUZZER_PIN, volume);
}

void play_song(unsigned long buzzer_elapsed, unsigned long incrementer_elapsed) {
  // Increment buzz_interval every 1000ms
  if (incrementer_elapsed > 100000) {
    incrementer_ts = micros();
    interval_index = (interval_index + 1) % no_of_beats;
    buzz_interval = intervals[interval_index];
  }
  
  // Implement a millis() or micros() task block
  // to toggle the state of the buzzer.
  if (buzzer_elapsed > buzz_interval) {
    buzzer_ts = micros();

    if (buzzer_on == false) play_tone(1);
    else play_tone(0);
    
    buzzer_on = !buzzer_on;
  }

  if (interval_index == (no_of_beats - 1)) {
    play_tone(0);
    buzzer_on = false;
    song_complete = true;
  }
}


void setupMotors() {
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial direction for l and r wheels
  digitalWrite( L_DIR_PIN, DIR_FWD );
  digitalWrite( R_DIR_PIN, DIR_FWD );
}

void setupTimestamps() {
  update_ts = millis();
  init_ts = millis();
  buzzer_ts = micros();
  incrementer_ts = micros();
}

void setupLineSensors() {
  // Calibrate line sensors
  line_left.calibrate();
  line_centre.calibrate();
  line_right.calibrate();

  on_line = false;
  first_line_found = false;

  left_line_confidence = 0.0;
  centre_line_confidence = 0.0;
  right_line_confidence = 0.0;
}

void setupBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  buzzer_on = false;
}

void setupSong() {
  song_complete = false;
  interval_index = 0;
  buzz_interval = intervals[interval_index];
}


