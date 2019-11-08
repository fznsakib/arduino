#include "encoders.h"
#include "pid.h"
#include "line_sensors.h"

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define DIR_FWD LOW
#define DIR_BKD HIGH

#define BAUD_RATE 9600
#define BUZZER_PIN 6

#define LINE_LEFT_PIN   A4 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN  A2 //Pin for the right line sensor

// Initialise line sensors
LineSensor line_left(LINE_LEFT_PIN); 
LineSensor line_centre(LINE_CENTRE_PIN);
LineSensor line_right(LINE_RIGHT_PIN);

// PID Values
#define kp 0.11
#define ki 0.0
#define kd 0.0

PID pid( kp, ki, kd );

// Timestamps
float update_ts;
float buzzer_ts;
float incrementer_ts;
float init_ts;

float demand;
float forward_speed;

// States
bool on_line;
bool first_line_found;
bool buzzer_on;
bool beep_complete;

// Line Confidence
float left_line_confidence;
float centre_line_confidence;
float right_line_confidence;

// Thresholds
float line_threshold;
float confidence_threshold;

unsigned long elapsed_time;

// unsigned long intervals[20] = { 2000, 5000, 2000, 2000, 3000, 5000, 5000, 5000, 500, 4000, 1000, 2000, 2500, 500, 5000, 4000, 3000, 1000, 2000, 3000 };
unsigned long intervals[25] = { 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000, 100, 3000 };
unsigned long buzz_interval;

int no_of_beats;
int interval_index;


void setup() 
{
  setupMotors();
  setupLeftEncoder();
  setupRightEncoder();

  pinMode(BUZZER_PIN, OUTPUT);

  update_ts = millis();
  init_ts = millis();
  buzzer_ts = micros();
  incrementer_ts = micros();

  line_left.calibrate();
  line_centre.calibrate();
  line_right.calibrate();

  // Calibration complete
  play_delay_tone(20, 100);

  demand = 0.0;
  forward_speed = 20.0;

  on_line = false;
  first_line_found = false;
  buzzer_on = false;
  beep_complete = false;

  left_line_confidence = 0.0;
  centre_line_confidence = 0.0;
  right_line_confidence = 0.0;

  line_threshold = 300.0;
  confidence_threshold = 50.0;

  no_of_beats = 25;
  interval_index = 0;
  buzz_interval = intervals[interval_index];
  
  // Delay for start of run
  delay(1000);

  // Initialise the Serial communication
  Serial.begin( 9600 );
  delay(1000);
  Serial.println("***RESET***");
}

void loop() 
{

  // Check if on a line
  updateLineConfidence();
  on_line = checkForLine();

  elapsed_time = millis() - init_ts;

  // Action if line is no longer found
  if (!on_line) {
    // Play song when end of line is found
    if (first_line_found && elapsed_time > 5000) {
      updateSpeed(0, 0);

      unsigned long time_now = micros();
      unsigned long buzzer_elapsed = time_now - buzzer_ts;
      unsigned long incrementer_elapsed = time_now - incrementer_ts;

      if (!beep_complete) {
        play_song(buzzer_elapsed, incrementer_elapsed);
      }
    }
    else updateSpeed(21, 20);    
  }

  // Update PID
  elapsed_time = millis() - update_ts;

  if (elapsed_time > 5) {
    update_ts = millis();

    if (on_line) {
      updatePID();
    }
  }

}


void updateLineConfidence() {
  // If sensor is above threshold, increase confidence
  if (line_left.readCalibrated() > line_threshold) {
    left_line_confidence += 1.0;
  }
  else {
    left_line_confidence -= 1.0;
  }

  if (line_centre.readCalibrated() > line_threshold) {
    centre_line_confidence += 1.0;
  }
  else {
    centre_line_confidence -= 1.0;
  }

  if (line_right.readCalibrated() > line_threshold) {
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
    beep_complete = true;
  }
}

void play_delay_tone(int volume, int duration) {
  analogWrite(BUZZER_PIN, volume);
  delay(duration);
  analogWrite(BUZZER_PIN, 0);
}

void play_tone(int volume) {
  digitalWrite(BUZZER_PIN, volume);
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




