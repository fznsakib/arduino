#include "encoders.h"
#include "pid.h"

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define DIR_FWD LOW
#define DIR_BKD HIGH

#define kp_left 0.12
#define ki_left 0.01
#define kd_left 4.0

#define kp_right 0.12
#define ki_right 0.01
#define kd_right 4.0

PID left_pid( kp_left, ki_left, kd_left );
PID right_pid( kp_right, ki_right, kd_right ); 

float update_ts;
float demand_switch_ts;

float demand;

// Remember, setup only runs once.
void setup() 
{
  setupMotors();
  
  // Initialise your other globals variables
  // and devices.
  setupLeftEncoder();
  setupRightEncoder();

  update_ts = millis();
  demand_switch_ts = millis();

  demand = 1000.0;

  // Initialise your other globals variables
  // and devices.
  left_e_last_time = micros();
  left_e_interval = 0.0;

  left_e_prev_speed = 0.0;
  left_e_speed = 0.0;

  right_e_last_time = micros();
  right_e_interval = 0.0;

  right_e_prev_speed = 0.0;
  right_e_speed = 0.0;
  
  // Initialise the Serial communication
  Serial.begin( 9600 );
  delay(1000);
  Serial.println("***RESET***");
}


// Remmeber, loop is called again and again.
void loop() 
{

  float elapsed_time = millis() - update_ts;

  // Call pid.update() at a regular time interval.
  if (elapsed_time > 10) {
    update_ts = millis();

    // output_signal <----PID---- demand, measurement
    float left_output = left_pid.update(demand, left_e_speed);
    float right_output = right_pid.update(demand, right_e_speed);

    // Plotted values
    Serial.print(left_e_speed);
    Serial.print(",");
    Serial.print(right_e_speed);
    Serial.print(",");
    Serial.print(left_output);
    Serial.print(",");
    Serial.println(right_output);


    updateSpeed(left_output, right_output);
  }

  elapsed_time = millis() - demand_switch_ts;
  
  // Switch direction of speed every 3 seconds
  if (elapsed_time > 3000) {
    demand_switch_ts = millis();
    demand = -demand;
  }
}

void updateSpeed(float left_new_speed, float right_new_speed) {

  if (left_new_speed < 0.0) {
    left_new_speed = left_new_speed * -1;
    digitalWrite( L_DIR_PIN, DIR_BKD );
  } else {
    digitalWrite( L_DIR_PIN, DIR_FWD );
  }

  if (right_new_speed < 0.0) {
    right_new_speed = right_new_speed * -1;
    digitalWrite( R_DIR_PIN, DIR_BKD );
  } else {
    digitalWrite( R_DIR_PIN, DIR_FWD );
  }

  analogWrite( L_PWM_PIN, left_new_speed );
  analogWrite( R_PWM_PIN, right_new_speed );
}

void resetZeroSpeed() {
    // Recorded speed goes down to 0  
  if (left_e_prev_speed == left_e_speed) {
    left_e_speed = 0;
  } else {
    left_e_prev_speed = left_e_speed;
  }

  if (right_e_prev_speed == right_e_speed) {
    right_e_speed = 0;
  } else {
    right_e_prev_speed = right_e_speed;
  }
}

void setupMotors() {
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial direction for l and r wheels
  digitalWrite( L_DIR_PIN, DIR_FWD  );
  digitalWrite( R_DIR_PIN, DIR_FWD );
}