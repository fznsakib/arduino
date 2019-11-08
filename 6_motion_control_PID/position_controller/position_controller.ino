#include "encoders.h"
#include "pid.h"

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define DIR_FWD LOW
#define DIR_BKD HIGH

// Proportional, Integral and Derivative gains for position controller
#define kp_left 0.5
#define ki_left 0.0
#define kd_left 12
#define kp_right 0.5
#define ki_right 0.0
#define kd_right 12

// Position controllers for left/right wheel position
PID left_pid(kp_left, ki_left, kd_left);
PID right_pid(kp_right, ki_right, kd_right); 

float update_ts;
float demand_switch_ts;

float demand;

void setup() 
{
  setupMotors();
  
  // Initialise your other globals variables
  // and devices.
  setupLeftEncoder();
  setupRightEncoder();

  demand = 500.0;
  update_ts = millis();
  demand_switch_ts = millis();
  
  // Initialise the Serial communication
  Serial.begin( 9600 );
  delay(1000);
  Serial.println("***RESET***");
}

void loop() 
{
  float elapsed_time = millis() - update_ts;

  // Call pid.update() at a regular time interval.
  if (elapsed_time > 10) {
    update_ts = millis();
    
    // output_signal <----PID---- demand, measurement
    float left_output = left_pid.update(demand, count_left_e);
    float right_output = right_pid.update(demand, count_right_e);

    Serial.print(count_left_e);
    Serial.print(", ");
    Serial.println(count_right_e);

    updateSpeed(left_output, right_output);
  }

  elapsed_time = millis() - demand_switch_ts;
  
  if (elapsed_time > 3000) {
    demand_switch_ts = millis();
    demand = -demand;
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

void setupMotors() {
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial direction for l and r wheels
  digitalWrite( L_DIR_PIN, DIR_FWD  );
  digitalWrite( R_DIR_PIN, DIR_FWD );
}
