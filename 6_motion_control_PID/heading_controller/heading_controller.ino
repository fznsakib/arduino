#include "encoders.h"
#include "pid.h"

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define DIR_FWD LOW
#define DIR_BKD HIGH

#define OUTPUT_LIMIT 215.0

// Proportional, Integral and Derivative gains for position controller
#define kp_left 0.5
#define ki_left 0.0
#define kd_left 12.0

#define kp_right 0.5
#define ki_right 0.0
#define kd_right 12.0

#define kp_heading 1.0
#define ki_heading 0.0
#define kd_heading 0.0

// Position controllers for left/right wheel position
PID left_pid(kp_left, ki_left, kd_left);
PID right_pid(kp_right, ki_right, kd_right); 
PID heading_pid(kp_heading, ki_heading, kd_heading);

float wheel_update_ts;
float heading_update_ts;
float demand_switch_ts;

float demand;
float heading_demand;
float heading_output;

void setup() 
{
  setupMotors();
  
  // Initialise your other globals variables
  // and devices.
  setupLeftEncoder();
  setupRightEncoder();

  demand = 1000.0;
  heading_demand = 0.0;
  heading_output = 0.0;

  wheel_update_ts = millis();
  heading_update_ts = millis();
  demand_switch_ts = millis();
  
  // Initialise the Serial communication
  Serial.begin( 9600 );
  delay(1000);
  Serial.println("***RESET***");
}

void loop() 
{ 
  // HEADING CONTROLLER: Update at a smaller interval of 100ms than wheels 
  // to avoid over compensating for bias.
  float elapsed_time = millis() - heading_update_ts;

  if (elapsed_time > 50) {
    heading_update_ts = millis();

    // Heading demand for a straight line requires equal counts for both wheel
    float heading_measurement = count_right_e - count_left_e;

    heading_output = heading_pid.update(heading_demand, heading_measurement);
  }

  elapsed_time = millis() - wheel_update_ts;

  // WHEELS: Call pid.update() on at a regular time interval of 10ms
  if (elapsed_time > 10) {
    wheel_update_ts = millis();

    float left_demand = demand - heading_output/2;
    float right_demand = demand + heading_output/2;

    // output_signal <----PID---- demand, measurement
    float left_output = left_pid.update(left_demand, count_left_e);
    float right_output = right_pid.update(right_demand, count_right_e);

    Serial.print(heading_output);
    Serial.print(", ");
    Serial.print(count_left_e);
    Serial.print(", ");
    Serial.print(count_right_e);
    Serial.print(", ");
    Serial.print(left_output);
    Serial.print(", ");
    Serial.println(right_output);

    updateSpeed(left_output, right_output);
  }

  // DEMAND: Switch every 3 seconds
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
