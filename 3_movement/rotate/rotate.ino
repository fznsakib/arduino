// Pin definitions, to make the
// code easier to read.
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

// Variables to remember our
// motor speeds for Left and Right.
#define DIR_FWD LOW
#define DIR_BKD HIGH

unsigned long ts;
unsigned long rotate_ts;

bool rotating = true;

float l_speed;
float r_speed;

// Setup, only runs once when powered on.
void setup() {

  // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial direction for l and r
  // Which of these is foward, or backward?
  digitalWrite( L_DIR_PIN, DIR_FWD  );
  digitalWrite( R_DIR_PIN, DIR_FWD );
  
  // Set initial l_speed and r_speed values.
  l_speed = 100;
  r_speed = 100;

  ts = millis();
  rotate_ts = millis();

  // Start up the Serial for debugging.
  Serial.begin(9600);
  delay(1000);
  // Print reset so we can catch any reset error.
  Serial.println(" ***Reset*** ");

}

// Set speed of wheel in the direction specified by the sign
// of the speed value. Speed changed for wheel specified as parameter.
void set_speed(int dirPin, int pwmPin, float newSpeed) {
  // Do nothing if speed is past limit  
  if (newSpeed > 255.0 || newSpeed < -255.0) {
    return;
  }
   
  // Set direction
  if (signbit(newSpeed)) digitalWrite(dirPin, DIR_BKD);
  else digitalWrite(dirPin, DIR_FWD);

  // Set speed  
  analogWrite(pwmPin, abs(newSpeed));
}

float rand_speed( float min, float max )
{   
    // Produce random float between 0.0 and 1.0
    float scale = rand() / (float) RAND_MAX;
    // Between min and max
    return min + scale * ( max - min );
}

void rotate() {
    l_speed = -30.0;
    r_speed = 60.0;
    set_speed(L_DIR_PIN, L_PWM_PIN, l_speed);
    set_speed(R_DIR_PIN, R_PWM_PIN, r_speed);  
}

void stop() {
  set_speed(L_DIR_PIN, L_PWM_PIN, 0.0);
  set_speed(R_DIR_PIN, R_PWM_PIN, 0.0);  
}

// put your main code here, to run repeatedly:
void loop() {

  unsigned long elapsed = millis() - ts;
  unsigned long rotate_elapsed = millis() - rotate_ts;

  set_speed(L_DIR_PIN, L_PWM_PIN, -50.0);
  set_speed(R_DIR_PIN, R_PWM_PIN, -50.0);  

  // Set speed at intervals  
//  if (elapsed > 2350) {
//    ts = millis();
//    if (!rotating) rotate();
//    else stop();
//    
//    rotating = !rotating;
//  }

//  if (rotate_elapsed > 250) {
//    rotate_ts = millis();
//    set_speed(L_DIR_PIN, L_PWM_PIN, 0.0);
//    set_speed(R_DIR_PIN, R_PWM_PIN, 0.0);  
//  }  

}
