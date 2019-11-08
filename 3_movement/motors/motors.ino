// Pin definitions, to make the
// code easier to read.
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define DIR_FWD LOW
#define DIR_BKD HIGH

#define MAX_SPEED 100

// Variables to remember our
// motor speeds for Left and Right.
// Byte stores 0 to 255
byte l_speed;
byte r_speed;

bool speeding_up = true;
int rate = 20;

unsigned long ts;

// Setup, only runs once when powered on.
void setup() {

  // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial direction for l and r
  // LOW is forward; HIGH is backward
  digitalWrite( L_DIR_PIN, DIR_FWD  );
  digitalWrite( R_DIR_PIN, DIR_FWD );
  
  // Set initial l_speed and r_speed values.
  l_speed = 0;
  r_speed = 0;
 
  // Set timestamp
  ts = millis();
  
  // Start up the Serial for debugging.
  Serial.begin(9600);
  delay(1000);
  // Print reset so we can catch any reset error.
  Serial.println(" ***Reset*** ");

}

// Increment or decrement speed depending on current speed
int get_speed() {
  int new_speed = l_speed;
  
  if (speeding_up) {
    new_speed = new_speed + rate;
  } else {
    new_speed = new_speed - rate; 
  }

  if (new_speed < 0) return 0;
  if (new_speed > MAX_SPEED) return MAX_SPEED;

  return new_speed;
}

// put your main code here, to run repeatedly:
void loop() {
  
  unsigned long elapsed_time = millis() - ts;
  
  
  // Increment/decrement speed every half a second
  if (elapsed_time > 500) {

    ts = millis();
    
    // Check if beyond max speed
    if (l_speed >= MAX_SPEED && r_speed >= MAX_SPEED) {
      speeding_up = false;
    }

    // Check if below min speed    
    if (l_speed == 0 && r_speed == 0) {
      speeding_up = true;
    }

    int new_speed = get_speed();
    
    // Adjust speeds every loop
    l_speed = new_speed;
    r_speed = new_speed;

    Serial.println(l_speed);
  
    // Send speeds to pins, to motor drivers.
    analogWrite( L_PWM_PIN, l_speed );
    analogWrite( R_PWM_PIN, r_speed );

  }
  
}
