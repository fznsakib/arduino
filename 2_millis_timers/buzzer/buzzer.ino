#define BAUD_RATE 9600
#define BUZZER_PIN 6

// Global variables.
unsigned long buzzer_ts;
unsigned long incrementer_ts;

unsigned long buzz_interval;
bool buzzer_state;

unsigned long intervals[10] = { 200, 200, 200, 200, 300, 500, 500, 500, 500, 400 };
int no_of_beats = 10;
int interval_index = 0;

// Remember, setup runs once when the romi is powered up
void setup() {
  
  //Start a serial connection
  Serial.begin(BAUD_RATE);
  
  // Wait for stable connection, report reset.
  delay(1000);
  Serial.println("***RESET***");

  // Set pin 6 (buzzer) to output.
  pinMode( BUZZER_PIN, OUTPUT);

  // Initialise your global variables.
  buzzer_ts = micros();
  incrementer_ts = micros();

  buzz_interval = intervals[interval_index];
}

void play_tone(int state) {
  digitalWrite(BUZZER_PIN, state);
}

void play_song(unsigned long buzzer_elapsed, unsigned long incrementer_elapsed) {
  // Increment buzz_interval every 1000ms
    if (incrementer_elapsed > 220000) {
      incrementer_ts = micros();
      interval_index = (interval_index + 1) % no_of_beats;
      buzz_interval = intervals[interval_index];
      Serial.println(buzz_interval);
    }
    
    // Implement a millis() or micros() task block
    // to toggle the state of the buzzer.
    if (buzzer_elapsed > buzz_interval) {
      buzzer_ts = micros();

      if (buzzer_state == false) play_tone(1);
      else play_tone(0);
      
      buzzer_state = !buzzer_state;
    }
}

// Remember, loop repeats again and again.
void loop() {

    unsigned long time_now = micros();
    unsigned long buzzer_elapsed = time_now - buzzer_ts;
    unsigned long incrementer_elapsed = time_now - incrementer_ts;

    play_song(buzzer_elapsed, incrementer_elapsed);
    
}
