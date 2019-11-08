
// LED timestamps
unsigned long yellow_led_ts;
unsigned long red_led_ts;
unsigned long green_led_ts;

//int sensor_value;             // We will store a sensor reading in this.

// Remember: Setup only runs once when the arduino is powered up.
void setup() {

//  pinMode(A0, INPUT );              // Setup up A0 as input to read.
  pinMode(LED_BUILTIN, OUTPUT);       // Yellow LED
  pinMode(LED_BUILTIN_RX, OUTPUT);    // Red LED
  pinMode(LED_BUILTIN_TX, OUTPUT);    // Green LED

  yellow_led_ts = millis();   // We set an intial timestamp value.
  red_led_ts = millis();      // We set an intial timestamp value.
  green_led_ts = millis();    // We set an intial timestamp value.
}

void flash_leds(int led) {
  digitalWrite(led, HIGH);   
  delay(50);                       
  digitalWrite(led, LOW);
  delay(50);
}



// Remember: loop is called again and again.
void loop() {

    // Get how much time has passed right now.
    unsigned long time_now = millis();     

    // Work out how many milliseconds have gone passed by subtracting
    // our two timestamps.  time_now will always be bigger than the
    // time_of_read (except when millis() overflows after 50 days).
    unsigned long yellow_elapsed_time = time_now - yellow_led_ts;
    unsigned long red_elapsed_time = time_now - red_led_ts;
    unsigned long green_elapsed_time = time_now - green_led_ts;


    // See if 5000 milliseconds have elapsed
    // If not, this block is skipped.
    if( yellow_elapsed_time > 5000 ) {

        // Since 5000ms elapsed, we overwrite our last timestamp for the yellow
        // LED with the current time so that another 5000ms is needed to pass.
        yellow_led_ts = millis();

        // Flash yellow LED
        flash_leds(LED_BUILTIN);
    }
    if( red_elapsed_time > 2500 ) {

        red_led_ts = millis();

        // Flash red LED
        flash_leds(LED_BUILTIN_RX);
    }
    if (green_elapsed_time > 1000) {
        
        green_led_ts = millis();

        // Flash green LED
        flash_leds(LED_BUILTIN_TX);
    }

    // Code outside the above if{} will run on every loop!
    // Therefore code here is no longer stopped waiting for delay()

}
