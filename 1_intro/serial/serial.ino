#define BAUD_RATE 9600

int no_of_flashes = 0;

void flash_leds ()
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000); 
}

// the setup function runs once when you press reset or power the board
void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  //Start a serial connection
  Serial.begin(BAUD_RATE);
  // Wait for stable connection, report reset.
  delay(1000);
  Serial.println("***RESET***");
}

// the loop function runs over and over again forever
void loop()
{
  //This line checks whether there is anything to read
  if (Serial.available()) { 
      char inChar = Serial.read(); //This reads one byte
      
      if (inChar =='s') {
        Serial.println("Got s");
      }
      else if (inChar == 'l') {
        flash_leds();
        no_of_flashes++;
      }
      else if (inChar == 'r') {
        for (int i = 0; i < 10; i++) {
          flash_leds();
          no_of_flashes++;
        }
      }
  }
  Serial.println(no_of_flashes);
}
