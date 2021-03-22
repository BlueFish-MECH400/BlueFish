const int PiPin = 3;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

// variables will change:
volatile int PiState = 0;         // variable for reading the pushbutton status

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(PiPin, INPUT);
  // Attach an interrupt to the ISR vector
  attachInterrupt(1, pin_ISR, CHANGE);
}

void loop() {
  // Nothing here!
}

void pin_ISR() {
  PiState = digitalRead(PiPin);
  digitalWrite(ledPin, PiState);
}
