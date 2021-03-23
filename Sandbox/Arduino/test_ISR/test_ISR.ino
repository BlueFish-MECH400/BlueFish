const int PiPin = 3;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

// variables will change:
volatile int PiState = 0;         // variable for reading the pushbutton status
volatile float LOG_RATE = 10; //  Hz
volatile float TARGET_DEPTH; //  m
volatile float TARGET_ALTITUDE; //  m
volatile float MODE; // -
volatile float PITCH_Kp;  // -
volatile float PITCH_Ki;  // -
volatile float PITCH_Kd;  // -
volatile float ROLL_Kp; // -
volatile float ROLL_Ki; // -
volatile float ROLL_Kd; // -

volatile float CONTROL_VARS[10] = {LOG_RATE, TARGET_DEPTH,
                                 MODE, PITCH_Kp, PITCH_Ki,
                                 PITCH_Kd, ROLL_Kp, ROLL_Ki,
                                 ROLL_Kd};
String PRINT_CONTROL_VARS[10] = {"LOG_RATE", "TARGET_DEPTH",
                                 "MODE", "PITCH_Kp", "PITCH_Ki",
                                 "PITCH_Kd", "ROLL_Kp", "ROLL_Ki",
                                 "ROLL_Kd"};

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(PiPin, INPUT);
  // Attach an interrupt to the ISR vector
  attachInterrupt(1, pin_ISR, RISING);
}

void loop() {
  for i=0, i=<10, i++{
    Serial.println(PRINT_CONTROL_VARS[i] + " = " 
                    + String(CONTROL_VARS[i]));
}

void pin_ISR() {
  for i=0, i=<10, i++{
    String temp = Serial.readStringUntil('\n');
    CONTROL_VARS[i] = temp.toFloat();
  }
}
