/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/
int pin_LED1 = 11;
int pin_LED2 = 12;
int pin_LED3 = 13;
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(pin_LED1, OUTPUT);
  pinMode(pin_LED2, OUTPUT);
  pinMode(pin_LED3, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(pin_LED1, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(pin_LED2, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(pin_LED3, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(pin_LED1, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(pin_LED2, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(pin_LED3, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
}
