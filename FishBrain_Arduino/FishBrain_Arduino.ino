#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <Servo.h>

/*
   Connections
   ===========
   Connect SCL to SCL pin (analog 5 on Arduino UNO)
   Connect SDA to SDA pin (analog 4 on Arduino UNO)
   Connect VDD to 5V DC (depending on your board's logic level)
   Connect GROUND to common ground
   Connect Digital Pin 9 to Servo1
   Connect Digital Pin 10 to Servo 2
   
*/

/**************************************************************************/
/*
    Initialization of Global Parameters
*/
/**************************************************************************/
/* ------ Definitions------------- */

#define BNO055_SAMPLERATE_DELAY_MS (100)  // Set the delay between samples
#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04
#define servo1Pin 9 //attach pin D3 Arduino to pin Trig of HC-SR04
#define servo2Pin 10 //attach pin D3 Arduino to pin Trig of HC-SR04

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // Set I2C address of BNO055


/* ------ Global Variables-------- */

const int initialServoValue = 90; // Set servo start value to 90 degrees
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

// PID Variables
double pitchSetpoint, rollSetpoint, pitchInput, rollInput, pitchOutput, rollOutput, OutP, OutR, Out1, Out2;

// PID Tuning Parameters
double Kp=1.5, Ki=0, Kd=0; // Proportional, Integral, Derivative

/* ------ Object Declarations-------- */

PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT); // Pitch PID 
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);    // Roll PID

Servo servo1;  // Servo 1 object
Servo servo2;  // Servo 2 object


/**************************************************************************/
/*
    Display sensor calibration status (For Debugging)
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}


/**************************************************************************/
/*
    Compute Distance using HC-SR04 Ultrasonic Range-finder
*/
/**************************************************************************/
int computeDistance(void)
{
  digitalWrite(trigPin, LOW); // Clear trigger
  delayMicroseconds(2);
  
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin (sound wave travel time in microseconds)
  duration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance from return wave
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (return wave)
  
  // Return the distance
  return distance;
}
/**************************************************************************/
/*
    Arduino Setup
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600); // Set baud rate to 9600
  
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  /* Set pinmodes for HR-SC04 ultrasonic sensor */
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

   /* Attach servos and write initial position */
  servo1.attach(servo1Pin);  // attaches the servo1 to defined pin
  servo1.write(initialServoValue); //set Servo position to midpoint

  servo2.attach(servo2Pin); // attaches the servo2 to defined pin
  servo2.write(initialServoValue); //set Servo position to midpoint
   
  /* Turn on PID and set output limits */
  pitchPID.SetMode(AUTOMATIC); // Turn pitch PID on
  rollPID.SetMode(AUTOMATIC); // Turn roll PID on
  pitchPID.SetOutputLimits(-90,90); //set PID limits to the servo limits
  rollPID.SetOutputLimits(-90,90); //set PID limits to the servo limits

  bno.setExtCrystalUse(true); // Used for BNO055

  delay(1000); // Delay for calibration
}

/**************************************************************************/
/*
    Main Arduino Loop Function
*/
/**************************************************************************/

void loop(void)
{

/* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);



 
/* If near surface, set foil angle to 45 degrees  */
  computeDistance();
  if(distance <10){
    servo1.write(45);
    servo2.write(135);
    delay(15);
  }
  else{

  /* Compute PID outputs for pitch and roll  */
  pitchInput = event.orientation.y;  // Pitch input = y position
  pitchPID.Compute();
  OutP = pitchOutput;
  
  rollInput = event.orientation.z;  // Roll input = z position
  rollPID.Compute();
  OutR = rollOutput;

  /* Compute combined roll and pitch outputs */
  Out1 = 90 - OutP + OutR;
  Out2 = 90 + OutP + OutR;

  /* Write combined output to servos*/
  servo1.write(Out1);
  servo2.write(Out2); 
  }
  

  /* Prints values to serial monitor. Used for Debugging.  */

  /*
  Serial.print("Pitch PID Input: ");
  Serial.println(pitchInput);
  Serial.print("Pitch PID Output: ");
  Serial.println(pitchOutput);
  Serial.print("Roll PID Input: ");
  Serial.println(rollInput);
  Serial.print("Roll PID Output: ");
  Serial.println(rollOutput);
  Serial.print("PID OutP: ");
  Serial.println(OutP);
  Serial.print("PID OutR: ");
  Serial.println(OutR);
  Serial.print("Output1: ");
  Serial.println(Out1);
  Serial.print("Output2: ");
  Serial.println(Out2);

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  */
  
  /* Write orientation data to Serial */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
