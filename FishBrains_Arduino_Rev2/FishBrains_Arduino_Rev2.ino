/**************************************************************************/
/*      Libraries                                                         */
/**************************************************************************/
#include <Adafruit_Sensor.h>  // IMU 
#include <Adafruit_BNO055.h>  // IMU
#include <utility/imumaths.h> // IMU math
#include "ping1d.h"           // Ping Sonar
#include "SoftwareSerial.h"   // Software Serial
#include "MS5837.h"           // Bar30 Sensor
#include <PID_v1.h>           // PID
#include <Servo.h>            // Servo
#include <Wire.h>             // I2C

/**************************************************************************/
/*      Definitions                                                       */
/**************************************************************************/

/* Arduino Pins */
#define EINT1_PIN 2 // External interrupt 1 pin
#define LEAK_PIN 4  // Leak signal pin
#define SERVO_1_PIN 5 // Servo 1 pin 
#define SERVO_2_PIN 6 // Servo 2 pin
#define SRX_PIN  9  // Software serial Rx pin
#define STX_PIN  10 // Software serial Tx pin
#define LED_1_PIN 11  //  LED pin (for debugging)
#define LED_2_PIN 12  //  LED pin (for debugging)
#define LED_3_PIN 13  //  LED pin (for debugging)

/* States */
#define IDLE 0
#define DEPTH 1
#define ALTITUDE 2
#define SURFACE 3

/* General Constants */
#define SEA_WATER 1029  // Density of seawater in kg/m^3
#define BAUD_RATE 9600  // Serial baud rate
#define INIT_SERVO_POS 90 // Initial servo position 90 degrees
#define SERVO_LIMIT 45  // Max servo position in degrees (inital + 45)

/* IMU Sample Rate */
#define BNO055_SAMPLERATE_DELAY_MS (100)  // BNO055 sample delay in ms

/**************************************************************************/
/*      Global Variables                                                  */
/**************************************************************************/

uint8_t leak = 0; // 0 = Dry , 1 = Leak
uint8_t leakState = 0; // 0 = LED off , 1 = LED on

int state = IDLE; // State value

unsigned long currentTime, lastTime, transmitTime; // For time tracking
double logRate, logPeriod;

/* Sensor Variables */
double depth, pressure, temperature; // Bar30 data
double dx, dy, dz, gx, gy, gz;  // BNO055 IMU gyro data (y is pitch, z is roll)
double altitude;  // Ping sonar data

/* PID Variables */
double targetDepth, targetAltitude;  // Target values of depth and altitude
double pitchSetpoint, pitchInput, pitchOutput, OutP;  // Pitch PID
double rollSetpoint, rollInput, rollOutput, OutR; // Roll PID
double Out1, Out2;  // Servo1 and servo2 output

/* PID Tuning Parameters */
double pKp, pKi, pKd; // Pitch proportional, integral, derivative gains
double rKp, rKi, rKd; // Pitch proportional, integral, derivative gains
double errX, errY, errZ; // Error in yaw, pitch, roll values

unsigned int mDelay = 15; // Delay for servo motors
unsigned int sDelay = 500;  // Short delay
unsigned int lDelay = 1000; // Long delay

unsigned int servoMax = INIT_SERVO_POS + SERVO_LIMIT;
unsigned int servoMin = INIT_SERVO_POS - SERVO_LIMIT;

/**************************************************************************/
/*      Object Declarations                                               */
/**************************************************************************/

SoftwareSerial pingSerial = SoftwareSerial(SRX_PIN, STX_PIN); // Set ping serial to use SS pins
static Ping1D ping { pingSerial };  // Create ping object with SS

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // Create BNO055 object and set I2C address
sensors_event_t event; // Create event for BNO055

MS5837 bar30; // Create Bar30 object

Servo servo1;  // Servo 1 object
Servo servo2;  // Servo 2 object

PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, pKp, pKi, pKd, P_ON_M, DIRECT); // Pitch PID 
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, rKp, rKi, rKd, P_ON_M, DIRECT);  // Roll PID

/**************************************************************************/
/*      Functions                                                         */
/**************************************************************************/

void initSensor(void);  // Checks if sensors are functioning
void displayCalStatus(void);  // Display IMU calibration (for debugging)
void readSensors(void); // Read all sensors and store values
void transmitData(void);  // Transmit data via serial
void leakWarningFlash(void);  // Display leak warning on LEDs
void runPID(void);  // Compute PID outputs
void updateSettings(void);  // ISR to update settings sent from RPi
void outputPID(void);   // Display PID I/O (for debugging)

/**************************************************************************/
/*      Program Setup                                                     */
/**************************************************************************/

void setup(void)
{
  /* Set I/O pinmodes */
  pinMode(LEAK_PIN, INPUT);  // Set leak sensor pin to INPUT
  pinMode(EINT1_PIN, INPUT); // Set interrupt pin to INPUT
  pinMode(LED_1_PIN, OUTPUT);  // Set LED1 pin to OUTPUT
  pinMode(LED_2_PIN, OUTPUT);  // Set LED2 pin to OUTPUT
  pinMode(LED_3_PIN, OUTPUT);  // Set LED3 pin to OUTPUT

  bno.setExtCrystalUse(true); // Used for BNO055

  /* Initialize serial and I2C */
  Serial.begin(BAUD_RATE);  // Initialize serial at baud rate
  pingSerial.begin(BAUD_RATE);  // Initialize software serial at baud rate
  Wire.begin(); // Initialize I2C

  initSensor(); // Checks if sensors are functioning

  attachInterrupt(digitalPinToInterrupt(EINT1_PIN),updateSettings,RISING);

   /* Attach servos and write initial position */
  servo1.attach(SERVO_1_PIN);  // Attach servo1 to servo1 pin
  servo2.attach(SERVO_2_PIN); // Attach servo2 to servo2 pin
  servo1.write(INIT_SERVO_POS); // Set servo1 position to initial position
  servo2.write(INIT_SERVO_POS); // Set servo2 position to initial position
   
  /* Turn on PID and set output limits */
  pitchPID.SetMode(AUTOMATIC); // Set pitch PID mode automatic (ON)
  rollPID.SetMode(AUTOMATIC); // Set roll PID mode automatic (ON)
  pitchPID.SetOutputLimits(-SERVO_LIMIT,SERVO_LIMIT); // Set pitch PID limits to servo limits
  rollPID.SetOutputLimits(-SERVO_LIMIT,SERVO_LIMIT); // Set roll PID limits to servo limits

  /* Bar 30 sensor setup */
  bar30.setModel(MS5837::MS5837_30BA);
  bar30.setFluidDensity(SEA_WATER); // Set fluid density to sea water
  
  delay(2000); // Delay for IMU calibration
}

/**************************************************************************/
/*      Main Routine                                                      */
/**************************************************************************/

void loop(void){
  
  displayCalStatus(); // Wait until BNO055 calibrated (display status on LEDs)

  /* Print sensor data headers to serial */
  //Serial.println("Altitude,Depth,Pressure,Temperature,X,Y,Z"); REMOVE IF NOT NEEDED
  goto RUN_BLUEFISH;
  

  RUN_BLUEFISH:

    currentTime = millis(); // Set current time
    /* Check for leak */
    leak = digitalRead(LEAK_PIN);// Read the Leak Sensor Pin

    if (leak == HIGH) { // If leak detected
      state = DEPTH;  // Set state to DEPTH mode
      leakWarningFlash(); // Flash LEDs indicating leak
      goto MODE_SWITCH;
    }
    
    logPeriod = (1/logRate)*1000; // Convert log rate in Hz to period in milliseconds

    if((currentTime-transmitTime)>=logPeriod) {  // Check if time to transmit data
      readSensors();
      transmitData(); // Transmit data to Raspberry Pi
    }
    
    goto MODE_SWITCH;

  MODE_SWITCH:

    switch(state) {

      case IDLE:
        goto IDLE_MODE;
        break;

      case DEPTH:
        goto DEPTH_MODE;
        break;

      case ALTITUDE:
        goto ALTITUDE_MODE;
        break;

      case SURFACE:
        goto SURFACE_MODE;
        break;

      default:
        goto RUN_BLUEFISH;
    }

    IDLE_MODE:
      
      servo1.write(INIT_SERVO_POS);
      servo2.write(INIT_SERVO_POS);
      goto RUN_BLUEFISH;

    DEPTH_MODE:
      
      if (leak == HIGH) { // If leak detected
        targetDepth = 0;  // Set target depth to 0 (surface)
      }

      readSensors();  // Read sensor data
      pitchSetpoint = targetDepth; // Set pitch setpoint to target depth
      pitchInput = depth; // Set PID pitch input to depth
      runPID(); // Run PID to computed servo outputs
      servo1.write(Out1); // Write output to servo1
      servo2.write(Out2); // Write output to servo2
      delay(mDelay);

      goto RUN_BLUEFISH;
    
    ALTITUDE_MODE:
      
      readSensors();

      if(altitude <1) { // Check if close to seafloor
        servo1.write(servoMin); // Set servo1 position to min
        servo2.write(servoMax); // Set servo1 position to max
        delay(mDelay);
      }else {
        pitchSetpoint = targetAltitude; // Set pitch setpoint to target altitude
        pitchInput = altitude; // Set PID pitch input to altitude
        runPID(); // Run PID to computed servo outputs
        servo1.write(Out1); // Write output to servo1
        servo2.write(Out2); // Write output to servo2
        delay(mDelay);
      }

      goto RUN_BLUEFISH;
    
    SURFACE_MODE:
      
      servo1.write(INIT_SERVO_POS); // Set servo positions to initial position
      servo2.write(INIT_SERVO_POS); 
      delay(mDelay);

      goto RUN_BLUEFISH;
}
/*---- End of Main Routine -----------------------------------------------*/


/**************************************************************************/
/*      Function Definitions                                              */
/**************************************************************************/

/*========================================================================*/
/*------Sensor Intitialziation Verification-------------------------------*/
/*========================================================================*/
void initSensor(void) {
  int bnoC, bar30C, pingC;  // Sensor status: 0 = connected, 1 = disconnected

  if(!bno.begin()){  //check if BNO055 functioning
    bnoC = 1;
  }
  if(!bar30.init()) {  //check if Bar30 functioning
    bar30C  = 1;
  }
  if(!ping.initialize()) {  //check if Ping functioning
    pingC = 1;
  }
   /* Blink LED while sensor is not initiailized */
   /* LED1 = BNO055, LED2 = Bar30, LED3 = Ping */
  while(!bno.begin()||!bar30.init()||!ping.initialize()) {
    if(bnoC == 1) {
      digitalWrite(LED_1_PIN,HIGH);
    }
    if(bar30C == 1) {
      digitalWrite(LED_1_PIN,HIGH);
    }
    if(pingC == 1) {
      digitalWrite(LED_1_PIN,HIGH);
    }
    delay(sDelay);
    digitalWrite(LED_1_PIN,LOW);
    digitalWrite(LED_2_PIN,LOW);
    digitalWrite(LED_3_PIN,LOW);
  }
}

/*========================================================================*/
/*------Display sensor calibration status (For Debugging)-----------------*/
/*========================================================================*/
void displayCalStatus(void) {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  
  /* While all values not calibrated, get calibration status.*/
  /* 3 means 'fully calibrated" and requires system > 0  */
  /* When fully calibrated, all LEDs are on. */
  /* (LED1 = gyro, LED2 = accel, LED3 = mag) */
  while((gyro!= 3) && (accel!= 3) && (mag!= 3) && (system<= 1)){
    
    /* Get the four calibration values (0..3) */
    bno.getCalibration(&system, &gyro, &accel, &mag);

    if(gyro==3){
      digitalWrite(LED_1_PIN,HIGH);
    }else{
      digitalWrite(LED_1_PIN,LOW);
    }

    if(accel==3){
      digitalWrite(LED_1_PIN,HIGH);
    }else{
      digitalWrite(LED_1_PIN,LOW);
    }

    if(mag==3){
      digitalWrite(LED_1_PIN,HIGH);
    }else{
      digitalWrite(LED_1_PIN,LOW);
    }
  }

  Serial.println("Calibration Complete");
}

/*========================================================================*/
/*------Leak Sensor Warning-----------------------------------------------*/
/*========================================================================*/
void leakWarningFlash(void) {
      if((currentTime-lastTime)>=sDelay) {
        lastTime = currentTime;
      
        if(leakState==LOW) {
          leakState = HIGH;
          digitalWrite(LED_1_PIN, leakState);
          digitalWrite(LED_2_PIN, leakState);
          digitalWrite(LED_3_PIN, leakState);
        }else if(leakState == HIGH) {
          leakState = LOW;
          digitalWrite(LED_1_PIN, leakState);
          digitalWrite(LED_2_PIN, leakState);
          digitalWrite(LED_3_PIN, leakState);
        } 
      } 
}

/*========================================================================*/
/*------Read Sensor Data--------------------------------------------------*/
/*========================================================================*/
void readSensors(void) {
  /* Read Bar30 sensor data */
  bar30.read();
  pressure = bar30.pressure();
  temperature = bar30.temperature();
  depth = bar30.depth();

  /* Read BNO055 sensor data */

   /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  /* Store data */
  dx = event.orientation.x;
  dy = event.orientation.y;
  dz = event.orientation.z;
  gx = event.gyro.x;
  gy = event.gyro.y;
  gz = event.gyro.z;

  /* Read Ping sensor Data*/
  if (ping.update()) {
    altitude = ping.distance();
  }
}

/*========================================================================*/
/*------Transmit Data via Serial------------------------------------------*/
/*========================================================================*/
void transmitData(void) {
  /* Transmit sensor data */
  Serial.print(ping.distance());
  Serial.print(",");
  Serial.print(bar30.depth());
  Serial.print(",");
  Serial.print(bar30.pressure());
  Serial.print(",");
  Serial.print(bar30.temperature());
  Serial.print(",");
  Serial.print(event.orientation.x, 2); // THESE WILL LIKELY CHANGE TO ERRORS
  Serial.print(",");
  Serial.print(event.orientation.y, 2);
  Serial.print(",");
  Serial.println(event.orientation.z, 2);
}

/*========================================================================*/
/*------Compute PID Output------------------------------------------------*/
/*========================================================================*/
void runPID(void) {
  /* Compute PID outputs for pitch and roll  */
  pitchPID.SetTunings(pKp,pKi,pKd);
  pitchPID.Compute();
  OutP = pitchOutput;

  rollInput = dz;  // Roll input = z position
  rollPID.SetTunings(rKp,rKi,rKd);
  rollPID.Compute();
  OutR = rollOutput;

  /* Compute combined roll and pitch outputs */
  Out1 = 90 - OutP + OutR;
  Out2 = 90 + OutP + OutR;
}

/*========================================================================*/
/*------Interrupt ISR for Updating Settings-------------------------------*/
/*========================================================================*/
void updateSettings() {
  if(Serial.available() > 0) {
    String temp = Serial.readStringUntil(',');
    logRate = temp.toDouble();
    temp = Serial.readStringUntil(',');
    targetDepth = temp.toDouble();
    temp = Serial.readStringUntil(',');
    targetAltitude = temp.toDouble();
    temp = Serial.readStringUntil(',');
    targetAltitude = temp.toDouble();
    temp = Serial.readStringUntil(',');
    state = temp.toInt();
    temp = Serial.readStringUntil(',');
    pKp = temp.toDouble();
    temp = Serial.readStringUntil(',');
    pKi = temp.toDouble();
    temp = Serial.readStringUntil(',');
    pKd = temp.toDouble();
    temp = Serial.readStringUntil(',');
    rKp = temp.toDouble();
    temp = Serial.readStringUntil(',');
    rKi = temp.toDouble();
    temp = Serial.readStringUntil(',');
    rKd = temp.toDouble();
  }
}

/*========================================================================*/
/*------Display PID I/O (For Debugging)-----------------------------------*/
/*========================================================================*/

void outputPID(void){
  /* Display PID pinput and output values in serial*/
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
}
