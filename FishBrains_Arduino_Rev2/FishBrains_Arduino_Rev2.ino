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
#define ADC_1_PIN A0 // ADC 1 pin for batt voltage measurement (Pin 4 of power sense)
#define ADC_2_PIN A1 // ADC 2 pin for batt current measurement (Pin 3 of power sense)

/* States */
#define IDLE 0
#define DEPTH 1
#define ALTITUDE 2
#define SURFACE 3

/* General Constants */
#define SEA_WATER 997  // Density of seawater in kg/m^3 (997 Fresh/1029 Salt)
#define BAUD_RATE 9600  // Serial baud rate
#define INIT_SERVO_POS 90 // Initial servo position 90 degrees
#define SERVO_LIMIT 18  // Max servo position in degrees (inital + 45)
#define HEIGHT_LIMIT 250  // Range of acceptable height/depth varation (mm)

/* IMU Sample Rate */
#define BNO055_SAMPLERATE_DELAY_MS (100)  // BNO055 sample delay in ms

/**************************************************************************/
/*      Global Variables                                                  */
/**************************************************************************/

uint8_t leak = 0; // 0 = Dry , 1 = Leak
uint8_t leakState = 0; // 0 = LED off , 1 = LED on
uint8_t state = IDLE; // State value
volatile boolean flag = LOW;

int adc1, adc2 = 0; // Variables for ADC values

unsigned long currentTime, lastTime, transmitTime = 0; // For time tracking
int logRate = 0;
double logPeriod = 0;

double minAltitude = 1000; // Minimum distance from sea floor (mm)
double maxDepth = 100000; // Maximum depth

/* Sensor Variables */
double depth, pressure, temperature = 0; // Bar30 data
double dx, dy, dz = 0;  // BNO055 IMU gyro data (y is pitch, z is roll)
double altitude = 0;  // Ping sonar data
double dAltitude, dDepth = 0;
double voltage, current = 0;

/* PID Variables */
double targetDepth, targetAltitude = 0;  // Target values of depth and altitude
double heightSetpoint, heightInput, heightOutput, OutH = 0;  // Height PID
double rollSetpoint, rollInput, rollOutput, OutR = 0; // Roll PID
double output1, output2 = 0;  // Servo1 and servo2 output

/* PID Tuning Parameters */
double hKp, hKi, hKd = 0; // Height proportional, integral, derivative gains
double dKp, dKi, dKd = 0; // Depth proportional, integral, derivative gains
double rKp, rKi, rKd = 0; // Roll proportional, integral, derivative gains
double aKp, aKi, aKd = 0; // Adaptive proportional, integral, derivative gains

unsigned int mDelay = 15; // Delay for servo motors
unsigned int sDelay = 500;  // Short delay
unsigned int lDelay = 2000; // Long delay

unsigned int servoMax = INIT_SERVO_POS + SERVO_LIMIT;
unsigned int servoMin = INIT_SERVO_POS - SERVO_LIMIT;
unsigned int servoRatio = SERVO_LIMIT/HEIGHT_LIMIT; //ratio of servo angle to depth/height range 

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

PID heightPID(&heightInput, &heightOutput, &heightSetpoint, hKp, hKi, hKd, P_ON_M, DIRECT); // Height PID 
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
void isrSettings(void);  // ISR to update settings sent from RPi
void updateSettings (void); 
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

  analogReference(EXTERNAL);  // External ADC ref for more accurate reading
                              // Aref pin is connected to 3.3V out

  /* Initialize serial and I2C */
  Serial.begin(BAUD_RATE);  // Initialize serial at baud rate
  pingSerial.begin(BAUD_RATE);  // Initialize software serial at baud rate
  Wire.begin(); // Initialize I2C

  bno.begin();  // Begin bno sensor
  bar30.init(); // Initialize Bar30 sensor
  ping.initialize(); // Initialize ping sensor
  initSensor(); // Checks if sensors are functioning

    /* Bar 30 sensor setup */
  bar30.setModel(MS5837::MS5837_30BA);
  bar30.setFluidDensity(SEA_WATER); // Set fluid density to sea water

  sensor_t sensor;        // Used for BNO055
  bno.getSensor(&sensor);
  bno.setExtCrystalUse(true); 
  displayCalStatus(); // Wait until BNO055 calibrated (display status on LEDs)
  

  attachInterrupt(digitalPinToInterrupt(EINT1_PIN),isrSettings,RISING);

   /* Attach servos and write initial position */
  servo1.attach(SERVO_1_PIN, 500, 2500);  // Attach servo1 to servo1 pin
  servo1.write(INIT_SERVO_POS); // Set servo1 position to initial position
  delay(sDelay);
  servo2.attach(SERVO_2_PIN, 500, 2500); // Attach servo2 to servo2 pin
  servo2.write(INIT_SERVO_POS); // Set servo2 position to initial position
  delay(mDelay);

  /* Turn on PID and set output limits*/ 
  heightPID.SetMode(AUTOMATIC); // Set height PID mode automatic (ON)
  rollPID.SetMode(AUTOMATIC); // Set roll PID mode automatic (ON)
  heightPID.SetOutputLimits(-HEIGHT_LIMIT,HEIGHT_LIMIT); // Set Height PID limits (+-250 mm)
  rollPID.SetOutputLimits(-SERVO_LIMIT,SERVO_LIMIT); // Set roll PID limits to servo limits
  
  
  /* Bar 30 sensor setup 
  bar30.setModel(MS5837::MS5837_30BA);
  bar30.setFluidDensity(SEA_WATER); // Set fluid density to sea water

  sensor_t sensor;        // Used for BNO055
  bno.getSensor(&sensor);
  bno.setExtCrystalUse(true); 
  */
  
  delay(1000); // Delay for IMU calibration
}

/**************************************************************************/
/*      Main Routine                                                      */
/**************************************************************************/

void loop(void){

  //displayCalStatus(); // Wait until BNO055 calibrated (display status on LEDs)

  goto RUN_BLUEFISH;
  

  RUN_BLUEFISH:
     
    if(flag==HIGH){
      updateSettings();
    }else{
    }

    
    /* Check for leak */
    leak = digitalRead(LEAK_PIN);// Read the Leak Sensor Pin

    if (leak == HIGH) { // If leak detected
      state = DEPTH;  // Set state to DEPTH mode
      leakWarningFlash(); // Flash LEDs indicating leak
      goto MODE_SWITCH;
    }
    
    logPeriod = (1/logRate)*1000; // Convert log rate in Hz to period in milliseconds
    currentTime = millis(); // Set current time
    
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
      delay(mDelay);
      goto RUN_BLUEFISH;

    DEPTH_MODE:
      
      if (leak == HIGH) { // If leak detected
        targetDepth = 0;  // Set target depth to 0 (surface)
      }

      if((currentTime-transmitTime)>=logPeriod) {  // Check if time to transmit data
      readSensors();
      transmitTime = currentTime;
      transmitData(); // Transmit data to Raspberry Pi
      }
      else{
      readSensors();  // Read sensor data
      }

      if(altitude <= minAltitude) { // Check if close to seafloor
        servo1.write(servoMin); // Set servo1 position to min
        servo2.write(servoMax); // Set servo1 position to max
        delay(mDelay);
      } else if(depth >= maxDepth) { // Check if maximum depth
        servo1.write(servoMin); // Set servo1 position to min
        servo2.write(servoMax); // Set servo1 position to max
        delay(mDelay);
      } else {
        heightSetpoint = targetDepth; // Set height setpoint to target depth
        heightInput = depth; // Set PID height input to depth
        runPID(); // Run PID to computed servo outputs
        servo1.write(output1); // Write output to servo1
        servo2.write(output2); // Write output to servo2
        delay(mDelay);
      }

      goto RUN_BLUEFISH;
    
    ALTITUDE_MODE:
      
      if((currentTime-transmitTime)>=logPeriod) {  // Check if time to transmit data
      readSensors();
      transmitTime = currentTime;
      transmitData(); // Transmit data to Raspberry Pi
      }else{
      readSensors();
      }

      if(altitude < minAltitude) { // Check if close to seafloor
        servo1.write(servoMin); // Set servo1 position to min
        servo2.write(servoMax); // Set servo1 position to max
        delay(mDelay);
      } else if(depth >= maxDepth) { // Check if maximum depth
        servo1.write(servoMin); // Set servo1 position to min
        servo2.write(servoMax); // Set servo1 position to max
        delay(mDelay);
      } else {
        heightSetpoint = targetAltitude; // Set height setpoint to target altitude
        heightInput = altitude; // Set PID height input to altitude
        runPID(); // Run PID to computed servo outputs
        servo1.write(output1); // Write output to servo1
        servo2.write(output2); // Write output to servo2
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
  int bnoC, bar30C, pingC = 0;  // Sensor status: 0 = connected, 1 = disconnected

  if(!bno.begin()){  //check if BNO055 functioning
    bnoC = 1;
  }else{
    bnoC = 0; 
  }
  if(!bar30.init()) {  //check if Bar30 functioning
    bar30C  = 1;
  }else{
    bar30C = 0; 
  }
  if(!ping.initialize()) {  //check if Ping functioning
    pingC = 1;
  }else{
    pingC = 0;
  }
   /* Blink LED while sensor is not initiailized */
   /* LED1 = BNO055, LED2 = Bar30, LED3 = Ping */
  while((bnoC==1)||(bar30C==1)||(pingC==1)) {
    if(bnoC == 1) {
      digitalWrite(LED_1_PIN,HIGH);
    }
    if(bar30C == 1) {
      digitalWrite(LED_2_PIN,HIGH);
    }
    if(pingC == 1) {
      digitalWrite(LED_3_PIN,HIGH);
    }
    delay(sDelay);
    digitalWrite(LED_1_PIN,LOW);
    digitalWrite(LED_2_PIN,LOW);
    digitalWrite(LED_3_PIN,LOW);
    delay(sDelay);
  }
}

/*========================================================================*/
/*------Display sensor calibration status (For Debugging)-----------------*/
/*========================================================================*/
void displayCalStatus(void) {
  uint8_t system, gyro, accel, mag = 0;
  
  /* While all values not calibrated, get calibration status.*/
  /* 3 means 'fully calibrated" and requires system > 0  */
  /* When fully calibrated, all LEDs are on. */
  /* (LED1 = gyro, LED2 = accel, LED3 = mag) */
  while(!((gyro==3)&&(accel== 3) && (mag == 3) && (system== 1))){
    
    /* Get the four calibration values (0..3) */
    bno.getCalibration(&system, &gyro, &accel, &mag);
      if(gyro==3){
        digitalWrite(LED_1_PIN,HIGH);
      }else{
        digitalWrite(LED_1_PIN,LOW);
      }
      if(accel==3){
        digitalWrite(LED_2_PIN,HIGH);
      }else{
        digitalWrite(LED_2_PIN,LOW);
      }
      if(mag==3){
        digitalWrite(LED_3_PIN,HIGH);
      }else{
        digitalWrite(LED_3_PIN,LOW);
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
  pressure = bar30.pressure()*10; //read pressure and convert to kPa
  temperature = bar30.temperature();  // temperature in degrees celcius
  depth = bar30.depth()*1000;  // depth in mm
  dDepth = targetDepth-depth; // error in depth

  /* Read BNO055 sensor data */

   /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  /* Store data */
  dx = event.orientation.x;
  dy = event.orientation.y;
  dz = event.orientation.z;

  /* Read Ping sensor Data*/
  if (ping.update()) {
    altitude = ping.distance(); //get distance in mm
    dAltitude = targetAltitude-altitude;  //determine error in altitude (mm)
  }

  adc1 = analogRead(ADC_1_PIN);  // Perform ADC on A0 (batt voltage)
  adc2 = analogRead(ADC_2_PIN); // Perform ADC on A1 (batt current)
 
  /* Using 3.3V connected to AREF pin */
  voltage = adc1*(3.3/1024)*11.0; // convert 10 bit number to volts
  current = (adc2*(3.3/1024)-0.33)*38.8788; // convert 10 bit number to amps
 
  /* Uncomment if not using 3.3V AREF jumper (ie 5V ref) */ /*
  voltage = adc1*(5.0/1024)*11.0; // convert 10 bit number to volts
  current = (adc2*(5.0/1024)-0.33)*38.8788; // convert 10 bit number to amps
  */

}

/*========================================================================*/
/*------Transmit Data via Serial------------------------------------------*/
/*========================================================================*/
void transmitData(void) {
  /* Transmit sensor data */
  Serial.print(altitude/1000);
  Serial.print(",");
  Serial.print(dAltitude/1000); // Error in altitude
  Serial.print(",");
  Serial.print(depth/1000);
  Serial.print(",");
  Serial.print(dDepth/1000); // Error in altitude
  Serial.print(",");
  Serial.print(pressure);
  Serial.print(",");
  Serial.print(temperature);
  Serial.print(",");
  Serial.print(dx); // THESE WILL LIKELY CHANGE TO ERRORS
  Serial.print(",");
  Serial.print(dy);
  Serial.print(",");
  Serial.print(dz);
  Serial.print(",");
  Serial.print(voltage); // Error in altitude
  Serial.print(",");
  Serial.print(current); // Error in altitude
  Serial.print(",");
  Serial.print(output1);
  Serial.print(",");
  Serial.print(output2); // Error in altitude
  Serial.print(",");
  Serial.println(state); // Error in altitude
}

/*========================================================================*/
/*------Compute PID Output------------------------------------------------*/
/*========================================================================*/
void runPID(void) {
  /* Compute PID outputs for height and roll  */
  if(state==DEPTH){
    heightPID.SetTunings(dKp,dKi,dKd);
  }
  if(state==ALTITUDE){
    heightPID.SetTunings(hKp,hKi,hKd);
  }
  
  heightPID.Compute();
  OutH = heightOutput;
  

  rollInput = dz;  // Roll input = z position
  rollPID.SetTunings(rKp,rKi,rKd);
  rollPID.Compute();
  OutR = rollOutput;

  /* Compute combined roll and height outputs */
  output1 = 90 - (servoRatio*OutH) + OutR;  // Convert height output to angular and add roll angle
  output2 = 90 + (servoRatio*OutH) + OutR;
}

/*========================================================================*/
/*------Interrupt ISR for Updating Settings-------------------------------*/
/*========================================================================*/
void isrSettings() {
  flag = !flag;
}


void updateSettings() {
  if(Serial.available() > 0) {
    String temp = Serial.readStringUntil(',');
    temp = Serial.readStringUntil(',');
    logRate = temp.toDouble();
    temp = Serial.readStringUntil(',');
    state = temp.toInt();
    temp = Serial.readStringUntil(',');
    targetDepth = (temp.toDouble())*1000;
    temp = Serial.readStringUntil(',');
    targetAltitude = (temp.toDouble())*1000;
    temp = Serial.readStringUntil(',');
    rKp = temp.toDouble();
    temp = Serial.readStringUntil(',');
    rKi = temp.toDouble();
    temp = Serial.readStringUntil(',');
    rKd = temp.toDouble();
    temp = Serial.readStringUntil(',');
    hKp = temp.toDouble();
    temp = Serial.readStringUntil(',');
    hKi = temp.toDouble();
    temp = Serial.readStringUntil(',');
    hKd = temp.toDouble();
    temp = Serial.readStringUntil(',');
    dKp = temp.toDouble();
    temp = Serial.readStringUntil(',');
    dKi = temp.toDouble();
    temp = Serial.readStringUntil(',');
    dKd = temp.toDouble();
    Serial.println("ISR FINISHED BITCH");
    // temp = Serial.readStringUntil(',');
    /* Uncomment if using adaptive tuning */
    /*
    aKp = temp.toDouble();
    temp = Serial.readStringUntil(',');
    aKi = temp.toDouble();
    temp = Serial.readStringUntil(',');
    aKd = temp.toDouble();
    */
  }
}



/*========================================================================*/
/*------Display PID I/O (For Debugging)-----------------------------------*/
/*========================================================================*/

void outputPID(void){
  /* Display PID pinput and output values in serial*/
  Serial.print("Height PID Input: ");
  Serial.println(heightInput);
  Serial.print("Height PID Output: ");
  Serial.println(heightOutput);
  Serial.print("Roll PID Input: ");
  Serial.println(rollInput);
  Serial.print("Roll PID Output: ");
  Serial.println(rollOutput);
  Serial.print("PID OutH: ");
  Serial.println(OutH);
  Serial.print("PID OutR: ");
  Serial.println(OutR);
  Serial.print("Output1: ");
  Serial.println(output1);
  Serial.print("Output2: ");
  Serial.println(output2);
}
