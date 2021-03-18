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
#define LED_PIN 13  //  LED pin (for debugging)

/* States */
#define IDLE 1
#define STABILIZE 2
#define DIVE 3
#define SURFACE 4

/* General Constants */
#define SEA_WATER 1029  // Density of seawater in kg/m^3
#define BAUD_RATE 9600  // Serial baud rate
#define INIT_SERVO_POS 90 // Initial servo position 90 degrees

/* IMU Sample Rate */
#define BNO055_SAMPLERATE_DELAY_MS (100)  // BNO055 sample delay in ms

/**************************************************************************/
/*      Global Variables                                                  */
/**************************************************************************/

uint8_t leak = 0; // 0 = Dry , 1 = Leak

static uint8_t state = 0; // State value

/* Sensor Variables */
double depth, pressure, temperature; // Bar30 data
double dx, dy, dz, ax, ay, az;  // BNO055 IMU data
double altitude;  // Ping sonar data

/* PID Variables */
double pitchSetpoint, pitchInput, pitchOutput, OutP;  // Pitch PID
double rollSetpoint, rollInput, rollOutput, OutR; // Roll PID
double Out1, Out2;  // Servo1 and servo2 output

/* PID Tuning Parameters */
double Kp=1.5, Ki=0, Kd=0; // Proportional, integral, derivative

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

PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT); // Pitch PID
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);  // Roll PID

/**************************************************************************/
/*      Functions                                                         */
/**************************************************************************/

void transmitData(void);  // Transmit data via serial
void initSensor(void);  // Checks if sensors are functioning
void displayCalStatus(void);  // Display IMU calibration (for debugging)
void outputPID(void);   // Display PID I/O (for debugging)

/**************************************************************************/
/*      Program Setup                                                     */
/**************************************************************************/

void setup(void)
{
  /* Initialize serial and I2C */
  Serial.begin(BAUD_RATE);  // Initialize serial at baud rate
  pingSerial.begin(BAUD_RATE);  // Initialize software serial at baud rate
  Wire.begin(); // Initialize I2C

  initSensor(); // Checks if sensors are functioning

  /* Set I/O pinmodes */
  pinMode(LED_PIN, OUTPUT);  // Set LED pin to OUTPUT
  pinMode(LEAK_PIN, INPUT);  // Set leak sensor pin to INPUT
  pinMode(EINT1_PIN, INPUT); // Set interrupt pin to INPUT


   /* Attach servos and write initial position */
  servo1.attach(SERVO_1_PIN);  // Attach servo1 to servo1 pin
  servo2.attach(SERVO_2_PIN); // Attach servo2 to servo2 pin
  servo1.write(INIT_SERVO_POS); // Set servo1 position to initial position
  servo2.write(INIT_SERVO_POS); // Set servo2 position to initial position

  /* Turn on PID and set output limits */
  pitchPID.SetMode(AUTOMATIC); // Set pitch PID mode automatic (ON)
  rollPID.SetMode(AUTOMATIC); // Set roll PID mode automatic (ON)
  pitchPID.SetOutputLimits(-90,90); // Set pitch PID limits to servo limits
  rollPID.SetOutputLimits(-90,90); // Set roll PID limits to servo limits


  bno.setExtCrystalUse(true); // Used for BNO055

  /* Bar 30 sensor setup */
  bar30.setModel(MS5837::MS5837_30BA);
  bar30.setFluidDensity(SEA_WATER); // Set fluid density to sea water

  /* Print sensor data headers to serial */
  Serial.println("Altitude,Depth,Pressure,Temperature,X,Y,Z");

  delay(2000); // Delay for IMU calibration
}

/**************************************************************************/
/*      Main Routine                                                      */
/**************************************************************************/

void loop(void)
{
  state = IDLE;
  goto RUN_BLUEFISH;

  RUN_BLUEFISH:

    switch(state){

      case IDLE:
        goto IDLE_STATE;
        break;

      case STABILIZE:
        goto STABILIZE_STATE;
        break;

      case DIVE:
        goto DIVE_STATE;
        break;

      case SURFACE:
        goto SURFACE_STATE;
        break;

      default:
        goto RUN_BLUEFISH;
    }

    IDLE_STATE:

      goto RUN_BLUEFISH;

    STABILIZE_STATE:
      goto RUN_BLUEFISH;

    DIVE_STATE:
      goto RUN_BLUEFISH;

    SURFACE_STATE:
      goto RUN_BLUEFISH;

  /* Check for leak */
  leak = digitalRead(LEAK_PIN);   // Read the Leak Sensor Pin
  digitalWrite(LED_PIN, leak);  // Sets the LED to the Leak Sensor's Value

  if (leak == 1) {
    Serial.println("Leak Detected!");
  }

  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Read Ping */
  if (ping.update()) {
    altitude = ping.distance();
  }

/* If near surface, set foil angle to 45 degrees  */
  if(altitude <5){
    //set states
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


/* Read Bar30 Sensor Data*/
  bar30.read();
  pressure = bar30.pressure();
  temperature = bar30.temperature();
  depth = bar30.depth();


  /* Wait specified delay before requesting next data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
/*---- End of Main Routine -----------------------------------------------*/


/**************************************************************************/
/*      Function Definitions                                              */
/**************************************************************************/


/*------Check Initialization of Sensors-----------------------------------*/
/*========================================================================*/
void initSensor(void)
{
  /* Sensor initialization checks */

  if(!bno.begin())  //check if BNO055 functioning
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!bar30.init())  //check if Bar30 functioning
  {
    Serial.println("Bar30 Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    while(1);
  }

  if(!ping.initialize()) {  //check if Ping functioning
      Serial.println("\nPing device failed to initialize!");
      Serial.println("Are the Ping rx/tx wired correctly?");
      Serial.print("Ping rx is the green wire, and should be connected to Arduino pin ");
      Serial.print(SRX_PIN);
      Serial.println(" (Arduino tx)");
      Serial.print("Ping tx is the white wire, and should be connected to Arduino pin ");
      Serial.print(STX_PIN);
      Serial.println(" (Arduino rx)");
    while(1);
  }

}

/*========================================================================*/
/*------Transmit Data via Serial------------------------------------------*/
/*========================================================================*/

void transmitData(void)
{
  /* Transmit sensor data */
  Serial.print(ping.distance());
  Serial.print(",");
  Serial.print(bar30.depth());
  Serial.print(",");
  Serial.print(bar30.pressure());
  Serial.print(",");
  Serial.print(bar30.temperature());
  Serial.print(",");
  Serial.print(event.orientation.x, 4);
  Serial.print(",");
  Serial.print(event.orientation.y, 4);
  Serial.print(",");
  Serial.println(event.orientation.z, 4);
}

/*========================================================================*/
/*------Display sensor calibration status (For Debugging)-----------------*/
/*========================================================================*/
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
/*========================================================================*/
/*------Display PID I/O (For Debugging)-----------------------------------*/
/*========================================================================*/

void outputPID(void)
{
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
