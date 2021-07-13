/*====================================
               Libraries
  ====================================
*/
// Arduino
#include <Arduino.h>

// MPU libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// Include motor library
#include "SparkFun_TB6612.h"

/* ======================================
           Define motor constants
   ======================================
*/
// Motor A:
#define AIN1 3
#define AIN2 4
#define PWMA 5
const int offsetA = 1; // Direction offset
// Motor B:
#define BIN1 8
#define BIN2 7
#define PWMB 6
const int offsetB = -1; // Direction offset
// Standby pin
#define STBY 9

/* =======================================
            Define MPU variables
   ======================================
*/
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/*========================================
          Variables for balancing
   =======================================
*/
// Angle and error variables
float currAngle = 0;
float targetAngle = 0;
float prevAngle = 0;
float error = 0;
float errorSum = 0;
float motorPower = 0;
float spMotorPower = 0; // Speed adjusted

// Pid constants
#define Kp  75000   // 40
#define Kd  750       // 0.05
#define Ki  100       // 40

// Time variables
unsigned long int currTime, prevTime = 0;
float sampleTime;

// Control variables
char moveDirection = "S";
float speedMult = 0;
#include "Bluetooth_Macros.h" // App macros for switch
/*=======================================
        Create motor and MPU objects
   ======================================
*/
// MPU
MPU6050 mpu;

// Motors
Motor RMotor = Motor(AIN1, AIN2, PWMA, offsetA, STBY); // Right Motor
Motor LMotor = Motor(BIN1, BIN2, PWMB, offsetB, STBY); // Left Motor


/*====================================
          Interrupt detection
  ====================================
*/
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
/*==================================================================
                           SETUP FUNCTION
   =================================================================
*/

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(9600); // Start Serial Monitor, HC-05 uses 9600

  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // set offsets
  mpu.setXGyroOffset(112);
  mpu.setYGyroOffset(12);
  mpu.setZGyroOffset(-11);
  mpu.setXAccelOffset(-4390);
  mpu.setYAccelOffset(-1294);
  mpu.setZAccelOffset(894);

  // Make sure it works (returns 0):
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}


/*=====================================
               MAIN LOOP
   ====================================
*/


void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) {
    digitalWrite(9, HIGH);
    delay(1000);
    digitalWrite(9, LOW);
    delay(1000);
    return;
  }

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
    /*====================================================================
                           Bluetooth Switch Statement
       ===================================================================
    */
    else if (Serial.available() > 0) {
      moveDirection = Serial.read();
      switch (moveDirection) {
        case STOP:
          targetAngle = 0; // Balance
          break;
        case FORWARD:
          targetAngle = speedMult * 5.0; // 5 deg times multiplier
          break;
        case REVERSE:
          targetAngle = -1.0 * (speedMult * 5.0); // 5 deg times multiplier
          break;
        case LEFT:
          LMotor.drive(-spMotorPower);
          RMotor.drive(spMotorPower);
          break;
        case RIGHT:
          LMotor.drive(spMotorPower);
          RMotor.drive(-spMotorPower);
          break;
        case FORLEFT:
          targetAngle = speedMult * 5.0; // 5 deg times multiplier
          LMotor.drive(spMotorPower / 2);
          RMotor.drive(spMotorPower * 2);
          break;
        case FORRIGHT:
          targetAngle = speedMult * 5.0; // 5 deg times multiplier
          LMotor.drive(spMotorPower * 2);
          RMotor.drive(spMotorPower / 2);
          break;
        case BACKLEFT:
          targetAngle = -1.0 * (speedMult * 5.0); // 5 deg times multiplier
          LMotor.drive(spMotorPower * 2);
          RMotor.drive(spMotorPower / 2);
          break;
        case BACKRIGHT:
          targetAngle = -1.0 * (speedMult * 5.0); // 5 deg times multiplier
          LMotor.drive(spMotorPower * 2);
          RMotor.drive(spMotorPower / 2);
          break;
        case SPEED0:
          speedMult = 0.0;
          break;
        case SPEED1:
          speedMult = .1; // Scale to decrease speed, since from 1-10
          break;
        case SPEED2:
          speedMult = .2; // Scale to decrease speed, since from 1-10
          break;
        case SPEED3:
          speedMult = .3; // Scale to decrease speed, since from 1-10
          break;
        case SPEED4:
          speedMult = .4; // Scale to decrease speed, since from 1-10
          break;
        case SPEED5:
          speedMult = .5; // Scale to decrease speed, since from 1-10
          break;
        case SPEED6:
          speedMult = .6; // Scale to decrease speed, since from 1-10
          break;
        case SPEED7:
          speedMult = .7; // Scale to decrease speed, since from 1-10
          break;
        case SPEED8:
          speedMult = .8; // Scale to decrease speed, since from 1-10
          break;
        case SPEED9:
          speedMult = .9; // Scale to decrease speed, since from 1-10
          break;
        case SPEED10:
          speedMult = 1.0; // Scale to decrease speed, since from 1-10
          break;
        default:
          break;
      }
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {
    //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
  }

  // check for overflow (this should never happen unless our code is too inefficient)
  else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // read a packet from FIFO
    while (fifoCount >= packetSize) { // Lets catch up to NOW, someone is using the dreaded delay()!
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }

    // Get yaw, pitch, and roll angles
    // Although we only need roll
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Calculate error
    currAngle = ypr[2] * DEG_TO_RAD;         // Get roll, which is the robot's angle
    error = currAngle - targetAngle; //Switch? // Find error
    errorSum = errorSum + error;               // Calculate sum of error for I
    errorSum = constrain(errorSum, -300, 300); // Limit Ki

    // Calculate motor power from P, I and D values
    currTime = millis();              // Get time for Ki
    sampleTime = currTime - prevTime; // Find time needed to get values

    // Calculate power with PID
    motorPower = Kp * (error) + Ki * (errorSum) * sampleTime - Kd * (currAngle - prevAngle) / sampleTime;
    spMotorPower = constrain(motorPower, -255, 255); // Limit to avoid overflow
    //spMotorPower = motorPower * speedMult; // Adjust for speed, not used

    //Refresh angles for next iteration
    prevAngle = currAngle;
    prevTime = currTime;

    // Set motors
    forward(RMotor, LMotor, spMotorPower);
    /* Uncomment to print yaw/pitch/roll
        Serial.print("ypr m/Sm/t\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[2] * 180 / M_PI);
        Serial.print("\t \t");
        Serial.print(motorPower);
        Serial.print("\t");
        Serial.print(spMotorPower);
        Serial.print("\t");
        Serial.println(sampleTime);
      //*/
  }
}
