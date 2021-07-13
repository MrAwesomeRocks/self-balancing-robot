/*
=============
  Libraries
=============
*/
// Arduino
#include <Arduino.h>

// MPU libraries
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

// Motor libraries
#include <L298N.h>
#include <PID_v1.h>
#include "motor_utils.h"

// Bluetooth
#include "bluetooth.h"

/*
===============
  Motor Setup
===============
*/
// Motor A:
#define IN1_A 3 // Direction Pin 1
#define IN2_A 4 // Direction Pin 2
#define EN_A 5  // PWM Speed Pin
// Motor B:
#define IN1_B 8 // Direction Pin 1
#define IN2_B 7 // Direction Pin 2
#define EN_B 6  // PWM Speed Pin

// Create motors
L298N RMotor(EN_A, IN1_A, IN2_A); // Right Motor
L298N LMotor(EN_B, IN1_B, IN2_B); // Left Motor

/*
=============
  MPU Setup
=============
*/
#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// MPU
MPU6050 mpu;

// Interrupt detection
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

/*
=============
  PID setup
=============
 */
// Angle and error variables
double angle = 0;        // Current angle (roll) of the robot
double targetAngle = 0;  // Target angle of the robot
double motorPower = 0;   // Motor power output by PID
double spMotorPower = 0; // Motor power adjusted by speedMult

// Pid constants
#define Kp 50  // 75000 // 40
#define Ki 1.4 // 100   // 40
#define Kd 60  // 750   // 0.05

// Control variables
char moveDirection = 'S'; // Movement direction of robot (from BT)
float speedMult = 1;      // Speed mutliplier of robot (from BT)

// PID
PID pid(&angle, &motorPower, &targetAngle, Kp, Ki, Kd, P_ON_E, DIRECT);

/*
===============
  Misc. Setup
===============
 */
// Log setup
bool printData = false; // Send info to serial
short logIter = 0;      // Amount of lines already printed
#define LOG_SPEED_DEC 5 // How much the log is slowed down by

/*
====================================
               Setup
====================================
*/
void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(9600); // Start Serial Monitor, HC-05 uses 9600
  pinMode(INTERRUPT_PIN, INPUT);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // set offsets
  mpu.setXGyroOffset(112);
  mpu.setYGyroOffset(12);
  mpu.setZGyroOffset(-11);
  mpu.setXAccelOffset(-4390);
  mpu.setYAccelOffset(-1294);
  mpu.setZAccelOffset(894);

  // Make sure it works (returns 0):
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    // Setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);

    // Info
    Serial.println(F("Ready!"));
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

/*
====================================
            MAIN LOOP
====================================
*/
void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady)
  {
    digitalWrite(9, HIGH);
    delay(1000);
    digitalWrite(9, LOW);
    delay(1000);
    return;
  }

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    // Compute PID
    pid.Compute();
    spMotorPower = motorPower * speedMult;
    drive(RMotor, LMotor, spMotorPower);

    // Check for interrupt
    if (mpuInterrupt && fifoCount < packetSize)
    {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
    /*
    ====================================================================
                         Bluetooth Switch Statement
     ===================================================================
    */
    else if (Serial.available() > 0)
    {
      moveDirection = Serial.read();
      switch (moveDirection)
      {
      case STOP:
        targetAngle = 0; // Balance
        break;
      case FORWARD:
        targetAngle = speedMult * 5.0; // 5 deg times multiplier
        break;
      case BT_REVERSE:
        targetAngle = -(speedMult * 5.0); // 5 deg times multiplier
        break;
      case LEFT:
        drive(LMotor, -spMotorPower);
        drive(RMotor, spMotorPower);
        break;
      case RIGHT:
        drive(LMotor, spMotorPower);
        drive(RMotor, -spMotorPower);
        break;
      case FORLEFT:
        targetAngle = speedMult * 5.0; // 5 deg times multiplier
        drive(LMotor, spMotorPower / 2);
        drive(RMotor, spMotorPower * 2);
        break;
      case FORRIGHT:
        targetAngle = speedMult * 5.0; // 5 deg times multiplier
        drive(LMotor, spMotorPower * 2);
        drive(RMotor, spMotorPower / 2);
        break;
      case BACKLEFT:
        targetAngle = -1.0 * (speedMult * 5.0); // 5 deg times multiplier
        drive(LMotor, spMotorPower * 2);
        drive(RMotor, spMotorPower / 2);
        break;
      case BACKRIGHT:
        targetAngle = -1.0 * (speedMult * 5.0); // 5 deg times multiplier
        drive(LMotor, spMotorPower * 2);
        drive(RMotor, spMotorPower / 2);
        break;
      case SPEED0:
        speedMult = 0.0;
        break;
      case SPEED1:
        speedMult = 0.1; // Scale to decrease speed, since from 1-10
        break;
      case SPEED2:
        speedMult = 0.2; // Scale to decrease speed, since from 1-10
        break;
      case SPEED3:
        speedMult = 0.3; // Scale to decrease speed, since from 1-10
        break;
      case SPEED4:
        speedMult = 0.4; // Scale to decrease speed, since from 1-10
        break;
      case SPEED5:
        speedMult = 0.5; // Scale to decrease speed, since from 1-10
        break;
      case SPEED6:
        speedMult = 0.6; // Scale to decrease speed, since from 1-10
        break;
      case SPEED7:
        speedMult = 0.7; // Scale to decrease speed, since from 1-10
        break;
      case SPEED8:
        speedMult = 0.8; // Scale to decrease speed, since from 1-10
        break;
      case SPEED9:
        speedMult = 0.9; // Scale to decrease speed, since from 1-10
        break;
      case SPEED10:
        speedMult = 1.0; // Scale to decrease speed, since from 1-10
        break;
      case DATA:
        printData = true;
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
  if (fifoCount < packetSize)
  {
    //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
  }

  // check for overflow (this should never happen unless our code is too inefficient)
  else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
  {
    // read a packet from FIFO
    while (fifoCount >= packetSize)
    { // Lets catch up to NOW, someone is using the dreaded delay()!
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

    // Calculate angle
    angle = ypr[2] * RAD_TO_DEG; // Get roll, which is the robot's angle

    // Print some debug info
    if (printData && logIter % LOG_SPEED_DEC == 0)
    {
      if (logIter == 0)
      {
        Serial.println(F("   \t     \t    \t│"));
        Serial.println(F("Yaw\tPitch\tRoll\t│\tmotorPower\tspMotorPower\tspeedMult"));
      }
      Serial.print(ypr[0] * RAD_TO_DEG);
      Serial.print(F("\t"));
      Serial.print(ypr[1] * RAD_TO_DEG);
      Serial.print(F("\t"));
      Serial.print(angle);
      Serial.print(F("\t│\t"));
      Serial.print(motorPower);
      Serial.print(F("\t\t"));
      Serial.print(spMotorPower);
      Serial.print(F("\t\t\t"));
      Serial.println(speedMult);
    }
    logIter++;
    if (logIter == 10 * LOG_SPEED_DEC)
    {
      logIter = 0;
    }
  }
}
