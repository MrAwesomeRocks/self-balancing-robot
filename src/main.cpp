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
#include "utils.hpp"

// Bluetooth
#include "bluetooth.hpp"
#include <AsyncDelay.h>

// Ultrasonic
#include <HCSR04.h>

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
double angle = 0;           // Current angle (roll) of the robot
double targetAngle = 0;     // Target angle of the robot
double motorPower = 0;      // Motor power output by PID
bool drivingEnabled = true; // Driving gets disabled if the robot is about to fall over
#define DRIVE_ANGLE 5.0     // Max amount of change in the target angle for driving

// PID constants
#define PID_CONST_MAX 50.0 // Max value for the PID constants
double Kp, Ki, Kd;         // PID constant
int rawKp, rawKi, rawKd;   // Used to calculate corresponding PID constant
// 75000 // 40
// 100   // 40
// 750   // 0.05

// PID pins
#define Kp_PIN A0
#define Ki_PIN A1
#define Kd_PIN A2

// Control variables
char moveDirection = 'S';              // Movement direction of robot (from BT)
char oldMoveDirection = moveDirection; // Used to determine when to blink LED
float speedMult = 0.5;                 // Speed mutliplier of robot (from BT). Default to 0.5 for easier start

// PID
PID pid(&angle, &motorPower, &targetAngle, Kp, Ki, Kd, P_ON_E, REVERSE);

/*
============
  US setup
============
 */
// Pins
#define TRIG_PIN 9                              // US pin connected to TRIG
#define ECHO_COUNT 2                            // Amount of US sensors
byte *ECHO_PINS = new byte[ECHO_COUNT]{10, 11}; // US sensor ECHO pins

// Control variables
#define MAX_OBJ_DISTANCE 10 // Max distance from any object
#define PING_PAUSE 1000     // Pause for 1000ms between pings
AsyncDelay pingTimer;       // Timer for pinging

/*
===============
  Misc. Setup
===============
 */
// Log setup
#define LOG_SPEED_DEC 5 // How much the log is slowed down by
#define LOG_CHAR 'z'    // Key that enables/disables logging
bool printData = false; // Send info to serial
short logIter = 0;      // Amount of lines already printed

// LED Blinker
AsyncDelay ledBlinker;

/*
====================================
               Setup
====================================
*/
void setup()
{
  // Set up LED
  pinMode(LED_BUILTIN, OUTPUT);
  ledBlinker.start(100, AsyncDelay::MILLIS);
  ledBlinker.expire();

  // Set up US sensors
  HCSR04.begin(TRIG_PIN, ECHO_PINS, ECHO_COUNT);
  pingTimer.start(1000, AsyncDelay::MILLIS);

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
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    return;
  }

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    // Compute PID
    pid.Compute();
    utils::drive(RMotor, LMotor, motorPower);

    // Update PID constants
    // Read values
    rawKp = analogRead(Kp_PIN);
    rawKi = analogRead(Ki_PIN);
    rawKd = analogRead(Kd_PIN);

    // Map to Kp/Ki/Kd
    Kp = utils::map(rawKp, 0, 1023, 0.0, PID_CONST_MAX);
    Ki = utils::map(rawKi, 0, 1023, 0.0, PID_CONST_MAX);
    Kd = utils::map(rawKd, 0, 1023, 0.0, PID_CONST_MAX);
    pid.SetTunings(Kp, Ki, Kd);

    //$ Make sure the robot is not about to fall over from driving
    if (drivingEnabled)
    {
      // Robot can drive
      if (abs(targetAngle - angle) > DRIVE_ANGLE)
      {
        // Robot is at an angle twice as much as the max drive angle -- BAD!!
        drivingEnabled = false;
        targetAngle = 0;
      }
    }
    else
    {
      // Driving has been disabled for some reason
      if (!(abs(targetAngle - angle) > DRIVE_ANGLE))
      {
        // Robot is balanced again
        drivingEnabled = true;
      }
    }

    // Stop LED Blinking
    if (ledBlinker.isExpired())
    {
      digitalWrite(LED_BUILTIN, LOW);
    }

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
    else if (drivingEnabled && Serial.available() > 0)
    {
      oldMoveDirection = moveDirection;
      moveDirection = Serial.read();
      switch (moveDirection)
      {
      case BT_STOP:
        targetAngle = 0; // Balance
        break;
      case BT_FORWARD:
        targetAngle = speedMult * DRIVE_ANGLE;
        break;
      case BT_REVERSE:
        targetAngle = -(speedMult * DRIVE_ANGLE);
        break;
      case BT_LEFT:
        utils::drive(LMotor, -motorPower);
        utils::drive(RMotor, motorPower);
        break;
      case BT_RIGHT:
        utils::drive(LMotor, motorPower);
        utils::drive(RMotor, -motorPower);
        break;
      case BT_FORLEFT:
        targetAngle = speedMult * DRIVE_ANGLE;
        utils::drive(LMotor, motorPower / 2);
        utils::drive(RMotor, motorPower * 2);
        break;
      case BT_FORRIGHT:
        targetAngle = speedMult * DRIVE_ANGLE;
        utils::drive(LMotor, motorPower * 2);
        utils::drive(RMotor, motorPower / 2);
        break;
      case BT_BACKLEFT:
        targetAngle = -(speedMult * DRIVE_ANGLE);
        utils::drive(LMotor, motorPower * 2);
        utils::drive(RMotor, motorPower / 2);
        break;
      case BT_BACKRIGHT:
        targetAngle = -(speedMult * DRIVE_ANGLE);
        utils::drive(LMotor, motorPower * 2);
        utils::drive(RMotor, motorPower / 2);
        break;
      case BT_SPEED0:
        speedMult = 0.0;
        break;
      case BT_SPEED1:
        speedMult = 0.1; // Scale to decrease speed, since from 1-10
        break;
      case BT_SPEED2:
        speedMult = 0.2; // Scale to decrease speed, since from 1-10
        break;
      case BT_SPEED3:
        speedMult = 0.3; // Scale to decrease speed, since from 1-10
        break;
      case BT_SPEED4:
        speedMult = 0.4; // Scale to decrease speed, since from 1-10
        break;
      case BT_SPEED5:
        speedMult = 0.5; // Scale to decrease speed, since from 1-10
        break;
      case BT_SPEED6:
        speedMult = 0.6; // Scale to decrease speed, since from 1-10
        break;
      case BT_SPEED7:
        speedMult = 0.7; // Scale to decrease speed, since from 1-10
        break;
      case BT_SPEED8:
        speedMult = 0.8; // Scale to decrease speed, since from 1-10
        break;
      case BT_SPEED9:
        speedMult = 0.9; // Scale to decrease speed, since from 1-10
        break;
      case BT_SPEED10:
        speedMult = 1.0; // Scale to decrease speed, since from 1-10
        break;
      case BT_EXTRA_ON:
        printData = true;
        break;
      case BT_EXTRA_OFF:
        printData = false;
        break;
      case LOG_CHAR:
        printData = !printData;
        break;
      default:
        break;
      }

      // Blink LED
      if (oldMoveDirection != moveDirection)
      {
        digitalWrite(LED_BUILTIN, HIGH);
        ledBlinker.restart();
      }
    }

    //$ Collision avoidance
    // Get distances
    if (pingTimer.isExpired())
    {
      double *distances = HCSR04.measureDistanceCm();
      if (distances[0] < MAX_OBJ_DISTANCE)
      {
        targetAngle = -DRIVE_ANGLE;
      }
      else if (distances[1] < MAX_OBJ_DISTANCE)
      {
        targetAngle = DRIVE_ANGLE;
      }
      pingTimer.restart();
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
    angle = degrees(ypr[2]); // Get roll, which is the robot's angle

    // Print some debug info
    if (printData && logIter % LOG_SPEED_DEC == 0)
    {
      if (logIter == 0)
      {
        Serial.println(F("   \t     \t    \t│\t  \t  \t  \t│"));
        Serial.println(F("Yaw\tPitch\tRoll\t│\tKp\tKi\tKd\t│\tmotorPower\tmoveDirection\ttargetAngle"));
      }
      Serial.print(degrees(ypr[0]));
      Serial.print(F("\t"));
      Serial.print(degrees(ypr[1]));
      Serial.print(F("\t"));
      Serial.print(angle);
      Serial.print(F("\t│\t"));
      Serial.print(Kp);
      Serial.print(F("\t"));
      Serial.print(Ki);
      Serial.print(F("\t"));
      Serial.print(Kd);
      Serial.print(F("\t│\t"));
      Serial.print(motorPower);
      Serial.print(F("\t\t"));
      Serial.print(moveDirection);
      Serial.print(F("\t\t"));
      Serial.println(targetAngle);
    }
    logIter++;
    if (logIter == 10 * LOG_SPEED_DEC)
    {
      logIter = 0;
    }
  }
}
