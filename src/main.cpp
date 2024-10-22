#include <Arduino.h>
#include <PID_v1.h>
#include <Ramp.h> // Include the Ramp library

// Motor control pins
#define LMF 5
#define LMB 6
#define RMF 11
#define RMB 10

#define sl1 0
#define sl2 1

int modet = 2;

int mode = 0;
// Encoder pins
#define LEFT_ENCODER 2
#define RIGHT_ENCODER 3

// Motor configurations
int maxSpeed = 50;
bool leftDir = false; // Direction flags
bool rightDir = false;

int previousMode = -1; // -1 indicates no previous mode

int errorl, errorr;
// Encoder variables
volatile long leftEncoderPos = 0;
volatile long rightEncoderPos = 0;
int targetLeftPos = 100;   // forwalking
int targetRightPos = -100; // forwalking

int handshakepos = 150;

int currentLeftTarget = targetLeftPos;
int currentRightTarget = targetRightPos;

// PID variables
double leftSetpoint, leftInput, leftOutput;
double rightSetpoint, rightInput, rightOutput;
double Kp = 4, Ki = 0.0, Kd = 0.005;

PID leftMotorPID(&leftInput, &leftOutput, &leftSetpoint, Kp, Ki, Kd, DIRECT);
PID rightMotorPID(&rightInput, &rightOutput, &rightSetpoint, Kp, Ki, Kd, DIRECT);

// Ramp object for motor speed control
ramp motorRamp;
int rampDuration = 1000;
int rampTargetSpeed = 0;

// Timer variables
unsigned long walkingStartTime;
bool isWalking = true;

char x;
// Function to handle encoder interrupts
void handleLeftEncoder()
{
    leftDir ? leftEncoderPos-- : leftEncoderPos++;
}

void handleRightEncoder()
{
    rightDir ? rightEncoderPos-- : rightEncoderPos++;
}

void handshake();
// Function to control motor speed and direction
void controlMotor(int leftSpeed, int rightSpeed)
{
    leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

    // Left motor control
    if (leftSpeed > 0)
    {
        analogWrite(LMF, leftSpeed);
        analogWrite(LMB, 0);
        leftDir = false;
    }
    else
    {
        analogWrite(LMF, 0);
        analogWrite(LMB, -leftSpeed);
        leftDir = true;
    }

    // Right motor control
    if (rightSpeed > 0)
    {
        analogWrite(RMF, rightSpeed);
        analogWrite(RMB, 0);
        rightDir = false;
    }
    else
    {
        analogWrite(RMF, 0);
        analogWrite(RMB, -rightSpeed);
        rightDir = true;
    }
}

void setup()
{
    Serial.begin(9600);

    // Motor setup
    pinMode(LMF, OUTPUT);
    pinMode(LMB, OUTPUT);
    pinMode(RMF, OUTPUT);
    pinMode(RMB, OUTPUT);


    // Encoder setup
    pinMode(LEFT_ENCODER, INPUT);
    pinMode(RIGHT_ENCODER, INPUT);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), handleLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), handleRightEncoder, RISING);

    // PID setup
    leftSetpoint = currentLeftTarget;
    rightSetpoint = currentRightTarget;
    leftMotorPID.SetMode(AUTOMATIC);
    leftMotorPID.SetOutputLimits(-255, 255);
    rightMotorPID.SetMode(AUTOMATIC);
    rightMotorPID.SetOutputLimits(-255, 255);

    // Initialize ramp

    // Start timer
    walkingStartTime = millis();
}

// Walking motion control
void walking()
{

    leftInput = leftEncoderPos;
    rightInput = rightEncoderPos;

    errorl = currentLeftTarget - leftEncoderPos;
    errorr = currentRightTarget - rightEncoderPos;

    // Adjust the target positions when within error threshold
    if (abs(currentLeftTarget - leftEncoderPos) < 5 && abs(currentRightTarget - rightEncoderPos) < 5)
    {
        if (currentLeftTarget == targetLeftPos)
        {
            currentLeftTarget = targetRightPos;
            currentRightTarget = targetLeftPos;
        }
        else
        {
            currentLeftTarget = targetLeftPos;
            currentRightTarget = targetRightPos;
        }
        leftSetpoint = currentLeftTarget;
        rightSetpoint = currentRightTarget;
    }

    // Compute PID output
    leftMotorPID.Compute();
    rightMotorPID.Compute();

    // Ramp motor speeds based on PID outputs
    controlMotor(leftOutput, rightOutput);
    //  Serial.print(leftEncoderPos);
    //  Serial.print("\t");
    //  Serial.print(rightEncoderPos);
    //  Serial.print("\t");
    //  Serial.print(leftSetpoint);
    //  Serial.print("\t");
    //  Serial.print(rightSetpoint);
    //  Serial.print("\t");
    //  Serial.print(errorl);
    //  Serial.print("\t");
    //  Serial.println(errorr);
}

// Return to zero position
void returnToZero()
{
    // Set the target setpoints to zero
    leftSetpoint = 0;
    rightSetpoint = 0;

    // Update the encoder readings
    leftInput = leftEncoderPos;
    rightInput = rightEncoderPos;

    // Compute PID output
    leftMotorPID.Compute();
    rightMotorPID.Compute();

    // Check if the motors are within a small threshold of the target (zero position)
    if (abs(leftEncoderPos) < 5 && abs(rightEncoderPos) < 5)
    {
        // Stop the motors if close enough to the target
        controlMotor(0, 0);
        return; // Exit the function early to avoid running the motors further
    }

    // Otherwise, continue controlling the motors using PID output
    controlMotor(leftOutput, rightOutput);
}

void handshake()
{
    // Set the target setpoints
    leftSetpoint = handshakepos; // Keep the left motor stationary
    rightSetpoint = 0;           // Move the right motor to position 150

    // Update the encoder readings
    leftInput = leftEncoderPos;
    rightInput = rightEncoderPos;

    // Compute PID output
    leftMotorPID.Compute();
    rightMotorPID.Compute();

    // Check if the right motor is near the target (150)
    if (abs(rightEncoderPos - 150) < 5)
    {
        // Stop the motors if close enough to the target
        controlMotor(0, 0);
        return; // Exit the function early to avoid running the motors further
    }

    // Keep the left motor stationary and control the right motor with PID output
    controlMotor(leftOutput, 0);
}

void loop()
{
    if(Serial.available() > 0){
        x = Serial.read();
    }
    if(x == '1'){
        mode = 1;
    }else if (x == '2'){
        mode = 0;
    }else if (x == '3'){
        mode = 2;
    }

    // Check for mode change
    if (mode != previousMode)
    {
        // Reset setpoints on mode change
        if (mode == 0)
        {
            // Set initial walking setpoints
            leftSetpoint = currentLeftTarget;
            rightSetpoint = currentRightTarget;
        }
        else if (mode == 1)
        {
            // Set return-to-zero setpoints
            leftSetpoint = 0;
            rightSetpoint = 0;
        }

        // Update previous mode
        previousMode = mode;
    }

    // Execute functions based on the current mode
    if (mode == 0)
    {
        walking();
    }
    else if (mode == 1)
    {
        returnToZero();
    }else if (mode == 2){
        handshake();
    }
    //Serial.println(mode);
}

