#include <PID_v1.h>
#include <Arduino.h>

#define lmf 5
#define lmb 6
#define rmf 11
#define rmb 10

// Single encoder pin (using only one channel, e.g., channel A)
#define left_motor_encoder 2
#define right_motor_encoder 3

// Encoder variables
volatile long encoderPos = 0;  // Current encoder position
int targetPosA = 1000;  // Encoder position for "to" position
int targetPosB = -1000; // Encoder position for "fro" position
int currentTarget = targetPosA;  // Current target position

// PID control variables
double setpoint, input, output;
double Kp = 1.2, Ki = 0.01, Kd = 0.05;  // PID constants
PID motorPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Function to handle encoder A interrupts
void handleEncoderA() {
    encoderPos++;  // Increment position on each pulse (assuming forward motion)
}

// Function to control motor direction and speed
void controlMotor(int speed) {
    speed = constrain(speed, -255, 255);  // Use constrain to limit speed
    if (speed > 0) {
        analogWrite(lmf, speed);
        analogWrite(lmb, 0);
    } else if (speed < 0) {
        analogWrite(lmf, 0);
        analogWrite(lmb, -speed);
    } else {
        analogWrite(lmf, 0);
        analogWrite(lmb, 0); // Stop the motor
    }
}

void setup() {
    Serial.begin(9600);

    // Motor control pins setup
    pinMode(lmf, OUTPUT);
    pinMode(lmb, OUTPUT);

    // Encoder pin setup
    pinMode(left_motor_encoder, INPUT);
    attachInterrupt(digitalPinToInterrupt(left_motor_encoder), handleEncoderA, RISING);  // Trigger on rising edge

    // PID setup
    setpoint = currentTarget;
    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(-255, 255);  // Limit output to motor's PWM range
}

void loop() {
    // Read current encoder position as input to the PID
    input = encoderPos;

    // Run PID calculation
    motorPID.Compute();

    // Control the motor based on PID output
    controlMotor(output);

    // Check if the position is close to the target and switch direction
    if (abs(encoderPos - currentTarget) < 5) {
        // Change target for to and fro motion
        if (currentTarget == targetPosA) {
            currentTarget = targetPosB;
        } else {
            currentTarget = targetPosA;
        }
        setpoint = currentTarget;  // Update setpoint for PID
        delay(1000);  // Brief pause to simulate the stop at each end
    }

    // Debugging information
    Serial.print("Current Position: ");
    Serial.print(encoderPos);
    Serial.print(" | Target Position: ");
    Serial.print(currentTarget);
    Serial.print(" | Output: ");
    Serial.println(output);

    delay(10);  // Small delay to avoid overwhelming the serial monitor
}