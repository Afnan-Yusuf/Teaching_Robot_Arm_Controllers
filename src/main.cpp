#include <PID_v1.h>
#include <Arduino.h>

#define lmf 5
#define lmb 6
#define rmf 11
#define rmb 10

int maxspeed = 100;

bool dir = false;  // Direction flag: false = forward, true = backward
#define left_motor_encoder 2
#define right_motor_encoder 3

int error = 0;
// Encoder variables
volatile long encoderPos = 0;  // Current encoder position
int targetPosA = 30;  // Encoder position for "to" position
int targetPosB = -500; // Encoder position for "fro" position
int currentTarget = targetPosA;  // Current target position

// PID control variables
double setpoint, input, output;
double Kp = 3, Ki = 0.05, Kd = 0;  // PID constants
PID motorPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Function to handle encoder A interrupts
void handleEncoderA() {
    // Increment or decrement encoderPos based on motor direction
    if (!dir) {
        encoderPos++;
    } else {
        encoderPos--;
    }
}

// Function to control motor direction and speed
void controlMotor(int speed) {
    // Constrain the speed to the maximum allowable speed
    speed = constrain(speed, -maxspeed, maxspeed);

    if (speed > 0) {
        analogWrite(lmf, speed);
        analogWrite(lmb, 0);
    } else if (speed < 0) {
        analogWrite(lmf, 0);
        analogWrite(lmb, -speed);
    } else {
        analogWrite(lmf, 0);
        analogWrite(lmb, 0);  // Stop the motor
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
    
    error = encoderPos - targetPosA;
    // Check if the position is close to the target and switch direction
    if(error < 0){
        controlMotor(0);
    }else{
        controlMotor(output);
    }

    // Debugging information
    Serial.print(encoderPos);
    Serial.print("\t");
    Serial.print(output);
    Serial.print("\t");
    Serial.print(targetPosA);
    Serial.print("\t");
    Serial.println(error);



    delay(10);  // Small delay to avoid overwhelming the serial monitor
}
