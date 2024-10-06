#include <PID_v1.h>
#include <Ramp.h>  // Include the Ramp library

#define lmf 5
#define lmb 6

int maxspeed = 255;
bool dir = false;  // Direction flag: false = forward, true = backward

#define left_motor_encoder 2

// Encoder variables
volatile long encoderPos = 0;
int targetPosA = 10;

// PID control variables
double setpoint, input, output;
double Kp = 1, Ki = 0.04, Kd = 0.001;  // PID constants
PID motorPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Ramp object
ramp motorRamp;  // Create a Ramp object for motor control
int rampDuration = 1000;  // Duration for ramping up (in ms)
int rampTargetSpeed = 0;  // Target motor speed that will be ramped

// Function to handle encoder interrupts
void handleEncoderA() {
    if (!dir) {
        encoderPos++;
    } else {
        encoderPos--;
    }
}

// Function to control motor direction and speed
void controlMotor(int speed) {
    speed = constrain(speed, -maxspeed, maxspeed);
    if (speed > 0) {
        analogWrite(lmf, speed);
        analogWrite(lmb, 0);
        dir = false;
    } else if (speed < 0) {
        analogWrite(lmf, 0);
        analogWrite(lmb, -speed);
        dir = true;
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
    setpoint = targetPosA;
    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(-255, 255);  // Limit output to motor's PWM range

    // Ramp setup
    motorRamp.go(0);  // Set initial speed to 0
}

void loop() {
    input = encoderPos;

    // Compute PID output
    motorPID.Compute();

    // Set ramp target based on PID output
    rampTargetSpeed = output;
    motorRamp.go(rampTargetSpeed);  // Set the PID output as the ramp target

    // Update the ramp object and get the current ramped value
    int currentRampSpeed = motorRamp.update();  // Update the ramp and get the current ramp speed

    // Control motor with ramped speed value
    controlMotor(currentRampSpeed);

    // Debugging information
    Serial.print("Encoder: ");
    Serial.print(encoderPos);
    Serial.print("\tPID Output: ");
    Serial.print(output);
    Serial.print("\tRamp Speed: ");
    Serial.println(currentRampSpeed);

}
