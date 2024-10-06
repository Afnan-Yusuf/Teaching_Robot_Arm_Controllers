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

/*
#include <PID_v1.h>
#include <Ramp.h>

// Motor A pins
#define MOTOR_A_FWD 5
#define MOTOR_A_BWD 6
#define ENCODER_A 2

// Motor B pins
#define MOTOR_B_FWD 9
#define MOTOR_B_BWD 10
#define ENCODER_B 3

#define MAX_SPEED 255

// Motor structures
struct Motor {
    volatile long encoderPos;
    int targetPos;
    bool direction;
    PID* pid;
    ramp* ramp;
    double input, output, setpoint;
    int fwdPin, bwdPin;
};

Motor motorA, motorB;

// PID constants
const double Kp = 1, Ki = 0.04, Kd = 0.001;

// Function prototypes
void setupMotor(Motor& motor, int fwdPin, int bwdPin, int encoderPin);
void handleEncoder(Motor& motor);
void controlMotor(Motor& motor);

void setup() {
    Serial.begin(115200);  // Increased baud rate for faster communication

    setupMotor(motorA, MOTOR_A_FWD, MOTOR_A_BWD, ENCODER_A);
    setupMotor(motorB, MOTOR_B_FWD, MOTOR_B_BWD, ENCODER_B);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), []{ handleEncoder(motorA); }, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), []{ handleEncoder(motorB); }, RISING);
}

void loop() {
    // Update both motors
    updateMotor(motorA);
    updateMotor(motorB);

    // Print debug info every 100ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        printDebugInfo();
        lastPrint = millis();
    }
}

void setupMotor(Motor& motor, int fwdPin, int bwdPin, int encoderPin) {
    motor.fwdPin = fwdPin;
    motor.bwdPin = bwdPin;
    motor.encoderPos = 0;
    motor.targetPos = 0;
    motor.direction = false;

    pinMode(fwdPin, OUTPUT);
    pinMode(bwdPin, OUTPUT);
    pinMode(encoderPin, INPUT_PULLUP);

    motor.pid = new PID(&motor.input, &motor.output, &motor.setpoint, Kp, Ki, Kd, DIRECT);
    motor.pid->SetMode(AUTOMATIC);
    motor.pid->SetOutputLimits(-MAX_SPEED, MAX_SPEED);

    motor.ramp = new ramp();
    motor.ramp->go(0);
}

void handleEncoder(Motor& motor) {
    if (!motor.direction) {
        motor.encoderPos++;
    } else {
        motor.encoderPos--;
    }
}

void updateMotor(Motor& motor) {
    motor.input = motor.encoderPos;
    motor.setpoint = motor.targetPos;

    motor.pid->Compute();

    motor.ramp->go(motor.output);
    int currentSpeed = motor.ramp->update();

    controlMotor(motor, currentSpeed);
}

void controlMotor(Motor& motor, int speed) {
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    if (speed > 0) {
        analogWrite(motor.fwdPin, speed);
        analogWrite(motor.bwdPin, 0);
        motor.direction = false;
    } else if (speed < 0) {
        analogWrite(motor.fwdPin, 0);
        analogWrite(motor.bwdPin, -speed);
        motor.direction = true;
    } else {
        analogWrite(motor.fwdPin, 0);
        analogWrite(motor.bwdPin, 0);
    }
}

void printDebugInfo() {
    Serial.print("A: ");
    Serial.print(motorA.encoderPos);
    Serial.print(",");
    Serial.print(motorA.output);
    Serial.print(" B: ");
    Serial.print(motorB.encoderPos);
    Serial.print(",");
    Serial.println(motorB.output);
}  */