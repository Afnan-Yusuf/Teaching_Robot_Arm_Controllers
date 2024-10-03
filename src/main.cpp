// Arm controller using serial data from esp32
#include <Arduino.h>
#define lmf 5
#define lmb 6
#define rmf 10
#define rmb 9

#define left_motor_limit1 3
#define left_motor_limit2 4
#define right_motor_limit1 7
#define right_motor_limit2 8

bool homed = false;
String dat;
int homespeed = 200;

void sweep();
void stopmot();
void home();
void setup() {
  Serial.begin(9600);
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
  pinMode(left_motor_limit1, INPUT);
  pinMode(left_motor_limit2, INPUT);
  pinMode(right_motor_limit1, INPUT);
  pinMode(right_motor_limit2, INPUT);
}

void loop() {
  if(Serial.available() > 0){
     dat = Serial.readStringUntil(',');
    
  }
  //convert the dat string to int
  int val = dat.toInt();

  if(val == 1){
    sweep();
  }

  if(val == 2){
    home();
  }
  if(val == 3){
    stopmot();
  }
}

void sweep(){
  digitalWrite(lmf, HIGH);
  digitalWrite(lmb, LOW);
  digitalWrite(rmf, HIGH);
  digitalWrite(rmb, LOW);
}

void home(){
  while (homed == true){
    bool lhomed  = false;
    
    float left_motor_limit1_state = digitalRead(left_motor_limit1);
    float left_motor_limit2_state = digitalRead(left_motor_limit2);
    float right_motor_limit1_state = digitalRead(right_motor_limit1);
    float right_motor_limit2_state = digitalRead(right_motor_limit2);
    if(left_motor_limit1_state == 0){
      analogWrite(lmf, homespeed);
    }else{
      analogWrite(lmf, 0);
      if (lhomed == true){
        homed = true;
      }
    }

    if(right_motor_limit1_state == 0){
      analogWrite(rmf, homespeed);
    }else{
      analogWrite(rmf, 0);
      lhomed = true;
    }
  }

}

void stopmot(){
  analogWrite(lmf, 0);
  analogWrite(lmb, 0);
  analogWrite(rmf, 0);
  analogWrite(rmb, 0);
}