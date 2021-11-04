/* Geared DC motor with Hall Effect Quadrature Encoder 
 * L298N Motor Driver
 * 
 * Wiring:
 * VCC -> 5V
 * GND -> GND
 * OUT A -> D2 
 * OUT B -> D3 
 * M1 -> Motor Driver A
 * M2 -> Motor Driver B
 * Motor Driver Input 1 -> 10
 * Motor Driver Input 2 -> 11
 */ 
 
#include <Robojax_L298N_DC_motor.h>

// Motor 1 pins
#define enablePinA 19   // should be PWM enabled
#define motorIn1 18
#define motorIn2 5
#define channelA 0

// Motor  2 pins
#define enablePinB 4   // should be PWM enabled
#define motorIn3 17
#define motorIn4 16
#define channelB 1

// encoder pins
#define encoder1A 34
#define encoder1B 35
#define encoder2A 32
#define encoder2B 33

// motor driver library variables
#define motor1 1    // do not change
#define motor2 2    // do not change

const int CCW = 2, CW = 1;

#define ENCODER_PPR 7
#define WHEEL_RADIUS 14.3

portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;

// encoder pulse counters
volatile int count1A = 0, count1B = 0;
volatile int count2A = 0, count2B = 0;

// wheel direction
int dir[] = {0, 0}; // 1 - CW, 0 - Stationary, 1 - CCW

// PID variables
double Kp = 0.0, Ki = 0.0, Kd = 0.0;
double error, previousError = 0, output1, output2, setpoint, cumError = 0, rateError = 0; 

int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;
double calculated_w1 = 0, calculated_w2 = 0, recieved_w1 = 0, recieved_w2 = 0;

Robojax_L298N_DC_motor robot(motorIn1, motorIn2, enablePinA, channelA, motorIn3, motorIn4, enablePinB, channelB);

void setup() {
  Serial.begin(115200);
  robot.begin();

  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);

  attachInterrupt(encoder1A, ISR1_A, RISING);
  attachInterrupt(encoder1B, ISR1_B, RISING);
  attachInterrupt(encoder2A, ISR2_A, RISING);
  attachInterrupt(encoder2B, ISR2_B, RISING);
}

void loop() {

// actual hardware trial
//  robot.rotate(motor1, 80, ((dir[0] == 1) ? CW : CCW) );//run motor1 at 60% speed in CW direction
//  robot.rotate(motor2, 70, ((dir[1] == 1) ? CW : CCW) );//run motor1 at 60% speed in CW direction
//  delay(3000);
//
//  robot.brake(1);
//  robot.brake(2);  
//  delay(2000);
//
//
//  robot.rotate(motor1, 100, ((dir[0] == 1) ? CW : CCW) );//run motor1 at 60% speed in CW direction
//  delay(3000);
//  
//  robot.rotate(motor2, 100, ((dir[1] == 1) ? CW : CCW) );//run motor1 at 60% speed in CW direction
  
  while(!Serial.available()) {}
  recieved_w1 = Serial.read();
  while(!Serial.available()) {}
  recieved_w2 = Serial.read();

  currentMillis = millis();

  if(currentMillis - previousMillis > interval) {
    

    calculated_w1 = ((float)(count1A * TWO_PI / ENCODER_PPR)) * dir[0];
    calculated_w2 = (float)(count2A * TWO_PI / ENCODER_PPR) * dir[1];
    
    output1 = computePID(recieved_w1, calculated_w1);
    output2 = computePID(recieved_w2, calculated_w2);
    
    previousMillis = currentMillis;

    robot.rotate(motor1, output1*WHEEL_RADIUS, ((recieved_w1 < 0) ? CW : CCW) );
    robot.rotate(motor2, output2*WHEEL_RADIUS, ((recieved_w2 < 0) ? CW : CCW) );
  
    Serial.println("Angular velocity of wheel 1: " + String(calculated_w1) + "Angular velocity of wheel 2: " + String(calculated_w2));

    count1A = 0; count1B = 0;  count2A = 0; count2B = 0;
  }
  
  delay(100);
}

void ISR1_A() {
  portENTER_CRITICAL(&synch);
  ISR_A(1);
  portEXIT_CRITICAL(&synch);
}

void ISR2_A() {
  portENTER_CRITICAL(&synch);
  ISR_A(2);
  portEXIT_CRITICAL(&synch);
}

void ISR1_B() {
  portENTER_CRITICAL(&synch);
  ISR_B(1);
  portEXIT_CRITICAL(&synch);
}

void ISR2_B() {
  portENTER_CRITICAL(&synch);
  ISR_B(2);
  portEXIT_CRITICAL(&synch);
}

void ISR_A(int motorId) {
  switch(motorId) {
    case 1: dir[0] = (digitalRead(encoder1B) == HIGH) ? 1 : -1;
            count1A += dir[0];
            break;
    case 2: dir[1] = (digitalRead(encoder2B) == HIGH) ? 1 : -1;
            count2A += dir[1];
  };
}

void ISR_B(int motorId) {
  switch(motorId) {
    case 1: count1B += dir[0];
            break;
    case 2: count2B += dir[1];
  };
}

double computePID(double recieved, double calclated) {
  error = setpoint - input;
  cumError += erroe * (currentMillis - previousMillis);
  rateError - (error - previousError) / (currentMillis - previousMillis);

  double out = Kp * error + Ki * cumError + Kd * rateError;

  lastError = error;

  return out;
}
