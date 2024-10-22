/******************************************************************************
CPE PC 3219 Final Project
Cebu Technological University

Programmed by: Laurence Miguel A. Ibarra

Resources:
TB6612 SparkFun Library
https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library
NewPing Library
https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home

Arduino IDE v2.3.2
Linefollower:
This uses the PROPORTIONAL ALGORITHM from PID Controller System for line sensing.
It allows for smoother and more responsive inputs thus making it faster.
NOTE: Since we have digital sensors, there is no need to calculate the rates of change (derivative) and steady state error (integral)

Sumobot:
Every operation has been done with time referencing and state machine for non-blocking functionality
Arduino Pro Mini
TB6612FNG 
N20 motor (500 RPM)
3-Channel Line Sensor
******************************************************************************/

// TB6612FNG Library for Algorithm implementation focus 
#include <SparkFun_TB6612.h>
#include <NewPing.h>
 
// CONNECTIONS
#define AIN1 14
#define BIN1 16
#define AIN2 15
#define BIN2 17
#define PWMA 10
#define PWMB 11
#define STBY 12
#define TRIG 3
#define ECHO 2

// motor declarations
Motor motorL = Motor(AIN1, AIN2, PWMA, -1, STBY);
Motor motorR = Motor(BIN1, BIN2, PWMB, 1, STBY);

// sensor declarations
NewPing sonar(TRIG, ECHO, 400);

bool botstate = 0;

void setup()
{
  // 3 Channel Line Sensor [PD4: L, PD5: C, PD6: R]
  // HC SR04 Ultrasonic Sensor [PD2: ECHO, PD3: TRIG] 
  DDRD = 0b10001011;
  // combat mode at PB1, switch mode at PB0
  DDRB = 0b11111100;
  PORTD = 0xff;
  PORTB = 0xff;
  DDRB |= (1 << DDB5); // BUILT IN LED
  Serial.begin(9600);
   // line follower - 1, sumobot - 0;
  if((PINB & 0x01) == 1){
    PORTB |= (1 << PORTB5);
    DDRD &= 0x40;
    botstate = 1;
  }
  else if((PINB & 0x01) == 0){
    PORTB &= ~(1 << PORTB5);
    botstate = 0;
  }
}
// for line sensing
byte sensor;
int baseSpeed = 65;
float Kp = 90; // max value of 75 to not exceed the speed value of 255

void linefollower() {
  while(botstate){
    sensor = (PIND & 0x70) >> 4;
  // calculate the error considering the x-axis 
  int error = 0;
  switch(sensor) {
    case 0b010: 
      error = 0; 
      break;
    case 0b011: 
    case 0b001: 
      error = -1; 
      break;
    case 0b110: 
    case 0b100: 
      error = 1;
      break;
    case 0b111:
      brake(motorL, motorR);
      break;
    default:
      motorL.drive(baseSpeed);
      motorR.drive(baseSpeed);
      break;
  }
  // considering the values of the errors to calculate the speed of each motor rather than an open loop configuration.
  // Proportional Algorithm calculation.
  int adjustment = Kp * error;
  int leftSpeed = baseSpeed - adjustment;
  int rightSpeed = baseSpeed + adjustment;

  motorL.drive(leftSpeed);
  motorR.drive(rightSpeed);

  }
}

// for sumobot
int attackSpeed = 255;
int searchSpeed = 75;
int reverseSpeed = 100;
unsigned int searchstart = 0;
unsigned int searchend = 0;
unsigned int forwardstart = 0;
unsigned int forwardend = 0;
byte rotate = 0;
unsigned long duration = 0;
unsigned long distance = 0;
unsigned long blinkstart = 0;
unsigned long prevblinkstart = 0;
unsigned long attackstart = 0;
unsigned long attackend = 0;
unsigned long borderStart = 0;
enum States{
  ATTACK,
  SEARCH,
  BORDER
};

States state = SEARCH;

void sumobot(){
  while(!botstate){
    sensor = (PIND & 0x70) >> 4;
    distance = constrain(sonar.ping_cm(), 1, 80);
  
    switch(state){
      
      case ATTACK:
        // transition to SEARCH state if opponent is out of range
        if (distance >= 15 || distance == 0) { 
          state = SEARCH;
        } else {
          PORTB |= (1 << PORTB5);
          motorL.drive(attackSpeed);
          motorR.drive(attackSpeed);
          if(sensor != 7){
            state = BORDER;
          }
        }
      break;
        
      case SEARCH:
        if (distance < 15 && distance > 1) { // Transition to ATTACK state if opponent is detected
            state = ATTACK;
        } else if (sensor != 7) { // Transition to BORDER_ROTATE state if border is detected
          state = BORDER;
        } else {
          blinkstart = millis();
          if (blinkstart - prevblinkstart >= 1000) {
            PORTB ^= (1 << PORTB5);
            prevblinkstart = blinkstart;
          }
          searchstart = millis();
          if (searchstart - searchend >= 1500) {
            if (rotate) {
              motorL.drive(searchSpeed);
              motorR.drive(-searchSpeed);
            } else {
              motorL.drive(-searchSpeed);
              motorR.drive(searchSpeed);
            }
            rotate = !rotate; // toggle rotate state
            searchend = searchstart;
          }
          forwardstart = millis();
          if(forwardstart - forwardend >= 300){
            motorL.drive(searchSpeed);
            motorR.drive(searchSpeed);
            forwardend = forwardstart;
          }
        }
      break;

      case BORDER:
        if (borderStart == 0) {
            borderStart = millis(); // Set the start time when entering BORDER state
        }
        if (millis() - borderStart < 750) {
            forward(motorL, motorR, -reverseSpeed);  
        } else {
            borderStart = 0;  
            state = SEARCH;  
          }
      break;
    }
  }
}


// main function

void loop()
{
  if(botstate == 1) {
    if((PINB & 0x02) == 0){
      motorL.drive(50, 100);
      motorR.drive(50, 100);
      linefollower();
    }
  }
  else if(botstate == 0) {
    if((PINB & 0x02) == 0){
      delay(5000);
      sumobot();
    }
  }
}



