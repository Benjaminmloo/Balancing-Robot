#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

#define RANGE	100
#define INITIAL_VALUE 0
int a1 = 23;
int a2 = 24;
int b1 = 28;
int b2 = 29;


void init_motors()
{
  wiringPiSetup();

  // initialize left motor (pin0, 2 & 3)
  softPwmCreate(a1, INITIAL_VALUE, RANGE);
  softPwmCreate(a2, INITIAL_VALUE, RANGE);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  // initialize right motor (pin4, 5 & 6)
  softPwmCreate(b1, INITIAL_VALUE, RANGE);
  softPwmCreate(b2, INITIAL_VALUE, RANGE);
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
}

void stop_motors()
{
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(a1, LOW);
  digitalWrite(a2, LOW);

  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(b1, LOW);
  digitalWrite(b2, LOW);

//  printf("motor stopped!!\n");

  // exit(1);

}

double left_speed;
double right_speed;

void motors(double speed, double left_offset, double right_offset)
{

// to come to me, drive pin 0&5 to some power
// to away from me, drive pin 2&4 to some power

// pin 0:LM+, 2:LM-, 3:LCE
// pin 5:RM+, 4:RM-, 6:RCE
// put M+ high & M- low will come to me
// put M+ low & M- High will away from me

  left_speed = speed + left_offset;
  right_speed = speed + right_offset;

  // left motor
  if (left_speed < 0)  {
    softPwmWrite(a1, (int) -left_speed);
    softPwmWrite(a2, 0);
  }
  else
  if (left_speed > 0)  {
    softPwmWrite(a2, (int) left_speed);
    softPwmWrite(a1, 0);
  }

  // right motor
  if (right_speed < 0)  {
    softPwmWrite(b1, (int) -right_speed);
    softPwmWrite(b2, 0);
  }
  else
  if (right_speed > 0)  {
    softPwmWrite(b2, (int) right_speed);
    softPwmWrite(b1, 0);
  }
}



