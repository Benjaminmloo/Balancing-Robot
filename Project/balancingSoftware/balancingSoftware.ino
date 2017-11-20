#include <stdio.h>
#include <stdlib.h>
#include <Wire.h>

//int const sampleSize = 20;
//double[sampleSize] sampleData;

const int MPU_ADDR = 0x68;

const int SIGNAL_MAX = 255; //max PWM output
const int SIGNAL_MIN = 100; //min PWM output
const int INITIAL_VALUE = 0;

// Motor pins
const int a1 = 6; //motor 1 +ve
const int a2 = 5; //motor 1 -ve
const int b1 = 11; //motor 2 +ve
const int b2 = 10; //motor 2 -ve

double left_speed;
double right_speed;

// PID parameters - Working parameters
const double Kp = 2.5;        // 2.5
const double Ki = 0.2;        // 0.2
const double Kd = 8;          // 8.0
const double K  = 1.9 * 1.12; // 1.9*1.12


// Complimentary Filter parameters
const double K0 = (double) 0.40; //gyro weight
const double K1 = (double) 0.60; //accl weight

int16_t acclX, acclY, acclZ; //raw values from sensor
int16_t gyroX, gyroY, gyroZ;
double accl_scaled_x, accl_scaled_y, accl_scaled_z; //scaled to m/s and degrees/minute respictively
double gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;


double gyro_offset_x, gyro_offset_y; //Initial gyro reading to offset any error in the sensor
double gyro_x_delta, gyro_y_delta; //The change rotation position calculated with (change in time) * (angular velocity)
double accl_rot_x, accl_rot_y; //rotation value calculated from accelerometer
double rotation_x, rotation_y; //rotation value used by PID to calculate error / weighted between accl and gyro values

double error, last_error, integrated_error; //last error and integrated error used by D I portions of PID respectively
double pTerm, iTerm, dTerm;
double angle; //The angle value that the PID is given
double angle_offset = 0;  //1.5

double speed; //pwm value given to motors
double pid_out; //out signal of the motors
double dir; //direction the motors should spin

unsigned long timer, t, deltaT;

int sampleNum, inc;

boolean pidReadout = true; //print control parameter (error, speed, pid_ouut, P, I, D)
boolean motionReadout = false;
int operationMode = 1; //0 run with sensors, 1 run with sample input,2 run with sample input using simulator
boolean motorEnable = true;

/*
   params:void
   return:void

   Initialize motor pins
*/
void init_motors()
{

  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);

  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);
  stop_motors();
}

/*
   params:void
   return:void

   stops motor signal
   by setting pins to 0
*/
void stop_motors()
{
  digitalWrite(a1, LOW);
  digitalWrite(a2, LOW);

  digitalWrite(b1, LOW);
  digitalWrite(b2, LOW);

}

/*
   a: first number
   b: second number

   return: applies pythagrious theorm to find the distance between two points

  finds the distance bewtween two points using pythagrious theorm
*/
double dist(double a, double b)
{
  return sqrt((a * a) + (b * b));
}

/*
   x: acceleration in the x direction
   y: acceleration in the y direction
   z: acceleration in the z direction

   return: An aproximate y rotation

   Aproximates y rotation position using accelerometer values
*/
double get_y_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(x, dist(y, z));
  return -(radians * (180.0 / M_PI));
}

/*
   x: acceleration in the x direction
   y: acceleration in the y direction
   z: acceleration in the z direction

   return: An aproximate x rotation

   Aproximates x rotation position using accelerometer values
*/
double get_x_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(y, dist(x, z));
  return (radians * (180.0 / M_PI));
}


/*
  Reads all of the relavent data regesters from the MPU 6050 and scales the values to real world units(m/s degrees/minute)
*/
void read_all()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); //write to the firstrelevant register to start reading from there
  Wire.endTransmission(false); //
  Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 14 registers

  acclX = Wire.read() << 8 | Wire.read();
  acclY = Wire.read() << 8 | Wire.read();
  acclZ = Wire.read() << 8 | Wire.read();

  Wire.read();
  Wire.read(); //read through unnused values

  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  accl_scaled_x = acclX / 16384.0;
  accl_scaled_y = acclY / 16384.0;
  accl_scaled_z = acclZ / 16384.0;

  gyro_scaled_x = gyroX / 131.0;
  gyro_scaled_y = gyroY / 131.0;
  gyro_scaled_z = gyroZ / 131.0;

  if (motionReadout)
  {
    Serial.print(accl_scaled_x); Serial.print(", ");
    Serial.print(accl_scaled_y); Serial.print(", ");
    Serial.print(accl_scaled_x); Serial.print(", ");
    Serial.print(gyro_scaled_x); Serial.print(", ");
    Serial.print(gyro_scaled_y); Serial.print(", ");
    Serial.print(gyro_scaled_z); Serial.print("\n");
  }
}

/*
   speed
   left_offset: offset speed for left wheel
   right_offset: offset speed for right wheel

   return: void

   Sets pwm duty to the nessary pins
   m1 - pwm
   m2 - ground  -> +ve direction

   m2 - pwm
   m1 - ground  -> -ve direction
*/
void motors(double speed, double left_offset, double right_offset)
{

  left_speed = speed + left_offset;
  right_speed = speed + right_offset;

  // left motor
  if (left_speed < 0)  {
    analogWrite(a1, (int) - left_speed);
    analogWrite(a2, 0);
  }
  else if (left_speed > 0)  {
    analogWrite(a2, (int) left_speed);
    analogWrite(a1, 0);
  }

  // right motor
  if (right_speed < 0)  {
    analogWrite(b1, (int) - right_speed);
    analogWrite(b2, 0);
  }
  else if (right_speed > 0)  {
    analogWrite(b2, (int) right_speed);
    analogWrite(b1, 0);
  }
}

/*
   uses external values fromqw:
   angle
   angle_offset
   kp, ki, kd

   PID controller used to create out put to balance the vehicle
*/
void pid()
{
  error = angle - angle_offset;

  pTerm = Kp * error;

  integrated_error = 0.95 * integrated_error + error; //integration done by adding values up over time with multiplier to limit the integrator
  iTerm = Ki * integrated_error;

  dTerm = Kd * (error - last_error); //differentiation  done by just finding the difference between the current and the lst error
  last_error = error;

  pid_out = K * (pTerm + iTerm + dTerm);
  dir = pid_out / abs(pid_out);

  if (abs(pid_out) < 30) //limit the out put too -255 to -100, 0, 100 to 255
  {
    speed = 0;
  } else if (pid_out > 0)
  {
    speed = constrain(pid_out, SIGNAL_MIN, SIGNAL_MAX);
  } else if (pid_out < 0)
  {
    speed = constrain(pid_out, -SIGNAL_MAX, -SIGNAL_MIN);
  }

  if (pidReadout)
  {
    Serial.print(angle); Serial.print(", ");
    Serial.print(error); Serial.print(", ");
    //Serial.print(speed); Serial.print(", ");
    Serial.print(pid_out); Serial.print(", ");
    Serial.print(pTerm); Serial.print(", ");
    Serial.print(iTerm); Serial.print(", ");
    Serial.print(dTerm); Serial.print("\n");
  }
}


double theta_a, theta_v, theta_p;
double M = 10, m = 2, l = 1, g = 9.81;
void sim()
{
  theta_a = (pid_out * cos(theta_p) - (M + m) * g * sin(theta_p) + m * l * sin(theta_p) * cos(theta_p) * sq(theta_v)) / (m * l * sq(cos(theta_p)) - (M + m) * l);
  theta_v += theta_a;
  theta_p += theta_v;
  angle = theta_p;
}


/*
   Initiates:
   inc
   sampleNum
   timer
   deltaT
   read_all()
    int16_t acclX, acclY, acclZ; //raw values from sensor
    int16_t gyroX, gyroY, gyroZ;
    accl_scaled_x, accl_scaled_y, accl_scaled_z; //scaled to m/s and degrees/minute respictively
    Gyro_scaled_x, gyro_scaled_y, gyro_scaled_z
   rotation_x
   rotation_y
   gyro_offset_x
   gyro_offset_y
   setup for the start of the program
*/
void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);  //turn off "sleep" register
  Wire.endTransmission(true);


  //initiate all necesary variables
  init_motors();

  delay(200);

  inc = 0;
  sampleNum = 0;

  timer = millis();

  deltaT = (double) (millis() - timer) / 1000000.0;
  read_all();

  rotation_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
  rotation_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);

  gyro_offset_x = gyro_scaled_x;
  gyro_offset_y = gyro_scaled_y;

  theta_p = 0;
  theta_a = 0;
  theta_v = 0;
}



void loop() {

  if (operationMode == 0) 
  {
    t = millis();
    deltaT = (double) (t - timer) / 1000000.0;
    timer = t;

    read_all();

    gyro_scaled_x -= gyro_offset_x;
    gyro_scaled_y -= gyro_offset_y;

    gyro_x_delta = (gyro_scaled_x * deltaT);
    gyro_y_delta = (gyro_scaled_y * deltaT);

    accl_rot_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
    accl_rot_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);

    rotation_x = K0 * (rotation_x + gyro_x_delta) + (K1 * accl_rot_x);
    rotation_y = K0 * (rotation_y + gyro_y_delta) + (K1 * accl_rot_y);

    angle = rotation_x;

    if (rotation_y < -60.0 || rotation_y > 60.0)
    {
      stop_motors();
      motorEnable = false;
    }

  } else if (operationMode >= 1) //Test PID using sample data
  {
    sampleNum ++;
    if (sampleNum  < 100 ) //zero-input respose
    {
      angle_offset = 0;
    } else if (sampleNum  < 200 ) // positive step
    {
      angle_offset = 10;
    } else if (sampleNum  < 300 ) // negative step
    {
      angle_offset = -10;
    } else if (sampleNum  == 301 ) //positive ramp
    {
      angle_offset = 0;
    } else if (sampleNum < 350 )
    {
      angle_offset ++;
    } else if (sampleNum  == 351 ) //negative ramp
    {
      angle_offset = 0;
    } else if (sampleNum < 400)
    {
      angle_offset --;
    } else //reset
    {
      angle_offset = 0;
      sampleNum = 0;
    }
    if (operationMode == 2)
      sim();

  }

  pid();

  if (motorEnable)
    motors(speed, 0.0, 0.0);

  delay(50);
}

