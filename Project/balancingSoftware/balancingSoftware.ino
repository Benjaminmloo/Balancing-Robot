#include <stdio.h>
#include <stdlib.h>
#include <Wire.h>

const int MPU_ADDR = 0x68;

const int RANGE = 100;
const int INITIAL_VALUE = 0;

// Motor pins
int a1 = 1;
int a2 = 2;
int b1 = 3;
int b2 = 4;

double left_speed;
double right_speed;

// PID parameters
double Kp = 2.5;   // 2.5
double Ki = 0.2;   // 1.0
double Kd = 8.0;   // 8.0
double K  = 1.9 * 1.12;


// Complimentary Filter parameters
double K0 = (double) 0.40;
double K1 = (double) 0.60;

int fd;
int16_t acclX, acclY, acclZ;
int16_t gyroX, gyroY, gyroZ;
double accl_scaled_x, accl_scaled_y, accl_scaled_z;
double gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;


double gyro_offset_x, gyro_offset_y;
double gyro_total_x, gyro_total_y;
double gyro_x_delta, gyro_y_delta;
double rotation_x, rotation_y;
double last_x, last_y;

double GUARD_GAIN = 255;
double error, last_error, integrated_error;
double pTerm, iTerm, dTerm;
double angle;
double angle_offset = 0;  //1.5

double speed;

unsigned long timer, t, deltaT;


void init_motors()
{

  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);

  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);
  stop_motors();
}

void stop_motors()
{
  digitalWrite(a1, LOW);
  digitalWrite(a2, LOW);

  digitalWrite(b1, LOW);
  digitalWrite(b2, LOW);

}

double dist(double a, double b)
{
  return sqrt((a * a) + (b * b));
}

double get_y_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(x, dist(y, z));
  return -(radians * (180.0 / M_PI));
}

double get_x_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(y, dist(x, z));
  return (radians * (180.0 / M_PI));
}

void read_all()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

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
}


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

void pid()
{
  error = last_x - angle_offset;

  pTerm = Kp * error;

  integrated_error = 0.95 * integrated_error + error;
  iTerm = Ki * integrated_error;

  dTerm = Kd * (error - last_error);
  last_error = error;

  speed = constrain(K * (pTerm + iTerm + dTerm), -GUARD_GAIN, GUARD_GAIN);

}

void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(MPU_ADDR);

  init_motors();

  delay(200);

  timer = millis();

  deltaT = (double) (millis() - timer) / 1000000.0;
  read_all();

  last_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
  last_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);


  gyro_offset_x = gyro_scaled_x;
  gyro_offset_y = gyro_scaled_y;

  gyro_total_x = last_x - gyro_offset_x;
  gyro_total_y = last_y - gyro_offset_y;
}

void loop() {
  t = millis();
  deltaT = (double) (t - timer) / 1000000.0;
  timer = t;

  read_all();

  gyro_scaled_x -= gyro_offset_x;
  gyro_scaled_y -= gyro_offset_y;

  gyro_x_delta = (gyro_scaled_x * deltaT);
  gyro_y_delta = (gyro_scaled_y * deltaT);

  gyro_total_x += gyro_x_delta;
  gyro_total_y += gyro_y_delta;

  rotation_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
  rotation_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);

  //    printf("[BEFORE] gyro_scaled_y=%f, deltaT=%lf, rotation_y=%f, last_y= %f\n", (double)gyro_scaled_y, (double)deltaT, (double)rotation_y, (double) last_y);

  //    printf("[1st part] = %f\n", (double) K0*(last_y + gyro_y_delta));
  //    printf("[2nd part] = %f\n", (double) K1*rotation_y);
  last_x = K0 * (last_x + gyro_x_delta) + (K1 * rotation_x);
  last_y = K0 * (last_y + gyro_y_delta) + (K1 * rotation_y);

  //    printf("[AFTER] gyro_scaled_y=%f, deltaT=%lf, rotation_y=%f, last_y=%f\n", (double)gyro_scaled_y, (double)deltaT, (double)rotation_y, (double) last_y);

  if (last_y < -60.0 || last_y > 60.0)
    stop_motors();

  pid();

  fprintf(stdout, "%llu, %lf,%lf, %lf, %lf, %lf\n", timer,  error, speed, pTerm, iTerm, dTerm);

  motors(speed, 0.0, 0.0);

  delay(10);
}
