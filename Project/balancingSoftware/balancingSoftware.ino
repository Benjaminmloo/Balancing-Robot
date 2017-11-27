#include <Wire.h>

#define SERIAL_SPEED 9600
#define MPU_ADDR 0x68
#define DATA_START 0x3B
#define DATA_SIZE 14

#define IN_SENSOR 0
#define IN_SIM 1

#define REFERENCE_NONE 20
#define REFERENCE_INTERNAL 21
#define REFERENCE_SERIAL 22

#define DELAY_LOOP 50
#define DELAY_INIT 200


//Program State Parameters
const boolean PRINT_PID = true; //print control parameter (error, speed, pidOut, P, I, D)
const boolean PRINT_MOTION = false; //print data from sensor (gX, gY, gZ, aX, aY, aZ)
const int INPUT_MODE = IN_SIM;
const int REFERENCE_MODE = REFERENCE_INTERNAL;

boolean motorEnable = true;


const int SIGNAL_MAX = 255; //max PWM output
const int SIGNAL_MIN = 100; //min PWM output
const int INITIAL_VALUE = 0;

// Motor Parameters
const int MA1 = 6; //motor 1 +ve
const int MA2 = 5; //motor 1 -ve
const int MB1 = 11; //motor 2 +ve
const int MB2 = 10; //motor 2 -ve

double left_speed;
double right_speed;

// PID parameters         //Know working parameters for physical vehicle 
const double KP = 2;      // 2.5
const double KI = 1;      // 0.2
const double KD = 1.5;    // 8.0
const double K  = 1;      // 1.9*1.12

const double I_MAX = 10;
double angleReference = 0;  //1.5

double error, lastError, integratedError; //last error and integrated error used by D I portions of PID respectively
double pTerm, iTerm, dTerm;
double dir; //direction the motors should spin
double angle; //The angle value that the PID is given

double speed; //pwm value given to motors
double pidOut; //out signal of the motors

//Simulation parameters
double simA = 0, simV = 0, simP = 0; //simulation acceleration velocity and position
double m = 10, b = 5, k = 6; //spring-damper-mass system variables
double M = 100, g = 9.81, l = 10; //pendulum system variables


// Complimentary Filter parameters
const double K0 = (double) 0.40; //gyro weight
const double K1 = (double) 0.60; //accl weight

//Motion Values
int16_t acclRawX, acclRawY, acclRawZ; //raw values from sensor
int16_t gyroRawX, gyroRawY, gyroRawZ;
double acclScaledX, acclScaledY, acclScaledZ; //scaled to m/s and degrees/minute respictively
double gyroScaledX, gyroScaledY, gyroScaledZ;

double gyroOffsetX, gyroOffsetY; //Initial gyro reading to offset any error in the sensor
double gyroDeltaX, gyroDeltaY; //The change rotation position calculated with (change in time) * (angular velocity)
double acclRotationX, acclRotationY; //rotation value calculated from accelerometer
double rotationX, rotationY; //rotation value used by PID to calculate error / weighted between accl and gyro values

//Time Parameters
unsigned long timer, t, deltaT;

//Sample Data Parameters
int sampleNum;



/*
   params:void
   return:void

   Initialize motor pins
*/
void initMotors()
{

  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);

  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);
  stopMotors();
}

/*
   params:void
   return:void

   stops motor signal
   by setting pins to 0
*/
void stopMotors()
{
  digitalWrite(MA1, LOW);
  digitalWrite(MA2, LOW);

  digitalWrite(MB1, LOW);
  digitalWrite(MB2, LOW);

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
double calcRotationY(double x, double y, double z)
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
double calcRotationX(double x, double y, double z)
{
  double radians;
  radians = atan2(y, dist(x, z));
  return (radians * (180.0 / M_PI));
}


/*
  Reads all of the relavent data regesters from the MPU 6050 and scales the values to real world units(m/s degrees/minute)
  outputs to global variables (gyro/accl, raw/scalled, x/y/x)
*/
void readAll()
{
  do
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(DATA_START); //write to the firstrelevant register to start reading from there
    Wire.endTransmission(false); //
  } while (Wire.requestFrom(MPU_ADDR, DATA_SIZE, true) == DATA_SIZE); //request a total of 14 registers retry if not all are returned

  acclRawX = Wire.read() << 8 | Wire.read();
  acclRawY = Wire.read() << 8 | Wire.read();
  acclRawZ = Wire.read() << 8 | Wire.read();

  Wire.read();
  Wire.read(); //read through unnused values

  gyroRawX = Wire.read() << 8 | Wire.read();
  gyroRawY = Wire.read() << 8 | Wire.read();
  gyroRawZ = Wire.read() << 8 | Wire.read();

  acclScaledX = acclRawX / 16384.0;
  acclScaledY = acclRawY / 16384.0;
  acclScaledZ = acclRawZ / 16384.0;

  gyroScaledX = gyroRawX / 131.0;
  gyroScaledY = gyroRawY / 131.0;
  gyroScaledZ = gyroRawZ / 131.0;

  if (PRINT_MOTION)
  {
    Serial.print(acclScaledX); Serial.print(", ");
    Serial.print(acclScaledY); Serial.print(", ");
    Serial.print(acclScaledX); Serial.print(", ");
    Serial.print(gyroScaledX); Serial.print(", ");
    Serial.print(gyroScaledY); Serial.print(", ");
    Serial.print(gyroScaledZ); Serial.print("\n");
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
    analogWrite(MA1, (int) - left_speed);
    analogWrite(MA2, 0);
  }
  else if (left_speed > 0)  {
    analogWrite(MA2, (int) left_speed);
    analogWrite(MA1, 0);
  }

  // right motor
  if (right_speed < 0)  {
    analogWrite(MB1, (int) - right_speed);
    analogWrite(MB2, 0);
  }
  else if (right_speed > 0)  {
    analogWrite(MB2, (int) right_speed);
    analogWrite(MB1, 0);
  }
}

/*
   uses external values from:
   angle
   angleReference
   kp, ki, kd

   PID controller used to create out put to balance the vehicle
*/
void pid()
{
  error = angle - angleReference;

  pTerm = KP * error;

  integratedError = integratedError + error; //integration done by adding values up over time
  if (abs(integratedError) > I_MAX)
    integratedError = I_MAX * (integratedError / abs(integratedError));
  iTerm = KI * integratedError;

  dTerm = KD * (error - lastError); //differentiation  done by just finding the difference between the current and the lst error
  lastError = error;

  pidOut = K * (pTerm + iTerm + dTerm);
  dir = pidOut / abs(pidOut);

  if (abs(pidOut) < 30) //limit the out put too -255 to -100, 0, 100 to 255
  {
    speed = 0;
  } else if (pidOut > 0)
  {
    speed = constrain(pidOut, SIGNAL_MIN, SIGNAL_MAX);
  } else if (pidOut < 0)
  {
    speed = constrain(pidOut, -SIGNAL_MAX, -SIGNAL_MIN);
  }

  if (PRINT_PID)
  {
    Serial.print(angle); Serial.print(", ");
    Serial.print(angleReference); Serial.print("\n");
    //    Serial.print(pidOut); Serial.print(", ");
    //    Serial.print(pTerm); Serial.print(", ");
    //    Serial.print(iTerm); Serial.print(", ");
    //    Serial.print(dTerm); Serial.print("\n");
  }
}

/*
 * simulates the motion of some real system
 * acceleration is calculated based on the real system
 * velocity and position are integrals and second integrals of that in the form of a summer
 */
void sim()
{
  //TODO 
  //create switch for different simulations. get inverted pendulum simmulation working
  //simA = (10 * pidOut * cos(simP) - (M + m) * g * sin(simP) + m * l * sin(simP) * cos(simP) * sq(simV)) / (m * l * sq(cos(simP)) - (M + m) * l);
  simA = (pidOut / m) - (b / m) * simV  - (k / m) * simP;
  simV += simA;
  simP += simV;
  angle = -simP;
  if (PRINT_PID)
  {
    //    Serial.print(simA); Serial.print(", ");
    //    Serial.print(simV); Serial.print(", ");
    //    Serial.print(simP); Serial.print("\n");
  }
}


/*
 *  setup for the start of the program
 *  initiates connection to external devices
 *  intialises variable that are based in the systems physical state (motion / time)
 */
void setup()
{
  Serial.begin(SERIAL_SPEED);

  Wire.begin();
  do {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);   //turn off "sleep" register
  } while (Wire.endTransmission(true)); //loop if there is an error


  initMotors();
  delay(DELAY_LOOP); //delay to let system settle
  
  sampleNum = 0;
  
  timer = millis();
  deltaT = (double) (millis() - timer) / 1000000.0;
  
  readAll();

  rotationX = calcRotationX(acclScaledX, acclScaledY, acclScaledZ); //set initial position based purely on accl values
  rotationY = calcRotationY(acclScaledX, acclScaledY, acclScaledZ);

  gyroOffsetX = gyroScaledX; //sets error in gyro values as the initial angular velosity
  gyroOffsetY = gyroScaledY; //HOLD VEHICLE STILL WHEN STARTING if this sin't done offset will be wrong
}


/*
 * Main control loop. 
 * Gets input signal, runs PID control, sends output to system
 */
void loop()
{

  //get input based from specified source
  if (INPUT_MODE == IN_SENSOR) 
  {
    //calculate time from last loop
    t = millis();
    deltaT = (double) (t - timer) / 1000000.0;
    timer = t;

    readAll();

    gyroScaledX -= gyroOffsetX; //adjust gyro values based on initial error
    gyroScaledY -= gyroOffsetY;

    gyroDeltaX = (gyroScaledX * deltaT); //find theoretical change in rotational position
    gyroDeltaY = (gyroScaledY * deltaT);

    acclRotationX = calcRotationX(acclScaledX, acclScaledY, acclScaledZ); //calculate rotational position based on accelerometer
    acclRotationY = calcRotationY(acclScaledX, acclScaledY, acclScaledZ);

    rotationX = K0 * (rotationX + gyroDeltaX) + (K1 * acclRotationX);  //weight the values calculated from gyro and accelerometer
    rotationY = K0 * (rotationY + gyroDeltaY) + (K1 * acclRotationY);  //gyro rotation position is calculated by summing all deltas

    angle = rotationX;

    if (rotationY < -60.0 || rotationY > 60.0) //If vehicle is flipped over Y axis turn off the motors
    {
      stopMotors();
      motorEnable = false;
    }

  } else if (INPUT_MODE == IN_SIM) //Test PID using simulator
  {
    sim();
  }

  //decide where reference angle is retreived from
  if (REFERENCE_MODE == REFERENCE_INTERNAL) //loop of sample data, can be changed to reflect any kind of input
                                            //currently the sample is a square wave
  {
    sampleNum ++;
    if (sampleNum  < 100 ) //zero-input 
    {
      angleReference = 0;
    } else if (sampleNum  < 200 ) // positive step
    {
      angleReference = 2;
    }  else if (sampleNum < 300) //reset
    {
      angleReference = 0;
      sampleNum = 0;
    }
  } else if (REFERENCE_MODE == REFERENCE_SERIAL)
  {
    angleReference = Serial.read();
    if(abs(angleReference) > 180)
      angleReference = 0;
  }

  pid();
  
  if (motorEnable)
    motors(speed, 0.0, 0.0);

  delay(DELAY_LOOP);
}

