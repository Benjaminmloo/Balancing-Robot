// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double accl_scaled_x, accl_scaled_y, accl_scaled_z;
double gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;
int16_t OldAcX, OldAcY,OldAcZ,OldGyX,OldGyY,OldGyZ;//reference values to test against
void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}
//from balancing software
double dist(double a, double b){
 return sqrt((a * a) + (b * b));
}
double get_y_rotation(double x, double y, double z){
   double radians;
   radians = atan2(x, dist(y, z));
   return -(radians * (180.0 / M_PI));
}
double get_x_rotation(double x, double y, double z){
   double radians;
   radians = atan2(y, dist(x, z));
   return (radians * (180.0 / M_PI));
}
void init(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  OldAcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  OldAcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  OldAcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  OldGyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  OldGyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  OldGyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  accl_scaled_x = OldAcX / 16384.0;//scaling factor for G value is 16384
  accl_scaled_y = OldAclY / 16384.0;
  accl_scaled_z = OldAclZ / 16384.0;
  gyro_scaled_x = OldGyX / 131.0;//scaling factor to account for accuracy of gyroscope and conversion to radians
  gyro_scaled_y = OldGyY / 131.0;
  gyro_scaled_z = OldGyZ / 131.0;
  delay (100)
}

void GetAll() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  accl_scaled_x = AcX / 16384.0;//scaling factor for G value is 16384
  accl_scaled_y = AclY / 16384.0;
  accl_scaled_z = AclZ / 16384.0;
  gyro_scaled_x = GyX / 131.0;//scaling factor to account for accuracy of gyroscope and conversion to radians
  gyro_scaled_y = GyY / 131.0;
  gyro_scaled_z = GyZ / 131.0;
  delay (100)
  /*Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(100);*/
}

void test{
  System.out.println("Tilt Forward and Hold until further notice");
  delay(100);
  GetAll();
  if (get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z)<=0){
    System.out.println("Test Failed");
  }
  else{
    System.out.println("Tilt Backward and Hold until further notice");
    delay(100);
    GetAll();
    if (get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z)>=0){
      System.out.println("Test Failed");
    }
    else{
      System.out.println("Tilt Left and Hold until further notice");
      delay(100);
      GetAll();
        if (get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z)<=0){
          System.out.println("Test Failed");
        }
        else{
          System.out.println("Tilt Right and Hold until further notice");
          delay(100);
          GetAll();
          if (get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z)>=0){
              System.out.println("Test Failed");
          }
          else{
            System.out.println("Test Passed");
          }
        }
    }
  }
}
    
