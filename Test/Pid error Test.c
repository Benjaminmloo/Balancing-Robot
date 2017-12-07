#include <stdio.h>
#include <stdlib.h>

//working PWM range: 100-255
const int SIGNAL_MAX = 255;//max value for pwm
const int SIGNAL_MIN = 100;//minimum value for the pwm
const int MEMORY_SIZE = 5;//use of the constant threw an error in line 39 but use of the value 5 itself is fine

// PID parameters
//testing required to fine tune PID constants
const double KP = 2.5;   // tuned Proportion value for use in PID controller
const double KI = 0.2;   // tuned Integral Value for use in PID controller
const double KD = 8.0;   // tuned Derivative Value for use in PID controller
const double INTEGRATIONRATIO=0.96;//how much of the integrated error to be used
const double K  = 1.9 * 1.12;//tuned K multiplier value for use in PID controller; output value =K*(Proportion value+ Integral Value+ Derivative Value)


// Complimentary Filter parameters
//double k0 = (double) 0.40;//percentage of gyroscopic value influence on final motor output 
//double k1 = (double) 0.60;//percentage of accelerometer/ rotation value's influence on final motor output

//int fd;
//originally uses int16_t for the arduino but not an available type for unit testing
int acclX, acclY, acclZ;//accelerometer inputs from the "6DOF" in the xyz axis for control calculations
int gyroX, gyroY, gyroZ;//gyroscope inputs in the xyz axis to be used for control calculations
double accl_scaled_x, accl_scaled_y, accl_scaled_z;
double gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;

double error, last_error, integrated_error;
double pTerm, iTerm, dTerm;

double dMem[5];//originally used constant MEMORY_SIZE but threw an error saying"error #2111: Integer expression must be constant."
//double dAvg;
//double angle;

//double speed;
double pid_out;
double correct;//correct output value used in testing of the algorithm

void pid(void){
	//SETTING VALUES TO VALIDATE WITH
  	error = 10;//instead of using last_x- angle offset, we use a manual value
  	integrated_error=30;//completely arbitrary value for testing PID output
  	last_error=8;//arbitrary value for testing

	correct=K*(KP*error+(INTEGRATIONRATIO*integrated_error+error)*KI+(error-last_error)*KD);//correct math for PID control to verify against

  	pTerm = KP * error;//calculation of the P of PID
  	integrated_error = INTEGRATIONRATIO * integrated_error + error;//replace with constant
  	iTerm = KI * integrated_error;
  	dTerm = KD * (error - last_error);
  	last_error = error;
  	pid_out = K * (pTerm + iTerm + dTerm);

}
void test(void){

  if(pid_out==correct){//Checking if the calculated value matches the correct one expected
    printf("Test Passed\n");
    }
  else{
  	printf("Test Failed\n");
  }
}
int main(void){
	pid();//calculation using the code being tested
	test();//test the code generated result with what is correct
	exit(0);
	return 0;
}
