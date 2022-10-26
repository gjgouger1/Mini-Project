//in this file we handle the arduino communcation with the motors for our robot. You can see the well commented code for our controllers that allows for us to complete this demo.

#include "Arduino.h"
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0X04

//Define constants for numerial and pin input/output operations
#define pi 3.1415
#define D2 4
#define M1DIR 7
#define M2DIR 8
#define M1PWM 9
#define M2PWM 10
#define SF 12
#define period 8

//Variables for MATLAB serial communication
String InputString = ""; // a string to hold incoming data
bool StringComplete = false;


//Variables for forward velocity PI controller 
float setRhoPrime = 0;
float finalRhoPrime = 0;
float kpRhoPrime = 51;
float kiRhoPrime = 0.1;
float rhoIntegrator = 0;
float eRhoPrime;

//Variables for angular velocity PI controller
float setPhiPrime;
float finalPhiPrime;
float kpPhiPrime = 9.7;
float kiPhiPrime = 0.1;
float phiIntegrator = 0;
float ePhiPrime;

//Variables to define theta and thetadot on motor 1
float theta1 = 0;
float thetaold1 = 0;
float thetadot1 = 0;

//Variables to define theta and thetadot on motor 2
float theta2 = 0;
float thetaold2 = 0;
float thetadot2 = 0;

//Variables to define the voltages applied to the two motor system
float va;
float deltaVa;

//Variables to define characteristsics of the motors width and wheel radius
float radius = 0.075;
float distance = 0.355;

//Variables to define the individual voltages applied to motors 1 and 2
float v1;
float v2;

//Variables to define the individual pulse-width modulations applied to motors 1 and 2
float PWM1;
float PWM2;

//Variables to define the true forward and angular velocities calculated from the encoder readings
float rhoPrime;
float phiPrime;

//Variables for use in conditional statements of either gradual velocity incrementation of time readings
int i = 1;
int j = 1;

//Variables for use to store time values in thetadot calculations
unsigned long t = 0;
unsigned long told = 0;

//Variables for use to store time values in P(I)(D) controllers
int Ts = 0;
int Tc = millis();

//Variable for Demo2 that faciliatates data transfer between Pi and Arduino 
/*float data = 0;*/

//Variables for angular velocity PID controller
float setPhi = 3.14;
float phi = 0;
float ePhi = 0;
float phiKp = 0.2;
float phiKi = 0.00005;
float phiKd = 0.01;
float positionIntegrator = 0;
float phiDerivative = 0;
float ePhiPast = 0;

//Variables to define parameters for straight motion
float setForwardDistance = 0;
float rho = 0;
float distanceMultiplier = 0;

//Defines encoders 1 and 2 and their respective pin connections
Encoder ENC1(3, 5);
Encoder ENC2(2, 6); 

void setup() {

  //Sets baud rate and prints serial montitor statement
  Serial.begin(115200);
  InputString.reserve(200);
  Serial.println("Ready!");

  //Initites pin channels as 'HIGH' or as 'inputs/outputs'
  digitalWrite(D2, HIGH);
  pinMode(M1DIR, OUTPUT);
  pinMode(M1DIR, OUTPUT);          
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(SF, INPUT);
  pinMode(D2, OUTPUT);

  //Defines forward/backward directions for the motors 
  digitalWrite(M1DIR, 1); // 1 = forward, 0 = backward
  digitalWrite(M2DIR, 0); // 0 = forward, 1 = backward

  //Demo2 data transfer code
  /*Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);*/
}

void loop() {

  //Demo2 data trasnfer code
  /*setPhi = (float)data; uncomment to receive angle*/

  //MATLAB communication code
  if (StringComplete) {
       StringComplete = false;
  }

  //Reads time to check for duration since previous run
  told = t;
  while(millis()<told + period);  
  t = millis();

  //Calculation of thetadot for motor 1 by calculations for the encoder readings
  thetaold1 = theta1;
  theta1 = (float)ENC1.read() * -2 * pi / 3200;
  
  float deltat1 = t-told;           
  float deltatheta1 = theta1 - thetaold1;

  deltat1 = deltat1/1000;         
  thetadot1 = deltatheta1/deltat1;

  //Calculation of thetadot for motor 2 by calculations for the encoder readings
  thetaold2 = theta2;
  theta2 = (float)ENC2.read() * 2 * pi / 3200; 
  
  float deltat2 = t-told;           
  float deltatheta2 = theta2 - thetaold2;

  deltat2 = deltat2/1000;         
  thetadot2 = deltatheta2/deltat2;

  //Calculation of current Rho (forward motion) using the change in the wheel's circuimference
  rho = rho + radius*((deltatheta1+deltatheta2)/2);

  //Angular position PID controller that uses proportional, integral, and derivative error to reach a set phi (angle in radians) value.
  phi = radius*(theta1-theta2)/distance;
  ePhi = setPhi - phi; //phi error
  positionIntegrator = positionIntegrator + ePhi*Ts;
  
  if (Ts > 0) {
    phiDerivative = (ePhi - ePhiPast)/Ts;
    ePhiPast = ePhi;
  }
  else {
    phiDerivative = 0;
  }

  //Output of the PID controller is fed into the input variable for angular velocity controller
  setPhiPrime = phiKp*ePhi + phiKi*positionIntegrator + phiKd*phiDerivative;
  finalPhiPrime = setPhiPrime;

  //Calculates the actual forward and angular velocitis
  rhoPrime = radius*(thetadot1 + thetadot2)/2;
  phiPrime = radius*(thetadot1 - thetadot2)/distance;

  //Conditional for stationary rotation with straight motion afterward 
  if(ePhi <= 0) {
    setRhoPrime = 0.157;
    finalRhoPrime = 0.157;
  }

  //Condtional to cause incremental (by tenths) forward velocity increases (slow the step function)
  if(i <= 10) {
    setRhoPrime = setRhoPrime*0.1*i;
    i++;
  }

  else {
    setRhoPrime = finalRhoPrime;
  }

  //Condtional to cause incremental (by tenths) angular velocity increases (slow the step function)
  if(j <= 10) {
    setPhiPrime = setPhiPrime*0.1*j;
    j++;
  }

  else {
    setPhiPrime = finalPhiPrime;
  }

  
  //Conditional for travelling along an arc with straight motion afterward or just straight motion (if setPhi is set equal to zero)
  if ((ePhi <= 0) && (setRhoPrime != 0)) {

      //Converts meters to feet
      setForwardDistance = 1*0.305*2;

      //Multipliers used are for tuning motion based on experiments with varying set meter distances
      if (setForwardDistance <= 1) {

        distanceMultiplier = 0.96;
        
      }

      if ((setForwardDistance > 1) && (setForwardDistance <= 2)) {

        distanceMultiplier = 0.98;
      }

      if ((setForwardDistance > 2)&& (setForwardDistance <= 3)) {

        distanceMultiplier = 0.98;
      }
      
  }

  //Halts straight motion if rover attains the desired traveled distance 
  if(rho >= distanceMultiplier*setForwardDistance) {
    Serial.print("made it");
    setRhoPrime = 0;
    finalRhoPrime = 0;
    setForwardDistance = 0;
  }

  //Prevents overshoot on the angle turn
  if(ePhi <= 0) {
    positionIntegrator = 0;
    phiDerivative = 0;
  }

  //Start of PI controllers for both angular and forward velocity 
  eRhoPrime = setRhoPrime - rhoPrime; //Rhoprime error
  ePhiPrime = setPhiPrime - phiPrime; //Phiprime error

  rhoIntegrator = rhoIntegrator + Ts*eRhoPrime;
  phiIntegrator = phiIntegrator + Ts*ePhiPrime;

  //Defines the input voltages for the two motor system
  va = kpRhoPrime*eRhoPrime + kiRhoPrime*rhoIntegrator;
  deltaVa = kpPhiPrime*ePhiPrime + kiPhiPrime*phiIntegrator;

  //Defines the motor 1 and 2 voltages
  v1 = (va + deltaVa)/2;
  v2 = (va - deltaVa)/2;

  //Antiwindup and saturation for motor 1
  if (abs(va + deltaVa) > 15.2) {
      v1 = sgn(va+deltaVa)*7.6;
      rhoIntegrator = 0;
      phiIntegrator = 0;
  }

  //Antiwindup and saturation for motor 2
  if (abs(va - deltaVa) > 15.2) {
      v2 = sgn(va-deltaVa)*7.6;
      rhoIntegrator = 0;
      phiIntegrator = 0; 
  }


  //converts output voltages to pwm values
  PWM1 = (abs(v1)/7.6)*255;
  PWM2 = (abs(v2)/7.6)*255;

  //The four if statements ensure the motors spin the correct direction based off the output voltages
  if (v1 < 0) {
      digitalWrite(M1DIR,0);
  }

  if (v2 < 0) {
      digitalWrite(M2DIR,1);
  }

  if (v1 >= 0) {
      digitalWrite(M1DIR,1);
  }

  
  if (v2 >= 0) {
      digitalWrite(M2DIR,0);
  }

  //Writing the pwm values to the motors
  analogWrite(M1PWM, PWM1);
  analogWrite(M2PWM, PWM2);

  //Read in time values
  Ts = millis() - Tc;
  Tc = millis();

  
  Serial.print(setRhoPrime);
  Serial.print("\t");
  Serial.println("");

}

//Facilitates the commuincation between Arduino and MATLAB
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    InputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      StringComplete = true;
    }
  }
}

//Function to check the sign of a value
int sgn(float uNumber) {
  int sign;
  if (uNumber >= 0) {
    sign = 1;
    return sign;
  }

  else {
    sign = -1;
    return sign;
  } 
}

/*void receiveData(){
    while(Wire.available()) {
      data = Wire.read();
      data = data / 100;
    }
    return data;
 }*/
