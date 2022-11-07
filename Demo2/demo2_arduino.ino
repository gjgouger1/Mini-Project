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
float kpRhoPrime = 60;
float kiRhoPrime = 0.1;
float rhoIntegrator = 0;
float eRhoPrime;

//Variables for angular velocity PI controller
float setPhiPrime;
float finalPhiPrime;
float kpPhiPrime = 9.7;
float kiPhiPrime = 0;
float phiIntegrator = 0;
float ePhiPrime;
float delta = 0;

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
float setPhi = 0.4;
float phi = 0;
float ePhi = 0;
float phiKp = 2;
float phiKi = 0.001;
float phiKd = 0.01;
float positionIntegrator = 0;
float phiDerivative = 0;
float ePhiPast = 0;

//Variables to define parameters for straight motion
float setForwardDistance = 0;
float rho = 0;
float distanceMultiplier = 0;
signed char data = 0;
float last_data = 0;
int counter = 0;

//Defines encoders 1 and 2 and their respective pin connections
Encoder ENC1(3, 5);
Encoder ENC2(2, 6); 

signed char data_test[32] = {0};
int i_test = 0;
float dist = 0;
float angle_desired = 0;
bool started_receiving_data = false;
bool once = false;
bool aruco_off_frame = false;
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
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
}

void loop() {

  //Demo2 data trasnfer code
//  if (Wire.available()){
  
  
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
 if (started_receiving_data == true){
  if (once == false){
     ENC1.write(0);
     ENC2.write(0);
     once = true;
//     Serial.println("HIIIIIII");
  }
  
   if (ePhi != 0){
//        Serial.println("angle");
//        Serial.println(angle_desired);
//        Serial.println("Phi");
//        Serial.println(phi);
        
//       
        setPhi = angle_desired;  
    }  
  
  }
  
  phi = radius*(theta1-theta2)/distance;
  /*if (phi > 0.52){
    phi = 0;
    ENC1.write(0);
    ENC2.write(0);
    setPhiPrime = 0;
    finalPhiPrime = 0;
    phiIntegrator = 0;
    positionIntegrator = 0;
    phiDerivative = 0;
    delay(1000);
    Ts = 0;
    Tc += 1000;
  }*/
//  if (phi % 0.717)
  if (setPhi == 0){
    phi = 0;
  }
//  setPhi = 3;
//Serial.println("setphi");
//
//Serial.println(setPhi);
//Serial.println("phi");
//Serial.println(phi);
 
  ePhi = setPhi - phi; //phi error
  
//  Serial.print(ePhi);
//  Serial.print('\t');
  //delta = (float)ePhi % (float)0.5235
 
//  Serial.println(ePhi);
//  Serial.println("ephi");
  if (ePhi < 0.01 && ePhi > -0.01){
    
    if (started_receiving_data == true){
//        Serial.println("Hello");
//        ENC1.write(0);
//        ENC2.write(0);
        setPhi = angle_desired;
        setForwardDistance = dist;
        setRhoPrime = 0.157;
        finalRhoPrime = 0.157;
        //Serial.println(dist);
        //Serial.println(angle_desired);
        if (dist == 0 && angle_desired == 0){
          counter++;
          Serial.println(counter);
        }
        if (counter >= 5){
          //Serial.println("made it inside counter");
          Serial.println(setPhiPrime);
          Serial.println(setRhoPrime);
          counter = 0;
          //setRhoPrime = 0;
          //finalRhoPrime = 0;
          aruco_off_frame = true;
        }
    }
     
//        Serial.println(setForwardDistance);
//        Serial.println(setRhoPrime);

    
    if (started_receiving_data == false){
//      ePhi = 0;
//      Serial.println("started receiving data is false");
      ENC1.write(0);
      ENC2.write(0);
      setPhi = 0.4;
      setForwardDistance = 0;
      setRhoPrime = 0;
      finalRhoPrime = 0;
    }
//    ENC1.write(0);
//    ENC2.write(0);
  }
  if (aruco_off_frame == true){
          aruco_off_frame = false;
//          started_receiving_data = false;
          Serial.println("hardcoding distance");
          setForwardDistance = 1;
          setRhoPrime = 0.157;
          finalRhoPrime = 0.157;
        }
  
  positionIntegrator = positionIntegrator + ePhi*Ts;

//  Serial.print(positionIntegrator);
//  Serial.print('\t');
  
  if (Ts > 0) {
    phiDerivative = (ePhi - ePhiPast)/Ts;
    ePhiPast = ePhi;
  }
  else {
    phiDerivative = 0;
  }

//Serial.print(phiDerivative);
//  Serial.print('\t');
  

  //Output of the PID controller is fed into the input variable for angular velocity controller
  setPhiPrime = phiKp*ePhi + phiKi*positionIntegrator + phiKd*phiDerivative;
  finalPhiPrime = setPhiPrime;

  if(setPhiPrime > 1) {
    setPhiPrime = 1*sgn(setPhiPrime);
    finalPhiPrime = 1*sgn(setPhiPrime);
    positionIntegrator = 0;
    
    
  }

  //Calculates the actual forward and angular velocitis
  rhoPrime = radius*(thetadot1 + thetadot2)/2;
  phiPrime = radius*(thetadot1 - thetadot2)/distance;

  //Conditional for stationary rotation with straight motion afterward 
  /*if(ePhi <= 0) {
    setRhoPrime = 0.157;
    finalRhoPrime = 0.157;
  }*/
//  Serial.println("forward distance");
//  Serial.println(setForwardDistance);
//  Serial.println("Rho Prime");
//  Serial.println(setRhoPrime);
//  Serial.println("EPhi");
//  Serial.println(ePhi);
  //Condtional to cause incremental (by tenths) forward velocity increases (slow the step function)
  if(i <= 10) {
    setRhoPrime = setRhoPrime*0.1*i;
    i++;
  }

  else {
    setRhoPrime = finalRhoPrime;
  }

  //Condtional to cause incremental (by tenths) angular velocity increases (slow the step function)
  if(j <= 4) {
    setPhiPrime = setPhiPrime*0.25*j;
    j++;
  }

  else {
    setPhiPrime = finalPhiPrime;
  }

  
  //Conditional for travelling along an arc with straight motion afterward or just straight motion (if setPhi is set equal to zero)
  if ((ePhi <= 0) && (setRhoPrime != 0)) {

      //Converts meters to feet
//      setForwardDistance = 0;

      //setForwardDistance = 1*0.305*2;

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
    //Serial.print("made it");
    setRhoPrime = 0;
    finalRhoPrime = 0;
    setForwardDistance = 0;
  }

  //Prevents overshoot on the angle turn
  if(ePhi <= 0.005) {
    positionIntegrator = 0;
    phiDerivative = 0;
  }

//  Serial.println(setPhiPrime);
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

//Serial.println(v2);
  //Antiwindup and saturation for motor 1
  if (abs(va + deltaVa) > 7.6) {
      v1 = sgn(va+deltaVa)*7.6;
      rhoIntegrator = 0;
      phiIntegrator = 0;
  }

  //Antiwindup and saturation for motor 2
  if (abs(va - deltaVa) > 7.6) {
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

  
  //Serial.print(setRhoPrime);
  //Serial.print("\t");
  //Serial.println("");

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

void receiveData(int byteCount){
    i_test = 0;
    while(Wire.available()) {
      
//      started_receiving_data = true;
//      Serial.println("BEGINNING LOOP");
      //last_data = data;
      data_test[i_test % 2] = Wire.read();
      if (data_test[i_test % 2] != 0){
        started_receiving_data = true;
      }
//      Serial.println("DATA");
//      Serial.println((data_test[i_test % 2]));
      
      //temp = s & 0b1000000;
      //Serial.println(data);
//      if (ePhi != 0) {
        if ((i_test % 2) == 0){
          angle_desired = -1 * (float)data_test[i_test % 2] * 0.003682;
        }
//      }
      if ((i_test % 2) == 1){
        dist = data_test[i_test % 2] * 0.1;
      }
      i_test++;
  
//      Serial.println(("TEMP"));
//      Serial.println(angle_desired);
//      Serial.println(("DIST"));
//
//      Serial.println(dist);
    }
}
