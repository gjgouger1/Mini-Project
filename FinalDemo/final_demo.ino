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
float kpRhoPrime = 5;  //I changed this.. could be tuned. Literally don't touch these two
float kiRhoPrime = 1; //tune
float rhoIntegrator = 0;
float eRhoPrime;

//Variables for angular velocity PI controller
float setPhiPrime;
float finalPhiPrime;
float kpPhiPrime = 10;

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
double Ts = 0;
double Tc = millis();

//Variable for Demo2 that faciliatates data transfer between Pi and Arduino 
/*float data = 0;*/

//Variables for angular position PID controller
float setPhi = 0.6;
float phi = 0;
float ePhi = 0;
//2.25
float phiKp = 2.7; //tune
//0.001
float phiKi = 0.6; //tune
float phiKd = 0.01; //tune
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
bool marker_was_found = false;
bool moving_forward = false;
bool set_distance_once = false;
bool check_once = false;
int my_arr[] = {0,0,0,0,0};
bool stop_hardcoding = false;
bool garbage_value = false;
int next_state = 0;
int move_state = 0;
int markerID = 12;
float starttime = 0;
int marker_we_are_looking_for = 1;

//Defines encoders 1 and 2 and their respective pin connections
Encoder ENC1(3, 5);
Encoder ENC2(2, 6); 

signed char data_test[32] = {0}; // made unsigned
//uint8_t data_test[32] = {0}; // made unsigned
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


Serial.println(dist);  
  switch(next_state){
    case 0:
      if (abs(ePhi) < 0.02){
          ENC1.write(0); //reset encoders
          ENC2.write(0);
//        ph/iKp = 12;
          if (marker_we_are_looking_for != 7){
            setPhi =  0.600; //rotate our setPhi again
            setForwardDistance = 0;
            setRhoPrime = 0;
            finalRhoPrime = 0;
          }
          if (marker_we_are_looking_for >= 7){
            setPhi =  0; //rotate our setPhi again
            setForwardDistance = 0;
            setRhoPrime = 0;
            finalRhoPrime = 0;
          }
          
          //started_receiving_data = false;
      }
      if (started_receiving_data == true && markerID == marker_we_are_looking_for){ //add markerID if, if(markerID == the one we are looking for) lock into it
        next_state = 1;
      }
      break;
  
    case 1:
      phiKp = 10;
      phiKi = 1.3;
      setPhi = angle_desired;


      break;
   
    
  }


  
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

  switch(move_state){
    case 0: //this handles 
      if (abs(ePhi < 0.06) && started_receiving_data == true && set_distance_once == false && next_state == 1){
        move_state = 1;
      }
      break;
    case 1: //this handles forward distance
      if (dist > 2){
        setForwardDistance = 20; //setting dist
        setRhoPrime = 1;
        finalRhoPrime = 1;

      }

      if (dist < 2 && set_distance_once == false && dist != 0){
        move_state = 2;
      }
      break;
    case 2: //dead recokiong state
  
      set_distance_once = true; //this might not be needed
      //was 0.25
      setForwardDistance = 0.45; //no touchie
      setRhoPrime = 1; //I chose small values so it doesn't zoom (hopefully more accurate??) // possibly increase speed
      finalRhoPrime = 1;
      rho = 0;
      
      move_state = 3;
      break;
    case 3://this case resets variables, we should be at the marker on top of it. so start rotating somehow?
      if (rho >= setForwardDistance){
        starttime = millis();
        marker_we_are_looking_for++;
        
        move_state = 4;
//        marke/ridwewant +=1
        //now we are at the marker adn can do this shiz
      }
      break;
     case 4:
        if ((millis() - starttime) > 5000){
          set_distance_once = false; //this might not be needed  
          //started_receiving_data = false; // new 11/19/2022         
          next_state = 0; //start moving looking for marker
          phi = 0.600; //set to value to move rotationally 
          phiKp = 6; //tune
          phiKi = 1; //tune
          move_state = 0;
          
        }
       break;
  }
  



 //begin all of the controller stuff
  
  positionIntegrator = positionIntegrator + ePhi*Ts/1000;

//  Serial.print(positionIntegrator);
//  Serial.print('\t');
  
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

  if(abs(setPhiPrime) > 1) {
    setPhiPrime = 1*sgn(setPhiPrime);
    finalPhiPrime = 1*sgn(setPhiPrime);
    positionIntegrator = 0;
    
    
  }

  //Calculates the actual forward and angular velocitis
  rhoPrime = radius*(thetadot1 + thetadot2)/2;
  phiPrime = radius*(thetadot1 - thetadot2)/distance;

  //Conditional for stationary rotation with straight motion afterward 
  


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

  


  distanceMultiplier = 0.98;

  //Prevents overshoot on the angle turn
  if(ePhi <= 0.005) {
    positionIntegrator = 0;
    phiDerivative = 0;
  }

//  Serial.println(setPhiPrime);
  //Start of PI controllers for both angular and forward velocity 
  eRhoPrime = setRhoPrime - rhoPrime; //Rhoprime error
  ePhiPrime = setPhiPrime - phiPrime; //Phiprime error


  rhoIntegrator = rhoIntegrator + Ts*eRhoPrime/1000;
  phiIntegrator = phiIntegrator + Ts*ePhiPrime/1000;

  
  //Halts straight motion if rover attains the desired traveled distance 
  if(rho >= distanceMultiplier*setForwardDistance) {
    //setForwardDistance = 0;
    eRhoPrime = 0;
    rhoIntegrator = 0;
   
  }

  //Defines the input voltages for the two motor system
  va = kpRhoPrime*eRhoPrime + kiRhoPrime*rhoIntegrator;
  deltaVa = kpPhiPrime*ePhiPrime + kiPhiPrime*phiIntegrator;

  //Defines the motor 1 and 2 voltages
  v1 = (va + deltaVa)/2;
  v2 = (va - deltaVa)/2;

  

  

//Serial.print(setRhoPrime);
//Serial.println('\t');

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

//  Serial.println(PWM1);
//  Serial.println(PWM2);

  //Read in time values
  Ts = millis() - Tc;
  Tc = millis();
  
  

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

void receiveData(int byteCount){ // make sure values are being sent to correct variables
                                 // i.e. print dist, phi, and markerID, 
    i_test = 0;
   
    while(Wire.available()) {
      
      data_test[i_test] = Wire.read(); // removed mod 2 

      
      if (i_test % 3 == 2 && markerID == marker_we_are_looking_for){

          
          angle_desired = (-1 * (float)data_test[i_test] * 0.003682)+phi; //big math boi
      }
      if (i_test % 3 == 1 && markerID == marker_we_are_looking_for){
        dist = data_test[i_test] * 0.2;
      }
      if (i_test % 3 == 0 ){
        markerID = data_test[i_test]; 
      }
      


      my_arr[i_test % 5] = data_test[i_test % 3]; //this populates the array
      
      
      if (dist != 0 && angle_desired != 0){ //if we receive something other than 0
        started_receiving_data = true;
        marker_was_found = true;

      }
    
        
      
      if (my_arr[0] == 0 && my_arr[1] == 0 && my_arr[2] == 0 && my_arr[3] == 0 && my_arr[4] == 0){ //I don't think this does jack, but it aint broke so idk
          started_receiving_data = false; //this statement is needed but the conditional is wack, I was tryna do the last 5 data points being zero means we are probably just not detecting anything
      }
      

      

      i_test++;
        
  } //end of while loop
  

}
