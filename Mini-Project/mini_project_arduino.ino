#include<Wire.h>
#define SLAVE_ADDRESS 0x04

int data[32] = {0};
int number = 0;
int state = 0;
int i = 0;

void setup() {
  
// put your setup code here, to run once:
pinMode(13, OUTPUT);
Serial.begin(115200); // start serial for output

//initialize i2c as slave
Wire.begin(SLAVE_ADDRESS);

// define callbacks for i2c communication
Wire.onReceive(receiveData);
Wire.onRequest(sendData);

Serial.println("Ready!");
}

void loop() {
  // put your main code here, to run repeatedly:
delay(100);
}


// callback received for data
void receiveData(int byteCount) {
 i = 0;
 while(Wire.available()) {
    
    data[i] = Wire.read();
    Serial.print(data[i]);
    Serial.print(' ');
    i++;
 }
 i--;

 number= data[1];
       
}


// callback for sending data

void sendData() {
  Wire.write(number);
  
}
