#include <Arduino.h>
#include <Swarmbot.h>
#include <Wire.h>
#include <String.h>
SwarmBot steve; 
imu IMU;

int timer = 0;
bool flag = false;
float inputArray[5];

void leftInterrupt(){
  steve.updateLeftEncoder();
}

void rightInterrupt(){
  steve.updateRightEncoder();
}


union BytesToFloat {
    // 'converts' incoming bytes to a float array
    // valueReading: [0] wheelSpeed; [1] wheelAngle; [2] wheelRotation
    byte valueBuffer[20];
    float valueReading[5];    
} converter;



// void receiveEvent(int howMany){
//   Serial.print("Recieved: ");

//   int numOfBytes = Wire.available();
//   Serial.println(numOfBytes);
//   String data; 
//   while(Wire.available()){
//     Serial.print(Wire.read());
//   }
//   Serial.print(data.toFloat());
//   // Serial.print(receiveArray[4]);
// }

void printInfo(){
    for(uint8_t index = 0; index<5; index++){
        Serial.print("The number is: ");
        float data = converter.valueReading[index];
        Serial.println(data);
        inputArray[index] = data;
    }
    // for(uint8_t index = 0; index<20; index++){
    //     Serial.print("Number ");
    //     Serial.print(index);
    //     Serial.print(" is: ");
    //     Serial.println(converter.valueBuffer[index]);
    // }

    flag = false;
}

void receiveEvent(int byteCount){
    for(uint8_t index = 0; index<byteCount; index++){
        converter.valueBuffer[index] = Wire.read();
    }
    
    flag = true;
}

void setup() {
  //IMU.setupIMU();

  Wire.begin(0x8);
  Wire.onReceive(receiveEvent);

  attachInterrupt(steve.getLeftEncoderA(), leftInterrupt, CHANGE);
  attachInterrupt(steve.getLeftEncoderB(), leftInterrupt, CHANGE);

  attachInterrupt(steve.getRightEncoderA(), rightInterrupt, CHANGE);
  attachInterrupt(steve.getRightEncoderB(), rightInterrupt, CHANGE);
  steve.initializePorts();
  Serial.begin(115200);
  
  Serial.print("ready");
  //delay(1000);
}

void loop() {
  //IMU.updateIMU();
  // analogWrite(steve.getLeftIn1(), 0);
  // analogWrite(steve.getLeftIn2(), 160);
  // if(timer < 200){
  //   steve.setMotorSpeed(100,0);
  // }
  // else{
  //   steve.setMotorSpeed(100,100);
  // }
  steve.updateOdometery();
  steve.moveToPID();
  steve.printOdom();
  //steve.moveToPID(0,0);
  //steve.setAngle(90);
  //Serial.print("Left: "); Serial.print(steve.leftEncoderValue); Serial.print(" | Right: "); Serial.println(steve.rightEncoderValue);
  //Serial.println(digitalRead(A1));
  //Serial.println(timer);
 // timer ++;
  if(flag){
    printInfo();
    Serial.println("Recieved: ");
    steve.callibrateOdometery(inputArray);
  } 
  delay(100);
}