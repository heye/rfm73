// rfm73 simple_sender
// by Heye Everts <heye.everts.1@gmail.com>
//
// Demonstrates use of the rfm73 library
// Send data and print received with serail
// 
// Refer to the "echo" example for use with this
//
// Created 2 January 2013
// Copyright 2013 Heye Everts

#include <rfm73.h>

void setup(){
  //Serial connection for debugging
  Serial.begin(19200);
  Serial.println("##reset##");
  
  RFM.Begin();  
  RFM.onReceive(receiveEvent);
}


byte array[32] = "1234567890123456789012345678901";

void loop(){  
  //send some text with length, the max length is 32 bytes!
  RFM.send((byte*)"hello!", 6);
  delay(100);
  
   //the tick function checks for any received data and calls the receiveEvent
  RFM.tick();
  
  //another way to send something
  RFM.send(array, 32);
  delay(100);  
  
   //the tick function checks for any received data and calls the receiveEvent
  RFM.tick();
  
}

void receiveEvent(void){
  //print received data
  Serial.println((char*)RFM.getRcvBuffer());
}

