// rfm73 receiver
// by Heye Everts <heye.everts.1@gmail.com>
//
// Demonstrates use of the rfm73 library
// Receive bytes from another device and echo
// 
// Refer to the "simple_sender" example for use with this
//
// Created 2 January 2013
// Copyright 2013 Heye Everts

#include <rfm73.h>

void setup(){
  Serial.begin(19200);
  Serial.println("##reset##");
  
  RFM.Begin();  
  RFM.onReceive(receiveEvent);
}

void loop(){  
  //delay for stability
  delay(1);

  //the tick function checks for any received data and calls the receiveEvent
  RFM.tick();
  
}


void receiveEvent(void){
  Serial.print("Len: ");
  Serial.print(RFM.getPacketLength());
  Serial.print("\tContent: ");
  Serial.println((char*)RFM.getRcvBuffer());
}

