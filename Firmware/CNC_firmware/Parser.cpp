#include "Arduino.h"
#include "Parser.h"
#include "CNC_movement.h"
#include <SoftwareSerial.h>

CNC_movement CNC;
SoftwareSerial nanoSerial(51, 53);

Parser::Parser(){
  Serial.begin(115200);
  nanoSerial.begin(38400);
  CNC_movement CNC;
}

byte Parser::readByteSafe(){//waits until a byte arrives to buffer, than returns it
  while (not Serial.available()){}
  return Serial.read();
}

long Parser::readValue(byte bytes){//returns a number composed of several [bytes] in buffer
  long vraceni = 0;
  for (byte x = 0; x < bytes - 1; x++){
    vraceni += readByteSafe();
    vraceni *= 256;
    //vraceni = vraceni << 8;
  }
  vraceni += readByteSafe();
  return vraceni; 
}

void Parser::sendInt(int cislo){
  Serial.write(byte(cislo/256));
  Serial.write(byte(cislo%256));
}

void Parser::sendIntNano(int cislo){
  nanoSerial.write(byte(cislo/256));
  nanoSerial.write(byte(cislo%256));
}

void Parser::sendInfo(){
  Serial.println("byteX");
  sendInt(byteX);
  Serial.println("byteY");
  sendInt(byteY);
  Serial.println("byteZ");
  sendInt(byteZ);
  Serial.println("byteE");
  sendInt(byteE);
  Serial.println("byteF");
  sendInt(byteF);
  Serial.println("byteC");
  sendInt(byteC);
  /////
  Serial.println("spmmX");
  sendInt(int(CNC.spmm_X*10));
  Serial.println("spmmY");
  sendInt(int(CNC.spmm_Y*10));
  Serial.println("spmmZ");
  sendInt(int(CNC.spmm_Z*10));
  Serial.println("spmmE");
  sendInt(int(CNC.spmm_E*10));
  Serial.println("coreXY");
  sendInt(int(coreXY));
  Serial.println("end");
}
  
void Parser::parseNext(){
  byte command = readValue(byteC);
  Serial.write(42);
  Serial.println(command);
  switch(command){
    case 1:
      CNC.setOldCoordinates();
      CNC.next_X = readValue(byteX) - base_X;
      CNC.next_Y = readValue(byteY) - base_Y;
      CNC.next_E = readValue(byteE) - base_E;
      CNC.moveXYE();
      break;
    case 32:
      CNC.setOldCoordinates();
      CNC.next_X = readValue(byteX) - base_X;
      CNC.next_Y = readValue(byteY) - base_Y;
      CNC.next_E = readValue(byteE) - base_E;
      CNC.moveXYE_start_acc();
      break;
    case 33:
      CNC.setOldCoordinates();
      CNC.next_X = readValue(byteX) - base_X;
      CNC.next_Y = readValue(byteY) - base_Y;
      CNC.next_E = readValue(byteE) - base_E;
      CNC.moveXYE_noAcc();
      break;
    case 34:
      CNC.setOldCoordinates();
      CNC.next_X = readValue(byteX) - base_X;
      CNC.next_Y = readValue(byteY) - base_Y;
      CNC.next_E = readValue(byteE) - base_E;
      CNC.moveXYE_end_acc();
      break;
    case 2:
      CNC.max_Speed = (float(readValue(byteF))/60.0);
      break;
    case 3:
      CNC.setOldCoordinates();
      CNC.next_E = readValue(byteE) - base_E;
      CNC.moveUniversal();
      break;
    case 4:
      CNC.setOldCoordinates();
      CNC.next_Z = readValue(byteZ) - base_Z;
      CNC.moveZ();
      break;
    case 5:
      CNC.setOldCoordinates();
      CNC.next_X = readValue(byteX) - base_X;
      CNC.next_Y = readValue(byteY) - base_Y;
      CNC.moveXYZ_noAcc();
      break;    
    case 6:
      CNC.setOldCoordinates();
      CNC.next_X = readValue(byteX) - base_X;
      CNC.next_Y = readValue(byteY) - base_Y;
      CNC.next_Z = readValue(byteZ) - base_Z;
      CNC.moveXYZ_noAcc();
      break;
    case 7:
      CNC.setOldCoordinates();
      CNC.next_X = readValue(byteX) - base_X;
      CNC.moveUniversal();
      break;
    case 8:
      CNC.setOldCoordinates();
      CNC.next_Y = readValue(byteY) - base_Y;
      CNC.moveUniversal();
      break;
    case 9:
      CNC.setOldCoordinates();
      CNC.next_X = readValue(byteX) - base_X;
      CNC.next_Y = readValue(byteY) - base_Y;
      CNC.next_Z = readValue(byteZ) - base_Z;
      CNC.next_E = readValue(byteE) - base_E;
      CNC.moveUniversal();
      break;
    case 13:
      CNC.setOldCoordinates();
      CNC.next_X = readValue(byteX) - base_X;
      CNC.next_Z = readValue(byteZ) - base_Z;
      CNC.moveXYZ_noAcc();
      break;
    case 14:
      CNC.setOldCoordinates();
      CNC.next_Y = readValue(byteY) - base_Y;
      CNC.next_Z = readValue(byteZ) - base_Z;
      CNC.moveXYZ_noAcc();
      break;
    case 17:
      CNC.calibrate();
      break;
    case 20:
      CNC.resetE();
      break;
    case 21:
      delay(100);//potreba pro software serial apparently..........
      nanoSerial.write(command);
      sendIntNano(readValue(byteA));
      break;
    case 22:
      delay(100);
      nanoSerial.write(command);
      break;
    case 23:
      delay(100);
      nanoSerial.write(command);
      sendIntNano(readValue(byteA));
      break;
    case 24:
      delay(100);
      nanoSerial.write(command);
      sendIntNano(readValue(byteA));
      while (nanoSerial.available() == 0){
      }
      nanoSerial.read();
      Serial.write(42);
      Serial.println("24 done");
      break;
    case 25:
      delay(100);
      nanoSerial.write(command);
      sendIntNano(readValue(byteA));
      break;
    case 26:
      delay(100);
      value = readValue(byteA);
      nanoSerial.write(command);
      sendIntNano(value);
      while (nanoSerial.available() == 0){
      }
      nanoSerial.read();
      Serial.write(42);
      Serial.println("26 done");
      break;
    case 28:
      CNC.disableMotors();
      break;
    case 29:
      sendInfo();
      break;
  }

  Serial.write(69);
}
  
