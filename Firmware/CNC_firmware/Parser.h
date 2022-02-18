#ifndef Parser_h
#define Parser_h
#include <Arduino.h>

class Parser{
  
  public:
    Parser();
    void parseNext();

    unsigned long start;
    
  private:

    byte byteX = 2;
    byte byteY = 2;
    byte byteZ = 3;
    byte byteE = 3;
    byte byteF = 3;
    byte byteC = 1;
    byte byteA = 2;
    byte coreXY = 1;


    long base_X = long(1)<<(8*byteX - 1);
    long base_Y = long(1)<<(8*byteY - 1);
    
    long base_E = long(1)<<(8*byteE - 1);
    long base_Z = long(1)<<(8*byteZ - 1);
    
    long long_null = -2147483648;
    byte readByteSafe();
    long readValue(byte bytes);
    void sendInt(int cislo);
    void sendIntNano(int cislo);
    void sendInfo();
    int value;
    
};

#endif
