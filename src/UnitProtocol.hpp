#pragma once
#include "Arduino.h"

typedef enum ASCII
{
    STX = 0x02, //Start of Text
    ETX = 0x03, //End of Text
    ENQ = 0x05, //Enquiry
    ACK = 0x06, //Acknowledge
} ControlCodes;

class UnitProtocol
{
public:
    UnitProtocol(Stream *str);
    void transmit(int arrayLenght, int *packet);
    void receive(int *variableToStore);
    void setTimeout(int timeoutTimeInMs);
    void addDataFlowLED(int LEDPin, char *whichDataFlow);

private:
    Stream *_myStream;
    unsigned long _watchDogInitTime, _watchDogComparePrevTime;
    int _TXLEDPin, _RXLEDPin;
    int _timeoutMs, _arrayLenght;
    bool _isTransmittable, _isReceivable;

    void ENQsend(char data);
    char ACKreceive();
};