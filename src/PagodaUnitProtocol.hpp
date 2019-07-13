#pragma once
#include "Arduino.h"

class PagodaUnitProtocol
{
public:
    PagodaUnitProtocol(Stream *str);
    void transmit(int arrayLenght, int *packet);
    void receive(int *variableToStore);
    void setTimeout(int timeoutTimeInMs);
    bool isAvailableToTransmit();
    bool isAvailableToReceive();
    void debugPrint(char *format, ...);
    void addDataFlowLED(int LEDPin, char *whichDataFlow);

private:
    Stream *_myStream;
    unsigned long _watchDogInitTime, _watchDogComparePrevTime;
    int _TXLEDPin, _RXLEDPin;
    int _timeoutMs, _arrayLenght;
    bool _isTransmittable, _isReceivable;
    va_list _args;

    enum ControlCodes
    {
        STX = 0x02, //Start of Text
        ETX = 0x03, //End of Text
        ENQ = 0x05, //Enquiry
        ACK = 0x06, //Acknowledge
    };

    void ENQsend(char data)
    {
        _myStream->write(ENQ);
        _watchDogComparePrevTime = millis();
        while (1)
        {
            if ((millis() - _watchDogComparePrevTime) > _timeoutMs)
            {
                return;
            }
            if (!(_myStream->available()))
            {
                continue;
            }
            if (_myStream->read() == ACK)
            {
                _myStream->write(data);
                return;
            }
        }
    }

    char ACKreceive()
    {
        _watchDogComparePrevTime = millis();
        while (1)
        {
            if ((millis() - _watchDogComparePrevTime) > _timeoutMs)
            {
                return 0;
            }
            if (!(_myStream->available()))
            {
                continue;
            }
            if (_myStream->read() == ENQ)
            {
                _myStream->write(ACK);
                break;
            }
        }
        _watchDogComparePrevTime = millis();
        while (1)
        {
            if ((millis() - _watchDogComparePrevTime) > _timeoutMs)
            {
                return 0;
            }
            if (_myStream->available())
            {
                char receive = _myStream->read();
                return receive;
            }
        }
    }
};