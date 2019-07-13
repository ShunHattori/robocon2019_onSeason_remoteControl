#include "PagodaUnitProtocol.hpp"

PagodaUnitProtocol::PagodaUnitProtocol(Stream *str)
{
    _myStream = str;
    _watchDogInitTime = 0;
    _timeoutMs = 20; //set the timeout time to 500ms
    _isTransmittable = 1;
    _isReceivable = 1;
}

void PagodaUnitProtocol::transmit(int arrayLenght, int *packet)
{
    _arrayLenght = arrayLenght;
    if (!_isTransmittable)
    {
        return;
    }
    ENQsend(STX);
    char _checkSum = 0;
    for (int i = 0; i < arrayLenght; i++)
    {
        ENQsend(packet[i]);
        _checkSum ^= packet[i];
    }
    ENQsend(_checkSum);
    ENQsend(ETX);
}

void PagodaUnitProtocol::receive(int *variableToStore)
{
    if (!(_myStream->available()))
    {
        return;
    }
    char data = ACKreceive();
    if (data != STX)
    {
        return;
    }
    int _incomingCounter = 0;
    char _buffer[50], _bufferSum = 0;
    while (1)
    {
        while (!(_myStream->available()))
        {
            if ((millis() - _watchDogComparePrevTime) > _timeoutMs)
            {
                return;
            }
        }
        data = ACKreceive();
        if (data == ETX)
        {
            break;
        }
        _buffer[_incomingCounter++] = data;
    }
    for (int i = 0; i < _incomingCounter - 1; i++)
    {
        _bufferSum ^= _buffer[i];
    }
    if (_buffer[_incomingCounter - 1] == _bufferSum)
    {
        for (int i = 0; i < _incomingCounter - 1; i++)
        {
            variableToStore[i] = _buffer[i];
        }
    }
}

void PagodaUnitProtocol::setTimeout(int timeoutTimeInMs)
{
}

bool PagodaUnitProtocol::isAvailableToTransmit()
{
}

bool PagodaUnitProtocol::isAvailableToReceive()
{
}

void PagodaUnitProtocol::debugPrint(char *format, ...)
{
}

void PagodaUnitProtocol::addDataFlowLED(int LEDPin, char *whichDataFlow)
{
    if (whichDataFlow == "tx" || whichDataFlow == "TX")
    {
        _TXLEDPin = LEDPin;
        pinMode(_TXLEDPin, OUTPUT);
    }
    else if (whichDataFlow == "rx" || whichDataFlow == "RX")
    {
        _RXLEDPin = LEDPin;
        pinMode(_RXLEDPin, OUTPUT);
    }
}
