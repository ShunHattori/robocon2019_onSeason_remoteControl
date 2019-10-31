#pragma once
#include "Arduino.h"

typedef enum ASCII
{
  STX = 0x02, // Start of Text
  ETX = 0x03, // End of Text
  ENQ = 0x05, // Enquiry
  ACK = 0x06, // Acknowledge
} ControlCodes;

class UnitProtocol
{
public:
  UnitProtocol(Stream *str);
  int transmit(uint16_t arrayLenght, int *packet);
  int receive(int *variableToStore);
  void setTimeout(uint16_t timeoutTimeInMs);

private:
  Stream *_myStream;
  unsigned long _watchDogInitTime, _watchDogComparePrevTime;
  uint16_t _timeoutMs, _arrayLenght;
  bool _isTransmittable, _isReceivable;
};