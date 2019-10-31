#include "UnitProtocol.hpp"

UnitProtocol::UnitProtocol(Stream *str)
{
  _myStream = str;
  _watchDogInitTime = 0;
  _timeoutMs = 20; // set the timeout time to  20ms
  _isTransmittable = 1;
  _isReceivable = 1;
}

int UnitProtocol::transmit(uint16_t arrayLenght, int *packet)
{
  _arrayLenght = arrayLenght;
  _myStream->write(ENQ);
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
    if (_myStream->read() == ACK)
    {
      break;
    }
  }
  _myStream->write(_arrayLenght);
  char _checkSum = 0;
  for (uint16_t i = 0; i < arrayLenght; i++)
  {
    _myStream->write(packet[i]);
    _checkSum ^= packet[i];
  }
  _myStream->write(_checkSum);
  return 1;
}

int UnitProtocol::receive(int *variableToStore)
{
  if (!(_myStream->available()))
  {
    return 0;
  }
  if (_myStream->read() != ENQ)
  {
    return 0;
  }
  _myStream->write(ACK);
  uint16_t _incomingCounter = 0;
  char _buffer[250], _bufferSum = 0;
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
    break;
  }
  _arrayLenght = _myStream->read();
  for (uint16_t i = 0; i < _arrayLenght; i++)
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
      _buffer[_incomingCounter++] = _myStream->read();
      break;
    }
  }
  if (_arrayLenght != _incomingCounter)
  {
    return 0;
  }
  for (uint16_t i = 0; i < _incomingCounter; i++)
  {
    _bufferSum ^= _buffer[i];
  }
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
    if (_myStream->read() != _bufferSum)
    {
      return 0;
    }
    break;
  }
  for (uint16_t i = 0; i < _incomingCounter; i++)
  {
    variableToStore[i] = _buffer[i];
  }
  return 1;
}

void UnitProtocol::setTimeout(uint16_t timeoutTimeInMs)
{
  _timeoutMs = timeoutTimeInMs;
}