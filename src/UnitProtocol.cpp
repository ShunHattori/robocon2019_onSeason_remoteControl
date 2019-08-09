#include "UnitProtocol.hpp"

UnitProtocol::UnitProtocol(Stream *str) {
  _myStream = str;
  _watchDogInitTime = 0;
  _timeoutMs = 20;  // set the timeout time to  20ms
  _isTransmittable = 1;
  _isReceivable = 1;
}

void UnitProtocol::transmit(int arrayLenght, int *packet) {
  _arrayLenght = arrayLenght;
  if (!_isTransmittable) {
              return;
  }
  ENQsend(STX);
  char _checkSum = 0;
  for (int i = 0; i < arrayLenght; i++) {
    ENQsend(packet[i]);
    _checkSum ^= packet[i];
  }
  ENQsend(_checkSum);
  ENQsend(ETX);
}

void UnitProtocol::receive(int *variableToStore) {
  if (!(_myStream->available())) {
    return;
  }
  char data = ACKreceive();
  if (data != STX) {
    return;
  }
  int _incomingCounter = 0;
  char _buffer[50], _bufferSum = 0;
  while (1) {
    while (!(_myStream->available())) {
      if ((millis() - _watchDogComparePrevTime) > _timeoutMs) {
        return;
      }
    }
    data = ACKreceive();
    if (data == ETX) {
      break;
    }
    _buffer[_incomingCounter++] = data;
  }
  for (int i = 0; i < _incomingCounter - 1; i++) {
    _bufferSum ^= _buffer[i];
  }
  if (_buffer[_incomingCounter - 1] == _bufferSum) {
    for (int i = 0; i < _incomingCounter - 1; i++) {
      variableToStore[i] = _buffer[i];
    }
  }
}

void UnitProtocol::setTimeout(int timeoutTimeInMs) {}

void UnitProtocol::addDataFlowLED(int LEDPin, char *whichDataFlow) {}

void UnitProtocol::ENQsend(char data) {
  _myStream->write(ENQ);
  _watchDogComparePrevTime = millis();
  while (1) {
    if ((millis() - _watchDogComparePrevTime) > _timeoutMs) {
      return;
    }
    if (!(_myStream->available())) {
      continue;
    }
    if (_myStream->read() == ACK) {
      _myStream->write(data);
      return;
    }
  }
}

char UnitProtocol::ACKreceive() {
  _watchDogComparePrevTime = millis();
  while (1) {
    if ((millis() - _watchDogComparePrevTime) > _timeoutMs) {
      return 0;
    }
    if (!(_myStream->available())) {
      continue;
    }
    if (_myStream->read() == ENQ) {
      _myStream->write(ACK);
      break;
    }
  }
  _watchDogComparePrevTime = millis();
  while (1) {
    if ((millis() - _watchDogComparePrevTime) > _timeoutMs) {
      return 0;
    }
    if (_myStream->available()) {
      char receive = _myStream->read();
      return receive;
    }
  }
}