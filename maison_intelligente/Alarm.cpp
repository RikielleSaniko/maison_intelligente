
#include "Alarme.h"
#include <Arduino.h>

Alarm::Alarm(int rPin, int gPin, int bPin, int buzzerPin, float& distancePtr) {
  _rPin = rPin;
  _gPin = gPin;
  _bPin = bPin;
  _buzzerPin = buzzerPin;
  _distance = distancePtr;
  pinMode(_buzzerPin, OUTPUT);
  pinMode(_rPin, OUTPUT);
  pinMode(_bPin, OUTPUT);
}

void Alarm::setVariationTiming(unsigned long ms) {
  _variationRate = ms;
}

void Alarm::setDistance(float d) {
  _distanceTrigger = d;
}

void Alarm::getDistance() {
  return _distance;
}

void Alarm::setTimeout(unsigned long ms) {
  _timeoutDelay = ms;
}

void Alarm::turnOn() {
  _turnOnFlag = true;
}
void Alarm::turnOff() {
  _turnOffFlag = true;
}

void Alarm::update() {
  switch (_state) {
    case ON:

      _onState();
      break;

    case OFF:
      _offState();
      break;

    case WATCHING:
      _watchState();
      break;

    case TESTING:
      _testingState();
      break;
  }
}

void Alarm::_offState() {
  _turnOff();
  _turnOffFlag = false;
  _state = WATCHING;
}

void Alarm::_watchState() {
  if (_turnOffFlag) {
    _state = OFF;
  }
  if (_distance <= distanceTrigger) {
    _state = ON;
  }
}

void Alarm::_onState() {
  unsigned long currentTime = millis();
  static unsigned long previousledMillis = 0;
  digitalWrite(buzzer, HIGH);

  if (currentTime - previousledMillis >= _timeoutDelay) {

    previousledMillis = currentTime;

    if (_currentColor) {
      _setRGB(colA[0], colA[1], colA[2]);
    } else {
      _setRGB(colB[0], colB[1], colB[2]);
    }
    _currentColor = !_currentColor;
  }
  _state = WATCHING;
  turnOff();
}

void Alarm::setColourA(int r, int g, int b) {
  _colA[0] = r;
  _colA[1] = g;
  _colA[2] = b;
}

void Alarm::setColourB(int r, int g, int b) {
  _colB[0] = r;
  _colB[1] = g;
  _colB[2] = b;
}

void Alarm::_setRGB(int r, int g, int b) {
  analogWrite(_rPin, r);
  analogWrite(_gPin, g);
  analogWrite(_bPin, b);
}

void Alarm::_turnOff() {
  digitalWrite(_gPin, LOW);
  digitalWrite(_rPin, LOW);
  digitalWrite(_bPin, LOW);
  digitalWrite(_buzzerPin, LOW);
}