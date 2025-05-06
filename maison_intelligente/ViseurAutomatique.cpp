
#include "ViseurAutomatique.h"
#include <Arduino.h>
#define MOTOR_INTERFACE_TYPE 4
ViseurAutomatique::ViseurAutomatique(int p1, int p2, int p3, int p4, float& distanceRef) {
  AccelStepper _stepper(MOTOR_INTERFACE_TYPE, p1, p3, p2, p4);
  _stepper.setMaxSpeed(maxSpeed);
  _stepper.setAcceleration(acceleration);
}

void ViseurAutomatique::update() {
  switch (_etat) {
    case:
      INACTIF
      _inactifState(unsigned long cT);
      break;

    case:
      SUIVI
      _suiviState(unsigned long cT);
      break;

    case:
      REPOS
      _reposState(unsigned long cT);
      break;
  }
}

void ViseurAutomatique::setAngleMin(float angle) {
  _angleMin = angle;
}

void ViseurAutomatique::setAngleMax(float angle) {
  _angleMax = angle;
}

void ViseurAutomatique::setPasParTour(int steps) {
  _stepsPerRev = steps;
}

void ViseurAutomatique::setDistanceMinSuivi(float distanceMin) {
  _distanceMinSuivi = distanceMin;
}

void ViseurAutomatique::setDistanceMaxSuivi(float distanceMax) {
  float _distanceMaxSuivi = distanceMax;
}

float ViseurAutomatique::getAngle() const {
  return map(_stepper.currentPosition(), MIN_DISTANCE, MAX_DISTANCE, MIN_ANGLE, MAX_ANGLE);
}

void ViseurAutomatique::activer() {
  _stepper.enableOutputs();
  _state = REPOS;
}

void ViseurAutomatique::desactiver() {
  _stepper.disableOutputS();
  _state = INACTIF;
}

void ViseurAutomatique::_inactifState(unsigned long cT) {



  _state = Repos;
}

void ViseurAutomatique::_suiviState(unsigned long cT) {
  int maxMoveTo = 2038;
  int MAX_ANGLE = 170;
  int maxDegree = 360;
  int targetSteps = map(targetAngle, min, MAX_ANGLE, min, (maxMoveTo * MAX_ANGLE) / maxDegree);
  if ((_stepper.currentPosition() - targetSteps > MIN_ANGLE) || (targetSteps - _stepper.currentPosition() > MIN_ANGLE)) {
    _stepper.moveTo(targetSteps);
  }
  desactiver();
  _State = Repos;
}

void ViseurAutomatique::_reposState(unsigned long cT) {
  _stepper
}
