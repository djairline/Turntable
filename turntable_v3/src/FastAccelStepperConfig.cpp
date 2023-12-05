#include "FastAccelStepper.h"
#include "HardwareSerial.h"

void Stepper::init(HardwareSerial *ptrSer) {
  configureTimer();
  setEnablePin();
  setStepPin();
  setDirPin();
  _ptrSer = ptrSer;
  _targetTrack = 0;
  _currentDir = 0;
  _currentPosition = 0;
  _currentOperation = REST;
  _isRunning = false;
}

void Stepper::configureTimer(){
  noInterrupts();
  //Set timer to normal mode <=> Set WGM13:0 
  TCCR1A &= ~(_BV(WGM11) | _BV(WGM10));
  TCCR1B &= ~(_BV(WGM13) | _BV(WGM12));
  // Set prescaler to 64
  uint8_t prescaler = 64;
  TCCR1B = (TCCR1B & ~_BV(CS12)) | _BV(CS11)| _BV(CS10); //TCCR1B = XXXXX011
  _ticksPerSecond = TICKS_PER_S/prescaler;
  // disable OVF interrupt
  TIMSK1 &= ~_BV(TOIE1);
  interrupts();
}

void Stepper::setEnablePin() {
  digitalWrite(EN_PIN, HIGH);
  pinMode(EN_PIN, OUTPUT);
}

void Stepper::setStepPin() {
  digitalWrite(STEP_PIN, LOW);
  pinMode(STEP_PIN, OUTPUT);
}

void Stepper::setDirPin() {
  digitalWrite(DIR_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
}

void Stepper::setSpeed(uint32_t speed) {
  _cruiseSpeed = speed;
  _cruiseTicksPerStep = _ticksPerSecond/speed;
  _stepsPerStageAtCruiseSpeed = _cruiseSpeed * STAGE_DURATION/100;
  _update_ramp_steps();
}

void Stepper::setAcceleration(uint32_t accel) {
  _accel = accel;
  _maxTicksPerStep = float(_ticksPerSecond)*sqrt(2./float(_accel))+0.5;
  _update_ramp_steps();
}

void Stepper::_update_ramp_steps() {
  _rampSteps = _cruiseSpeed * _cruiseSpeed / (2*_accel);
}

