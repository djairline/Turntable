#include "FastAccelStepper.h"
#include "HardwareSerial.h"

//********************************** Mid level API methods **********************************

void Stepper::_computeNextMove(){
  _targetPosition = _targetTrack*STEPS_PER_TRACK;
  if (_currentPosition == _targetPosition){
    _currentOperation = REST;
  }
  else{
    _calculateMove();
  }
}

void Stepper::_calculateMove(){
  int32_t deltaSE = _targetPosition - _currentPosition; //delta Start End
  uint32_t clockwiseGap;
  uint32_t antiClockwiseGap;
  //if deltaSE is positive the clockwise rotation will correspond to currentPosition <= targetPos without passing by 0

  clockwiseGap = _modulo(deltaSE, HALF_THE_NB_OF_ROTATION_STEPS); //the modulo function can give negative result, if so we assure that 0<r< wholeRotatationStep/2
  antiClockwiseGap = _modulo(-deltaSE, HALF_THE_NB_OF_ROTATION_STEPS);

  if (_isRunning){ // If the motor is running, the path needed to invert the direction is penalized by the time to stop the motor
    if (_currentDir == CLOCKWISE){ 
      antiClockwiseGap += _computeStepsToBrake();
    }
    else{
      clockwiseGap += _computeStepsToBrake();
    }
  }

  _ptrSer->print("clockwise gap : ");
  _ptrSer->println(clockwiseGap);
  _ptrSer->print("antiClockwiseGap : ");
  _ptrSer->println(antiClockwiseGap);

  if (clockwiseGap <= antiClockwiseGap){
    _nextDir = CLOCKWISE;
  }
  else{
    _nextDir = ANTICLOCKWISE;
  }

  if(!_isRunning){
    _currentDir = _nextDir;
    _performedAccelerationSteps = 0;
    _remainingSteps = min(clockwiseGap, antiClockwiseGap);
    _decelerationSteps = min(_rampSteps, (_performedAccelerationSteps + _remainingSteps) / 2);//??
  }
  else if (_currentDir == _nextDir){
    computePerformedAcceleration();
    _remainingSteps = min(clockwiseGap, antiClockwiseGap);
    _decelerationSteps = min(_rampSteps, (_performedAccelerationSteps + _remainingSteps) / 2);//??
    //_ptrSer->println("Running ...");
    //_ptrSer->print("PerformedAccelerationSteps : ");
    //_ptrSer->println(_performedAccelerationSteps);
    //_ptrSer->print("PerformedAccelerationTime : ");
    //_ptrSer->println(_performedAccelerationTime);
  }
  _setCurrentOperation();
}

inline void Stepper::_setCurrentOperation(){
  if (_isRunning){
    if ((_remainingSteps <= _decelerationSteps) || (_currentDir != _nextDir)){
      _currentOperation = DECELERATE_TO_STOP;
      _ptrSer->println("De");
      computePerformedDeceleration();
    }
    else if( _currentTicksPerStep > _cruiseTicksPerStep){
        _currentOperation = ACCELERATE;
        _ptrSer->println("Acc");
    }
    else{
      _currentOperation = CONTINUE_AT_CONST_SPEED;
      _ptrSer->println("VC");
    }
  }
  else{ // not running
      _currentOperation = START_MOTOR;
  }
  debug();
  computeNextAction();
  
}

uint32_t Stepper::_computeStepsToBrake(){//to be implemented
  if (_currentTicksPerStep<=_cruiseTicksPerStep){
    return _rampSteps;
  }
  else{
    float t = float(_cruiseSpeed - _ticksPerSecond/_currentTicksPerStep)/float(_accel);//v = vconst - at
    //tf = _cruiseSpeed/a
    //breakTime = ticksPerSecond/(currentTicksPerStep*a)
    uint16_t performedDecelerationSteps = uint16_t(-0.5*float(_accel) *t*t + float(_cruiseSpeed) * t);
    return _rampSteps-performedDecelerationSteps;
  } 
}

void Stepper::computePerformedDeceleration(){
    _performedDecelerationTime = (_cruiseSpeed - _ticksPerSecond/_currentTicksPerStep)*100/_accel;
    _performedDecelerationSteps = _cruiseSpeed*_performedDecelerationTime/100 - _accel*_performedDecelerationTime*_performedDecelerationTime/20000;
}

void Stepper::computePerformedAcceleration(){ 
    _performedAccelerationTime = (_ticksPerSecond/uint32_t(_currentTicksPerStep))*100/_accel;
    _performedAccelerationSteps = _accel *_performedAccelerationTime*_performedAccelerationTime/20000;
}


void Stepper::debug(){
  _ptrSer->print("Target track = ");
  _ptrSer->print(_targetTrack);
  _ptrSer->print(" sur ");
  _ptrSer->println(NB_OF_TRACKS);
  _ptrSer->print("Target position = ");
  _ptrSer->println(_targetPosition);
  _ptrSer->print("Current direction : ");
  _ptrSer->println(_currentDir);
  _ptrSer->print("Current state : ");
  _ptrSer->println(_currentOperation);
  _ptrSer->print("Current ticks per step : ");
  _ptrSer->println(_currentTicksPerStep);
  _ptrSer->print("Current position : ");
  _ptrSer->println(_currentPosition);
  _ptrSer->print("Remaining Steps : ");
  _ptrSer->println(_remainingSteps);
  _ptrSer->print("Deceleration Steps : ");
  _ptrSer->println(_decelerationSteps);
  _ptrSer->print("Is Running : ");
  _ptrSer->println(_isRunning);
  _ptrSer->print("Max pos : ");
  _ptrSer->println(uint32_t(MAXPOS));
}