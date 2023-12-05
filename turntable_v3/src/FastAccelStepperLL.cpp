#include "FastAccelStepper.h"
#include "HardwareSerial.h"

/* In all calculations, time unit will be 10^-2s
   If we had time in ms, the calculation of step at high speed during acceleration would give
   s = accel * t * t = 2000*4000*4000 > 2^32. Which leads to an overflow
   Being in 10^-2s allows to avoid this phenomenum and to keep a good speed discrimination for small t */

void Stepper::runCommand() {  
  uint16_t timerTicks = TCNT1;
  if ((timerTicks -_previousTicks) >= _currentTicksPerStep){
    togglePin9();
    _previousTicks += _currentTicksPerStep; //update ticks of last target toggle
    _remainingStageSteps --;
    if(_remainingStageSteps == 0){// we have performed all steps of current entry
      computeNextAction();
    }
  }
}

inline void Stepper::computeNextAction(){
  //_currentTicksPerStep = stepperQueue.ticks_at_queue_end;
  uint32_t deceleration;
  uint32_t nextPerformedTime;
  uint32_t planningSteps;
  uint16_t meanSpeed;
  switch (_currentOperation){
    case START_MOTOR:
        startMotor();

    case ACCELERATE:
        accelerate();
        break;
    
    case CONTINUE_AT_CONST_SPEED:
        continueAtConstantSpeed();
        break;

    case DECELERATE_TO_STOP:
        decelerateToStop();
        break;

    case STOP_MOTOR:
        stopMotor();

    case REST:
      break;
  }
}

//*********************************** Acceleration *******************************************************

void Stepper::startMotor(){
    _ptrSer->println("Starting motor ...");
    _isRunning = true;
    _performedAccelerationTime = 10;
    _performedAccelerationSteps = 10;
    _remainingStageSteps = 10;
    _remainingSteps -= 10;
    computeNextCommand(10);
    _previousTicks = TCNT1;
    _currentTicksPerStep = 1250;
    _currentDir = _nextDir;
    _currentOperation = ACCELERATE;
    digitalWrite(DIR_PIN,_currentDir);
    digitalWrite(EN_PIN, LOW);
    
}

void Stepper::computeNextAccelerationStage(uint16_t planningSteps, uint16_t nextPerformedTime){
    uint32_t meanSpeed = _accel*_performedAccelerationTime/100; //dividing by 100 to convert nextPerformedTime^2 in seconds^2
    _currentTicksPerStep = _divRound(_ticksPerSecond,meanSpeed);
    _performedAccelerationSteps += planningSteps;
    _performedAccelerationTime = nextPerformedTime;
    computeNextCommand(planningSteps);
    _remainingSteps -= planningSteps;
}

void Stepper::accelerate(){
    uint16_t nextPerformedTime = _performedAccelerationTime + STAGE_DURATION;
    uint16_t planningSteps = _accel*nextPerformedTime*nextPerformedTime/20000-_performedAccelerationSteps; //dividing by 20000 to convert nextPerformedTime^2 in seconds^2
    if (_remainingSteps-planningSteps < _decelerationSteps){
      planningSteps = _remainingSteps-_decelerationSteps;
      computeNextAccelerationStage(planningSteps, nextPerformedTime);
    }
    else if (_performedAccelerationSteps + planningSteps >= _rampSteps){
      planningSteps = _rampSteps-_performedAccelerationSteps;
      computeNextAccelerationStage(planningSteps, nextPerformedTime);
      startConstantSpeed();
    }
    else{
      computeNextAccelerationStage(planningSteps, nextPerformedTime);
    }
    
    if (_remainingSteps == _decelerationSteps) {
        startDeceleration();
    }
}

//*********************************** Constant speed *******************************************************

void Stepper::startConstantSpeed(){
    _ptrSer->println("VC");
    _currentOperation = CONTINUE_AT_CONST_SPEED;
    _currentTicksPerStep = _cruiseTicksPerStep;
}

void Stepper::continueAtConstantSpeed(){
    uint16_t planningSteps = _stepsPerStageAtCruiseSpeed;
    if (_remainingSteps-planningSteps < _decelerationSteps){
      planningSteps = _remainingSteps - _decelerationSteps;
    }
    computeNextCommand(planningSteps);
    _remainingSteps -= planningSteps;

    if (_remainingSteps == _decelerationSteps) {
      startDeceleration();
    }
}

//*********************************** Deceleration *******************************************************

void Stepper::startDeceleration(){
    _ptrSer->println("DEC");
    _currentOperation = DECELERATE_TO_STOP;
    if(_performedDecelerationSteps = _rampSteps-_decelerationSteps){
        uint32_t meanSpeed = _accel*_performedAccelerationTime/100; //dividing by 100 to convert nextPerformedTime^2 in seconds^2
        _performedDecelerationTime = (_cruiseSpeed-meanSpeed)*100/_accel;
        //_performedDecelerationTime = 100*(_cruiseSpeed/_accel - sqrt(2.*float(_decelerationSteps)/float(_accel)));
    }
    else{
        _performedDecelerationTime = 0;
    }
}

void Stepper::decelerateToStop(){
    uint16_t nextPerformedTime = _performedDecelerationTime + STAGE_DURATION;
    uint16_t planningSteps = _cruiseSpeed*nextPerformedTime/100 - _accel*nextPerformedTime*nextPerformedTime/20000 -_performedDecelerationSteps;
    planningSteps = min(planningSteps, _rampSteps - _performedDecelerationSteps);
    uint16_t meanSpeed = _cruiseSpeed - _accel*(_performedDecelerationTime + nextPerformedTime)/200;
    _currentTicksPerStep = _divRound(_ticksPerSecond,meanSpeed);
    _currentTicksPerStep = min(_maxTicksPerStep, _currentTicksPerStep);
    computeNextCommand(planningSteps);
    _performedDecelerationSteps += planningSteps;
    _performedDecelerationTime = nextPerformedTime;

    if (_performedDecelerationSteps == _rampSteps){
      _currentOperation = STOP_MOTOR;
    }
}

void Stepper::stopMotor(){
    _currentOperation = REST;
    _remainingSteps = 0;
    _isRunning = false;
    digitalWrite(EN_PIN, HIGH);
    _ptrSer->print(millis());
    _ptrSer->print("->");
    _ptrSer->println(_currentPosition);
    _computeNextMove();
}

void Stepper::computeNextCommand(uint16_t planningSteps){
  _remainingStageSteps = planningSteps;
  if (((_currentPosition + planningSteps)>=uint32_t(MAXPOS)) && (_currentDir == CLOCKWISE)){
    _currentPosition = _currentPosition + planningSteps - MAXPOS;
  }
  else if ((_currentPosition < planningSteps) && (_currentDir == ANTICLOCKWISE)){
    _currentPosition = MAXPOS - (planningSteps-_currentPosition);
  }
  else{
    if(_currentDir == CLOCKWISE){
      _currentPosition += planningSteps;
    }
    else{
      _currentPosition -= planningSteps;
    }
  }
  //_ptrSer->println(_currentPosition);
}
