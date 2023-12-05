#ifndef FASTACCELSTEPPER_H
#define FASTACCELSTEPPER_H

#include <Arduino.h>
#include <stdint.h>
#include "HardwareSerial.h"

// MOTION DRIVER AND STEPPER PROPERTIES
#define TURNTABLE_SPEED 1 // in RPM
#define STEPS_PER_MOTOR_ROTATION 200 //number of steps of the motor
#define MICROSTEPS 256 //number of micro-steps in the driver
#define REDUCTION 9.6 //turntableSpeed/motorSpeed
#define NB_OF_TRACKS_ACCEL 3 //Number of track crossed to reach const_speed
#define NB_OF_TRACKS 48
#define STEPS_PER_TABLE_ROTATION (STEPS_PER_MOTOR_ROTATION*REDUCTION) // Warning : STEPS_PER_TABLE_ROTATION must be divible by 2 and by NB_OF_TRACKS
#define STEPS_PER_TRACK (STEPS_PER_TABLE_ROTATION/NB_OF_TRACKS) // steps to cross one track
#define STEPS_PER_TRACK (STEPS_PER_TABLE_ROTATION*MICROSTEPS/NB_OF_TRACKS)
#define HALF_THE_NB_OF_TRACKS (NB_OF_TRACKS/2)
#define HALF_THE_NB_OF_ROTATION_STEPS (STEPS_PER_TRACK*HALF_THE_NB_OF_TRACKS)
#define MAXPOS (HALF_THE_NB_OF_ROTATION_STEPS) //to hardcode
// WIRING
#define STEP_PIN 9 //DO NOT CHANGE THIS VALUE
#define DIR_PIN 3 // you can use all pin
#define EN_PIN 4 // you can use all pin


//TIMER
#define TICKS_PER_S F_CPU
#define MIN_DELTA_TICKS (F_CPU / 50000)
#define ABSOLUTE_MAX_TICKS (65535)
#define STAGE_DURATION 2 //each stage is 20ms long and time unit is 10^-2s

//STATES OF MOTOR
#define REST 0
#define START_MOTOR 1
#define ACCELERATE  2
#define CONTINUE_AT_CONST_SPEED 3
#define DECELERATE_TO_STOP 4
#define STOP_MOTOR 5

//MOTOR DIRECTION
#define CLOCKWISE 1
#define ANTICLOCKWISE 0 

//Algorithm layers
//Configuration : setSpeed, setAcceleration
//High level API methods : moveForNTracks, moveTo, moveAtConstSpeed 
//Mid level API : calculateMove and set current state
//Low level API : queue storing the number of ticksPerStep for a given amount of next steps


class Stepper {
 public:
  //**********************************Iniatialisation and setup method**********************************
  void init(HardwareSerial *ptrSer); 
  void configureTimer();
  void setEnablePin();
  void setStepPin();
  void setDirPin();
  void setSpeed(uint32_t speed);
  void setAcceleration(uint32_t accel);
  void _update_ramp_steps();

//********************************** High level API methods **********************************
  void moveForNTracks(uint8_t nTracks, bool dir);
  void moveTo(uint8_t track);
  void moveAtConstSpeed(bool dir);
  void stop();
  void emergencyStop();
  void handleCommands();

 private:
  HardwareSerial* _ptrSer;
  uint32_t _accel;
  uint32_t _cruiseSpeed;

  uint8_t _currentOperation;
  uint8_t _targetTrack;
  uint32_t _targetPosition;
  uint32_t _currentPosition; //position at the end ongoing command
  bool _currentDir;
  bool _nextDir;
  bool _isRunning;

  uint32_t _ticksPerSecond;
  uint16_t _cruiseTicksPerStep;
  uint32_t _stepsPerStageAtCruiseSpeed;
  uint16_t _maxTicksPerStep;
  uint16_t _previousTicks;
  uint16_t _currentTicksPerStep;
  uint32_t _remainingSteps;
  uint16_t _decelerationSteps;
  uint16_t _rampSteps; // nb of ramp steps from 0 to max speed

  uint32_t _performedAccelerationTime;
  uint32_t _performedDecelerationTime;
  uint16_t _performedAccelerationSteps;
  uint16_t _performedDecelerationSteps;
  uint16_t _remainingStageSteps;

//********************************** Mid level API methods **********************************

  void _computeNextMove();
  void _calculateMove();
  void _setCurrentOperation();
  uint32_t _computeStepsToBrake();
  void computePerformedDeceleration();
  void computePerformedAcceleration();
  void debug();

//********************************** Low level API methods **********************************

  void runCommand();
  void computeNextAction();
  void startMotor();
  void computeNextAccelerationStage(uint16_t planningSteps, uint16_t nextPerformedTime);
  void accelerate();
  void startConstantSpeed();
  void continueAtConstantSpeed();
  void startDeceleration();
  void decelerateToStop();
  void stopMotor();
  void computeNextCommand(uint16_t planningSteps);
  void Stepper::togglePin9(){
    PORTB ^= _BV(PB1);
  }

//********************************** Tools *************************************************

  inline uint32_t _divRound(uint32_t n, uint32_t d){ // to put in header
    return (n + d/2)/d;
  }

  uint32_t _modulo(int32_t dividend, uint32_t divisor){ // to put in header
    int32_t mod = dividend%int32_t(divisor);
    return mod<0?mod+divisor:mod;
  }

};


#endif