#include "FastAccelStepper.h"
#include "HardwareSerial.h"

//********************************** High level API methods **********************************

void Stepper::moveForNTracks(uint8_t nTracks, bool dir) {
  if (dir){
    _targetTrack = _modulo(int8_t(_targetTrack) + int8_t(nTracks),HALF_THE_NB_OF_TRACKS);
  }
  else{
    _targetTrack = _modulo(int8_t(_targetTrack) - int8_t(nTracks),HALF_THE_NB_OF_TRACKS);
  } 
  _computeNextMove();
}

void Stepper::moveTo(uint8_t track) {
  _targetTrack = _modulo(track, HALF_THE_NB_OF_TRACKS);
  _computeNextMove();
}
void Stepper::moveAtConstSpeed(bool dir){
  //to be implemented
}

void Stepper::stop(){
  //to be implemented
}

void Stepper::emergencyStop(){
  //to be implemented
}

void Stepper::handleCommands(){
  if (_isRunning){
    runCommand();
  }
}








