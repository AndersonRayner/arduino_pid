// Library for a PID controller
#include "PID.h"

PID::PID()
{
      
}

void PID::init()
{
    // Reset the controller
    reset();
    
    // Return
    return;
}

void PID:: reset()
{
    // Reset the PID controller
    _Ierror = 0.0f;
    _error_prev = 0.0f;
    
    // All done
    return;
}

float PID::update(float error)
{
  // Calculate the timestep for the iteration
  static unsigned long t_prev = 0;
  float dt = (millis() - t_prev)/1000.f;
  t_prev = millis();
  
  // Reset the PID controller if it's been more than 1 s since the last run
  if (dt > 1000.0f)
  {
      reset();
  }

  // Calculate the new PID contributions
  _Perror = error;
  
  _Ierror = _Ierror + error*dt;
  _Ierror = constrain(_Ierror, -_Imax/fabsf(_kI), _Imax/fabsf(_kI));

  _Derror = (error - _error_prev)/fmaxf(dt,0.00001f);
  _Derror = constrain(_Derror,-_Dmax/fabsf(_kD),_Dmax/fabsf(_kD));
  
  _error_prev = error;
  
  // Calculate the new control force
  float control_force = _kP*_Perror + _kI*_Ierror + _kD*_Derror;

  // Debugging
  if (_debug)
  {
      SERIAL_DEBUG.print("PID :\n");
      SERIAL_DEBUG.print("\terr: "); SERIAL_DEBUG.print(error);   SERIAL_DEBUG.print("\n");
      SERIAL_DEBUG.print("\t  P: "); SERIAL_DEBUG.print(_kP*_Perror); SERIAL_DEBUG.print("\n");
      SERIAL_DEBUG.print("\t  I: "); SERIAL_DEBUG.print(_kI*_Ierror); SERIAL_DEBUG.print("\n");
      SERIAL_DEBUG.print("\t  D: "); SERIAL_DEBUG.print(_kD*_Derror); SERIAL_DEBUG.print("\n");
      SERIAL_DEBUG.print("\tPID: "); SERIAL_DEBUG.print(control_force); SERIAL_DEBUG.print("\n");
  }

  // All done
  return (control_force);  
}

// Variable setting functions
void PID::set_kP(float kP) { _kP = kP; return; }
void PID::set_kI(float kI) { _kI = kI; return; }
void PID::set_kD(float kD) { _kD = kD; return; }
void PID::set_Imax(float Imax) { _Imax = Imax; return; }
void PID::set_Dmax(float Dmax) { _Dmax = Dmax; return; }
