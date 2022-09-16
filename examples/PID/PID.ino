// Test code for the PID controller

// Libraries
#include "PID.h"

// Objects
PID controller;

// Globals
const float dt = 0.01f;    // Timestep in [ s ]
const float mass = 1.0f;   // Mass in [ kg ]

unsigned long t_prev_impulse;
float velocity;

void setup()
{
  // Initialise comm ports
  Serial.begin(115200);
  
  // Initialise the controller
  controller.init();
  controller.reset();
  controller._debug = 0;

  // Controller tuning
  controller.set_kP(20.0f);
  controller.set_kI(50.0f);
  controller.set_kD(0.8f);

  controller.set_Imax(50.0f);
  controller.set_Dmax(50.0f);

  // Set up the simulation
  t_prev_impulse = millis();
  velocity = 0.0f;
  
}

void loop()
{
  unsigned long loop_start = millis();

  // Provide an impulse if required
  if (millis() > t_prev_impulse + 2000)
  {
    velocity = 10.0f;
    t_prev_impulse = millis();
    controller.reset();
    
  }

  // Calculate error
  float vel_error = 0.0f - velocity;
  
  // Update controller
  float force = controller.update(vel_error);
  force = constrain(force,-100.0f,100.0f);
  
  // Calculate dynamics
  float accel = force / mass;
  velocity = velocity + accel*dt + 0.3f;

  // Plot results
  Serial.print("0,"); // Target speed
  Serial.print(velocity); // Actual speed
  Serial.print("\n");

  // Loop timing
  while (millis() < (loop_start + dt*1000.0f))
  {
    // do nothing
    
  }
  
}
