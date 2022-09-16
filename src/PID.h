// PID controller for Arduino

#include <Arduino.h>
#include <stdint.h>

#ifndef SERIAL_DEBUG
#define SERIAL_DEBUG Serial
#endif

class PID {
    public :
        PID();

        // === Functions
        void init();
        float update(float error);
        void reset();

        // === Parameter Setting
        void set_kP(float P);
        void set_kI(float I);
        void set_kD(float D);
        
        void set_Imax(float Imax);
        void set_Dmax(float Dmax);

        // === Debugging
        uint8_t _debug = 0;
        
    private:
    
        // PID gains
        float _kP = 3.0f;
        float _kI = 0.2f;
        float _kD = 0.0f;
        
        float _Imax = 1.0f;    // Maximum contribution of the I term on the output
        float _Dmax = 0.3f;    // Maximum contribution of the D term on the output

        // Things to remember between loops
        float _error_prev = 0.0f;

        float _Perror = 0.0f;
        float _Ierror = 0.0f;
        float _Derror = 0.0f;
        
};
