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
        void set_kP(float kP) { _kP = kP; };
        void set_kI(float kI) { _kI = kI; };
        void set_kD(float kD) { _kD = kD; };
        
        void set_ff(float ff) { _ff = ff; };

        void set_Imax(float Imax) { _Imax = Imax; };
        void set_Dmax(float Dmax) { _Dmax = Dmax; };

        // === Get Parameters
        float get_kP()  { return (_kP); };
        float get_kI()  { return (_kI); };
        float get_kD()  { return (_kD); };
        
        float get_P() { return (get_kP()*_Perror); };
        float get_I() { return (get_kI()*_Ierror); };
        float get_D() { return (get_kD()*_Derror); };
        float get_ff() { return (_ff); };

        float get_Imax() { return (_Imax); };
        float get_Dmax() { return (_Dmax); };

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
        unsigned long _t_prev = 0;
        float _error_prev = 0.0f;

        float _Perror = 0.0f;
        float _Ierror = 0.0f;
        float _Derror = 0.0f;
        float _ff     = 0.0f; // Feedforward term
        
};
