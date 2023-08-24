// PID controller for Arduino

#include <Arduino.h>
#include <stdint.h>
#include <limits>

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

        void set_OutputMin(float output_min) { _output_min = output_min; };
        void set_OutputMax(float output_max) { _output_max = output_max; };

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

        float get_OutputMin() { return (_output_min); };
        float get_OutputMax() { return (_output_max); };

        // === Debugging
        uint8_t _debug = 0;
        
    private:
    
        // PID gains
        float _kP = 3.0f;
        float _kI = 0.2f;
        float _kD = 0.0f;
        
        float _Imax = 1.0f;    // Maximum contribution of the I term on the output
        float _Dmax = 0.3f;    // Maximum contribution of the D term on the output

        float _output_min = -std::numeric_limits<float>::infinity();
        float _output_max =  std::numeric_limits<float>::infinity();

        // Things to remember between loops
        unsigned long _t_prev = 0;
        float _Ierror_prev = 0.0f;
        float _error_prev = 0.0f;

        float _Perror = 0.0f;
        float _Ierror = 0.0f;
        float _Derror = 0.0f;
        float _ff     = 0.0f; // Feedforward term
        
};
