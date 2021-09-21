#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include "Math_functions.h"
#include "Filter.h"

class PID_controller{
    public:
        PID_controller();
        void set_gains(float _Kp, float _Ki, float _Kd);
        void set_Kp(float _Kp);
        float get_Kp();
        void set_Ki(float _Ki);
        float get_Ki();
        void set_Kd(float _Kd);
        float get_Kd();
        void set_I_max(float _I_max);
        float get_I_max();
        void set_output_max(float _pid_max);
        float get_output_max();
        void set_lpf_alpha(float _lpf_alpha);
        float get_lpf_alpha();
        float get_P();
        float get_I();
        float get_D();

        float calculate_output1(float x_des, float x, float dt);
        float calculate_output2(float x_des, float x, float dt);
        float calculate_output3(float x_des, float x, float dt);
        void reset();
        void initialize();

    private:
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;

        float I_max   = 999999999;
        float pid_max = 999999999;

        float de = 0;
        float e_pre = 0;
        float x_pre = 0;
        float P = 0;
        float I = 0;
        float D = 0;
        float pid = 0;
        float lpf_alpha = 0;

        Math_functions math_fun;
        Filter_LP lpf_filter;
};

#endif
