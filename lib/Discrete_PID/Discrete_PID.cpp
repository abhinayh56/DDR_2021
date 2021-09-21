#include "Discrete_PID.h"

Discrete_PID::Discrete_PID(){
}

/*
float Discrete_PID::calculate_1(float x0, float x){
    float e_k = x0-x;
    float u_k = (1.0-d)*u_k_1 + d*u_k_2 + A*e_k + B*e_k_1 + C*e_k_2;

    if(u_k>u_max){
        u_k = u_max;
    }
    else if(u_k<-u_max){
        u_k = -u_max;
    }

    e_k_2 = e_k_1;
    e_k_1 = e_k;

    u_k_2 = u_k_1;
    u_k_1 = u_k;

    return u_k;
}
*/

float Discrete_PID::calculate_2(float x0, float x){
    float q0 =  Kp*(1.0 + tau_d/T_0);
    float q1 = -Kp*(1.0 + 2.0*tau_d/T_0 - T_0/tau_i);
    float q2 = Kp*tau_d/T_0;

    e_k = x0 - x;
    float delta_u = q0*e_k + q1*e_k_1 + q2*e_k_2;

    u_k = u_k_1 + delta_u;
    if(u_k>u_max){
        u_k = u_max;
    }
    else if(u_k<-u_max){
        u_k = -u_max;
    }
    
    u_k_1 = u_k;

    e_k_2 = e_k_1;
    e_k_1 = e_k;

    return u_k;
}
