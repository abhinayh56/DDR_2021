#include "PID_discrete_bilinear.h"

PID_discrete_bilinear::PID_discrete_bilinear(){
}

float PID_discrete_bilinear::calculate(float x0, float x){
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
