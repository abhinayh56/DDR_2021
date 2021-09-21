#ifndef DISCRETE_PID
#define DISCRETE_PID


class Discrete_PID{
    public:
        Discrete_PID();
        float calculate_1(float x0, float x);
        float calculate_2(float x0, float x);

    private:
    /*
    // calculate_1 starts
        float Kp = 17.0; //17
        float Ki = 200.0;
        float Kd = 0.335;//0.34; //21.0*0.022572525*0.725;
        float fc = 40.0;
        //float tau = 1.0/(6.283185307179586476925286766559*fc);
        float N = 6.283185307179586476925286766559*fc; //1.0/tau;
        float Ts = 1/500.0;

        float a = Kp;
        float b = Ki*Ts/2.0;
        float c = 2.0*Kd*N/(N*Ts+2.0);
        float d = (N*Ts-2.0)/(N*Ts+2.0);

        float A = a + b + c;
        float B = a*(d-1.0) + b*(d+1.0) - 2.0*c;
        float C = d*(b-a) + c;

        float e_k_1 = 0;
        float e_k_2 = 0;
        
        float u_k_1 = 0;
        float u_k_2 = 0;

        float u_max = 250.0;
    // calculate_1 ends
    */

    // calculate_2 starts
    float Kp = 22; //26.0;
    float Kd = 0.343;
    float Ki = 0*230.0;

    float tau_d = Kd/Kp; //0.022572525*0.725; //0.01153846153846153846153846153846;
    float tau_i = Kp/Ki; //0.0902901; //0.26;
    float T_0   = 1.0/500.0;

    float e_k   = 0;
    float e_k_1 = 0;
    float e_k_2 = 0;
    float u_k_1 = 0;
    float u_k   = 0;
    float u_max = 250.0;
    // calculate_2 ends
};

#endif
