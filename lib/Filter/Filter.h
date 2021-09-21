#ifndef FILTER
#define FILTER

class Filter_LP{
  public:
    Filter_LP();
    float apply_LPF(float x);
    void set_alpha_LPF(float _alpha_LPF);
    float get_alpha_LPF();

  private:
    float alpha_LPF = 0.885;
    float y_prev = 0;
};

class Filter_HP{
  public:
    Filter_HP();
    float apply_HPF(float x);
    void set_alpha_HPF(float _alpha_HPF);
    float get_alpha_HPF();

  private:
    float alpha_HPF = 0.885;
    float y_prev = 0;
    float x_prev = 0;
};

#endif
