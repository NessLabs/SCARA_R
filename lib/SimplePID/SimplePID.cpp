#include "SimplePID.h"
 
SimplePID::SimplePID()
{
}
 
void SimplePID::setup(float gains[3], float dt_s, float out_min, float out_max)
{
    Kp = gains[0];
    Ki = gains[1];
    Kd = gains[2];
    dt = dt_s;
    this->out_min = out_min;
    this->out_max = out_max;
}
 
float SimplePID::calc(float error)
{
    float U = Kp * error;
    U += Kd * (error - prev_error) / dt;
    integral += (dt / 2.0f) * (error + prev_error);
 
    // Anti-windup: clamp integral contribution
    if (Ki != 0.0f)
        integral = fmaxf(out_min / Ki, fminf(out_max / Ki, integral));
 
    U += Ki * integral;
    prev_error = error;
 
    // Clamp output
    return fmaxf(out_min, fminf(out_max, U));
}
 
void SimplePID::reset()
{
    prev_error = 0.0f;
    integral = 0.0f;
}