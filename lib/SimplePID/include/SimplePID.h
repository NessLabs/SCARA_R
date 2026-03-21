#ifndef _SIMPLEPID_H
#define _SIMPLEPID_H
 
#include <math.h>
 
class SimplePID
{
public:
    SimplePID();
    void setup(float gains[3], float dt_s, float out_min, float out_max);
    float calc(float error);
    void reset();
 
private:
    float Kp = 0.0f, Ki = 0.0f, Kd = 0.0f;
    float dt = 0.01f;
    float out_min = -1.0f, out_max = 1.0f;
    float prev_error = 0.0f;
    float integral = 0.0f;
};
 
#endif // _SIMPLEPID_H